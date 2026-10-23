import sys
import types
from pathlib import Path
from types import SimpleNamespace

import pytest

_PACKAGE_DIR = Path(__file__).parents[2] / "linkerhand_retarget"
if str(_PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_DIR))


def _install_ros_stubs(monkeypatch):
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = type("Node", (), {})
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.JointState = type("JointState", (), {})
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    for name in (
        "String",
        "Int32MultiArray",
        "Header",
        "Float32MultiArray",
        "MultiArrayLayout",
        "MultiArrayDimension",
    ):
        setattr(std_msgs_msg, name, type(name, (), {}))

    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy_node)
    monkeypatch.setitem(sys.modules, "sensor_msgs", sensor_msgs)
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", sensor_msgs_msg)
    monkeypatch.setitem(sys.modules, "std_msgs", std_msgs)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", std_msgs_msg)


class FakeHandCore:
    hand_numjoints_r = 20
    hand_numjoints_l = 20


class FakeNode:
    def create_publisher(self, *args, **kwargs):
        return SimpleNamespace()

    def create_subscription(self, *args, **kwargs):
        return SimpleNamespace()

    def create_timer(self, *args, **kwargs):
        return SimpleNamespace()

    def get_logger(self):
        return SimpleNamespace(info=lambda *args, **kwargs: None)


@pytest.mark.parametrize("hand_class_name", ["RightHand", "LeftHand"])
def test_l30_uncalibrated_fallback_keeps_its_qpos_mapping(monkeypatch, hand_class_name):
    from linkerhand_retarget.motion.linkerforce.hand import linkerforce_l30, linkerforce_o30

    hand = getattr(linkerforce_l30, hand_class_name)(FakeHandCore())
    hand.smooth_enabled = False
    captured = {}
    monkeypatch.setattr(
        linkerforce_o30,
        "_sync_o30_fallback_arcs",
        lambda *_args: pytest.fail("L30 must not use O30 fallback arcs"),
    )
    monkeypatch.setattr(
        hand,
        "_set_g_jointpositions_from_qpos",
        lambda qpos: captured.update(qpos=list(qpos)),
    )

    pose = [0.0] * 21
    pose[1] = 4.0
    pose[2] = 0.4
    hand.joint_update(pose)

    assert captured["qpos"][17] == pytest.approx(-1.0)


L30_EXPECTED_TOPIC_NAMES = (
    "拇指指根",
    "拇指指尖",
    "拇指侧摆",
    "拇指旋转",
    "无名指侧摆",
    "无名指指尖",
    "无名指指根",
    "中指指根",
    "中指指尖",
    "小指指根",
    "小指指尖",
    "小指侧摆",
    "中指侧摆",
    "食指侧摆",
    "食指指根",
    "食指指尖",
    "手腕",
    "未使用17",
)


def test_linkerforce_retarget_routes_l30_to_independent_hand_classes(monkeypatch):
    _install_ros_stubs(monkeypatch)

    from linkerhand.constants import RobotName
    from linkerhand_retarget.motion.linkerforce.hand.linkerforce_l30 import (
        LeftHand,
        RightHand,
    )
    from linkerhand_retarget.motion.linkerforce.retarget import Retarget

    retarget = Retarget(
        FakeNode(),
        righthand=RobotName.l30,
        lefthand=RobotName.l30,
        handcore=FakeHandCore(),
        lefthandpubprint=False,
        righthandpubprint=False,
        auto_detect=False,
        isgetdebug=False,
        baseconfig={},
    )

    assert isinstance(retarget.righthand, RightHand)
    assert isinstance(retarget.lefthand, LeftHand)


def test_l30_mapping_uses_declared_topic_order_and_skips_wrist_for_display():
    from linkerhand_retarget.motion.linkerforce.hand.linkerforce_l30 import RightHand

    hand = RightHand(FakeHandCore())

    assert len(hand.g_jointpositions) == 18
    assert tuple(hand.topic_joint_names) == L30_EXPECTED_TOPIC_NAMES
    assert len(hand.mujoco_joint_arc_indices) == 21
    assert hand.mujoco_joint_arc_indices[0] is None
    assert hand.mujoco_joint_arc_indices[1:] == (
        16, 17, 18, 18,
        12, 13, 14, 14,
        8, 9, 10, 10,
        4, 5, 6, 6,
        1, 0, 2, 3,
    )


def test_l30_mapping_outputs_declared_motor_ranges():
    from linkerhand_retarget.motion.linkerforce.hand.linkerforce_l30 import RightHand

    hand = RightHand(FakeHandCore())
    hand.smooth_enabled = False
    hand.g_jointpositions_arc = [0.0] * 20
    hand.g_jointpositions_arc[2] = 1.021
    hand.g_jointpositions_arc[3] = 1.484
    hand.g_jointpositions_arc[12] = 0.2
    hand.g_jointpositions_arc[16] = -0.2

    hand._set_g_jointpositions_from_arc()

    assert hand.g_jointpositions[0] == 1000
    assert hand.g_jointpositions[1] == 1500
    assert hand.g_jointpositions[4] == 200
    assert hand.g_jointpositions[11] == -200
    assert hand.g_jointpositions[16] == 0
    assert hand.g_jointpositions[17] == 0


def test_l30_keeps_its_own_mapper_and_product_config():
    from linkerhand_retarget.motion.linkerforce.hand import linkerforce_l30
    from linkerhand_retarget.motion.linkerforce.config.lffg1.l30_config import (
        FINGER_CONFIGS as L30_FINGER_CONFIGS,
        MOTOR_CONSTRAINTS as L30_MOTOR_CONSTRAINTS,
    )

    hand = linkerforce_l30.RightHand(FakeHandCore())

    assert linkerforce_l30.FINGER_CONFIGS is L30_FINGER_CONFIGS
    assert hand.product_config_module.__name__.endswith("config.lffg1.l30_config")
    assert len(hand.multi_state_mapper.finger_configs) == 16
    assert len(hand.motor_constraints) == 18
    assert len(L30_MOTOR_CONSTRAINTS["left"]) == 18

    hand.set_product_version("LFFG2-L30-V01", "3.1.3")
    assert hand.product_family == "LFFG2"
    assert hand.glove_version == "v1"
    assert hand.product_config_module.__name__.endswith("config.lffg2.l30_config")
    assert isinstance(hand.multi_state_mapper.finger_configs["thumb_rotate"]["weights"], list)
