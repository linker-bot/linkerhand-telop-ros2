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


def _o30_limits_for(hand_class_name):
    import linkerhand_retarget.motion.linkerforce.hand.linkerforce_o30 as linkerforce_o30

    if hand_class_name == "LeftHand":
        return linkerforce_o30.O30_URDF_JOINT_LIMITS_LEFT
    return linkerforce_o30.O30_URDF_JOINT_LIMITS_RIGHT


def _o30_disable_motor_constraints(hand):
    hand.motor_constraints = [{"enabled": False} for _ in range(20)]


@pytest.mark.parametrize("family", ["lffg1", "lffg2"])
@pytest.mark.parametrize("side", ["left", "right"])
def test_o30_ring_pose_targets_follow_calibration_reference(family, side):
    import importlib

    config = importlib.import_module(
        f"linkerhand_retarget.motion.linkerforce.config.{family}.o30_config"
    )
    open_pose = getattr(config, f"ROBOT_ORIGINAL_{side.upper()}")
    opose = getattr(config, f"ROBOT_OPOSE_{side.upper()}")
    fist = getattr(config, f"ROBOT_FIST_{side.upper()}")

    assert open_pose[12] == pytest.approx(0.29)
    assert opose[12] == pytest.approx(0.0)
    assert fist[12] == pytest.approx(0.0)


@pytest.mark.parametrize("family", ["lffg1", "lffg2"])
@pytest.mark.parametrize("side", ["left", "right"])
def test_o30_index_roll_open_target_matches_real_hand_calibration(family, side):
    import importlib

    config = importlib.import_module(
        f"linkerhand_retarget.motion.linkerforce.config.{family}.o30_config"
    )
    open_pose = getattr(config, f"ROBOT_ORIGINAL_{side.upper()}")

    assert open_pose[4] == pytest.approx(-0.40)


@pytest.mark.parametrize("family", ["lffg1", "lffg2"])
@pytest.mark.parametrize("side", ["left", "right"])
def test_o30_middle_roll_targets_match_real_hand_calibration(family, side):
    import importlib

    config = importlib.import_module(
        f"linkerhand_retarget.motion.linkerforce.config.{family}.o30_config"
    )
    open_pose = getattr(config, f"ROBOT_ORIGINAL_{side.upper()}")
    opose = getattr(config, f"ROBOT_OPOSE_{side.upper()}")

    assert open_pose[8] == pytest.approx(0.11)
    assert opose[8] == pytest.approx(-0.10)


class FakeHandCore:
    hand_numjoints_r = 20
    hand_numjoints_l = 20

    def trans_to_motor_right(self, qpos):
        raise AssertionError("O30 should not call HandCore.trans_to_motor_right")

    def trans_to_motor_left(self, qpos):
        raise AssertionError("O30 should not call HandCore.trans_to_motor_left")


class FakeNode:
    def create_publisher(self, *args, **kwargs):
        return SimpleNamespace()

    def create_subscription(self, *args, **kwargs):
        return SimpleNamespace()

    def create_timer(self, *args, **kwargs):
        return SimpleNamespace()

    def get_logger(self):
        return SimpleNamespace(info=lambda *args, **kwargs: None)


O30_EXPECTED_TOPIC_NAMES = (
    "拇指横滚",
    "拇指侧摆",
    "食指侧摆",
    "中指侧摆",
    "无名指侧摆",
    "小指侧摆",
    "拇指指根",
    "食指指根",
    "中指指根",
    "无名指指根",
    "小指指根",
    "食指指中",
    "中指指中",
    "无名指指中",
    "小指指中",
    "拇指指尖",
    "食指指尖",
    "中指指尖",
    "无名指指尖",
    "小指指尖",
)


def test_linkerforce_retarget_routes_o30_to_independent_hand_classes(monkeypatch):
    _install_ros_stubs(monkeypatch)

    from linkerhand_retarget.motion.linkerforce.hand.linkerforce_o30 import (
        LeftHand,
        RightHand,
    )
    from linkerhand_retarget.motion.linkerforce.retarget import Retarget
    from linkerhand.constants import RobotName

    retarget = Retarget(
        FakeNode(),
        righthand=RobotName.o30,
        lefthand=RobotName.o30,
        handcore=FakeHandCore(),
        lefthandpubprint=False,
        righthandpubprint=False,
        auto_detect=False,
        isgetdebug=False,
        baseconfig={},
    )

    assert isinstance(retarget.righthand, RightHand)
    assert isinstance(retarget.lefthand, LeftHand)


@pytest.mark.parametrize("hand_class_name", ["RightHand", "LeftHand"])
def test_o30_control_output_uses_local_urdf_normalization(hand_class_name, monkeypatch):
    import linkerhand_retarget.motion.linkerforce.hand.linkerforce_o30 as linkerforce_o30

    hand = getattr(linkerforce_o30, hand_class_name)(FakeHandCore())
    hand.smooth_enabled = False
    hand.motor_constraints = [{"enabled": False} for _ in range(20)]
    hand.calibrationoriginal, hand.calibrationopose, hand.calibrationfistpose = _state_calibration()
    hand.initialize_mapper()
    monkeypatch.setattr(hand.multi_state_mapper, "map_glove_to_robot", lambda _joint_arc: [0.0] * 20)

    hand.joint_update([0.0] * 21)

    assert tuple(getattr(hand, "topic_joint_names", ())) == O30_EXPECTED_TOPIC_NAMES
    assert hand.g_jointpositions[0] == 0
    assert hand.g_jointpositions[1] == 0
    # All four finger rolls use the physical direction of the O30 hand.
    limits = _o30_limits_for(hand_class_name)
    normal_zero_outputs = []
    for arc_idx in (4, 8, 12, 16):
        lower, upper = limits[arc_idx]
        clamped_zero = min(upper, max(lower, 0))
        normal_zero_outputs.append(
            int(round((clamped_zero - lower) / (upper - lower) * 255))
        )
    assert hand.g_jointpositions[2:6] == normal_zero_outputs
    assert hand.g_jointpositions[6:] == [0] * 14


@pytest.mark.parametrize("hand_class_name", ["RightHand", "LeftHand"])
def test_o30_local_arc_to_motor_normalizes_urdf_limits(hand_class_name):
    import linkerhand_retarget.motion.linkerforce.hand.linkerforce_o30 as linkerforce_o30

    hand = getattr(linkerforce_o30, hand_class_name)(FakeHandCore())
    hand.smooth_enabled = False
    _o30_disable_motor_constraints(hand)

    # 每个关节输入其 URDF 限位端值（正向关节超上限、反转关节低于下限），
    # 使 clamp 后恰好到达限位，期望输出满值 255
    limits = _o30_limits_for(hand_class_name)
    reversed_indices = ()
    hand.g_jointpositions_arc = [
        limits[i][1] + 0.01 if i not in reversed_indices else limits[i][0] - 0.01
        for i in range(20)
    ]
    hand._set_g_jointpositions_from_arc()

    assert hand.g_jointpositions[0] == 255
    assert hand.g_jointpositions[1] == 255
    assert hand.g_jointpositions[2:6] == [255, 255, 255, 255]
    assert hand.g_jointpositions[6] == 255
    assert hand.g_jointpositions[7:20] == [255] * 13


@pytest.mark.parametrize("hand_class_name", ["RightHand", "LeftHand"])
def test_o30_thumb_rotate_motor_output_uses_physical_direction(hand_class_name):
    import linkerhand_retarget.motion.linkerforce.hand.linkerforce_o30 as linkerforce_o30

    hand = getattr(linkerforce_o30, hand_class_name)(FakeHandCore())
    hand.smooth_enabled = False
    _o30_disable_motor_constraints(hand)

    hand.g_jointpositions_arc = [0.0] * 20
    hand.g_jointpositions_arc[0] = 0.0
    hand._set_g_jointpositions_from_arc()
    assert hand.g_jointpositions[0] == 0

    hand.g_jointpositions_arc = [0.0] * 20
    hand.g_jointpositions_arc[0] = _o30_limits_for(hand_class_name)[0][1] + 0.01
    hand._set_g_jointpositions_from_arc()
    assert hand.g_jointpositions[0] == 255


@pytest.mark.parametrize("hand_class_name", ["RightHand", "LeftHand"])
def test_o30_four_finger_roll_outputs_follow_motor_directions(hand_class_name):
    import linkerhand_retarget.motion.linkerforce.hand.linkerforce_o30 as linkerforce_o30

    hand = getattr(linkerforce_o30, hand_class_name)(FakeHandCore())
    hand.smooth_enabled = False
    _o30_disable_motor_constraints(hand)

    limits = _o30_limits_for(hand_class_name)
    roll_joints = (
        (4, 2, limits[4][0], limits[4][1], 0, 255),
        (8, 3, limits[8][0], limits[8][1], 0, 255),
        (12, 4, limits[12][0], limits[12][1], 0, 255),
        (16, 5, limits[16][0], limits[16][1], 0, 255),
    )
    for arc_idx, motor_idx, lower, upper, lower_output, upper_output in roll_joints:
        hand.g_jointpositions_arc = [0.0] * 20
        hand.g_jointpositions_arc[arc_idx] = lower
        hand._set_g_jointpositions_from_arc()
        assert hand.g_jointpositions[motor_idx] == lower_output

        hand.g_jointpositions_arc = [0.0] * 20
        hand.g_jointpositions_arc[arc_idx] = upper
        hand._set_g_jointpositions_from_arc()
        assert hand.g_jointpositions[motor_idx] == upper_output


@pytest.mark.parametrize("hand_class_name", ["RightHand", "LeftHand"])
def test_o30_index_roll_open_angle_outputs_zero_motor(hand_class_name):
    import linkerhand_retarget.motion.linkerforce.hand.linkerforce_o30 as linkerforce_o30

    hand = getattr(linkerforce_o30, hand_class_name)(FakeHandCore())
    hand.smooth_enabled = False
    _o30_disable_motor_constraints(hand)
    hand.g_jointpositions_arc = [0.0] * 20
    hand.g_jointpositions_arc[4] = -0.40

    hand._set_g_jointpositions_from_arc()

    assert hand.g_jointpositions[2] == 0


@pytest.mark.parametrize("hand_class_name", ["RightHand", "LeftHand"])
def test_o30_ring_opose_angle_maps_to_urdf_motor_value(hand_class_name):
    import linkerhand_retarget.motion.linkerforce.hand.linkerforce_o30 as linkerforce_o30

    hand = getattr(linkerforce_o30, hand_class_name)(FakeHandCore())
    hand.smooth_enabled = False
    _o30_disable_motor_constraints(hand)
    hand.g_jointpositions_arc = [0.0] * 20
    hand.g_jointpositions_arc[12] = 0.0

    hand._set_g_jointpositions_from_arc()

    lower, upper = _o30_limits_for(hand_class_name)[12]
    clamped_opose = min(upper, max(lower, 0.0))
    expected_motor = int(round(
        (clamped_opose - lower) / (upper - lower) * 255
    ))
    assert hand.g_jointpositions[4] == expected_motor


@pytest.mark.parametrize("hand_class_name", ["RightHand", "LeftHand"])
@pytest.mark.parametrize(
    "arc_idx, motor_idx, calibration_angle",
    [(12, 4, 0.29), (16, 5, 0.31)],
)
def test_o30_real_roll_calibration_angles_reach_full_motor_value(
    hand_class_name, arc_idx, motor_idx, calibration_angle
):
    import linkerhand_retarget.motion.linkerforce.hand.linkerforce_o30 as linkerforce_o30

    hand = getattr(linkerforce_o30, hand_class_name)(FakeHandCore())
    hand.smooth_enabled = False
    _o30_disable_motor_constraints(hand)
    hand.g_jointpositions_arc = [0.0] * 20
    hand.g_jointpositions_arc[arc_idx] = calibration_angle

    hand._set_g_jointpositions_from_arc()

    assert hand.g_jointpositions[motor_idx] == 255


@pytest.mark.parametrize("hand_class_name", ["RightHand", "LeftHand"])
def test_o30_middle_roll_open_and_opose_targets_use_real_motor_values(hand_class_name):
    import linkerhand_retarget.motion.linkerforce.hand.linkerforce_o30 as linkerforce_o30

    hand = getattr(linkerforce_o30, hand_class_name)(FakeHandCore())
    hand.smooth_enabled = False
    _o30_disable_motor_constraints(hand)

    limits = _o30_limits_for(hand_class_name)
    hand.g_jointpositions_arc = [0.0] * 20
    hand.g_jointpositions_arc[8] = 0.11
    hand._set_g_jointpositions_from_arc()
    assert hand.g_jointpositions[3] == 255

    hand.g_jointpositions_arc = [0.0] * 20
    hand.g_jointpositions_arc[8] = -0.10
    hand._set_g_jointpositions_from_arc()
    lower, upper = limits[8]
    clamped_opose = min(upper, max(lower, -0.10))
    expected_opose = int(round((clamped_opose - lower) / (upper - lower) * 255))
    assert hand.g_jointpositions[3] == expected_opose


@pytest.mark.parametrize("hand_class_name", ["RightHand", "LeftHand"])
def test_o30_thumb_mcp_uses_actual_control_output_order(hand_class_name):
    import linkerhand_retarget.motion.linkerforce.hand.linkerforce_o30 as linkerforce_o30

    hand = getattr(linkerforce_o30, hand_class_name)(FakeHandCore())
    hand.smooth_enabled = False
    hand.motor_constraints = [{"enabled": False} for _ in range(20)]

    limits = _o30_limits_for(hand_class_name)
    hand.g_jointpositions_arc = [0.0] * 20
    hand.g_jointpositions_arc[0] = limits[0][1] + 0.01
    hand.g_jointpositions_arc[2] = limits[2][1] + 0.01
    hand._set_g_jointpositions_from_arc()

    assert hand.g_jointpositions[0] == 255
    assert hand.g_jointpositions[6] == 255


def test_linkerforce_retarget_uses_o30_topic_joint_names(monkeypatch):
    _install_ros_stubs(monkeypatch)

    from linkerhand_retarget.motion.linkerforce.retarget import Retarget

    retarget = Retarget.__new__(Retarget)
    hand = SimpleNamespace(
        g_jointpositions=[0] * 20,
        topic_joint_names=O30_EXPECTED_TOPIC_NAMES,
    )

    assert retarget._joint_names_for(hand) == list(O30_EXPECTED_TOPIC_NAMES)


def _state_calibration():
    original = [0.0] * 21
    opose = [1.0] * 21
    fist = [2.0] * 21
    return original, opose, fist


def _settle_pose(hand, pose, frames=20):
    for _ in range(frames):
        hand.joint_update(pose)
    return hand.g_jointpositions_arc[1]


def _pose_with_thumb_roll(value):
    pose = [0.0] * 21
    pose[1] = value
    return pose


@pytest.mark.parametrize(
    "hand_class_name, yaw_delta",
    [("RightHand", 0.4), ("LeftHand", -0.4)],
)
def test_o30_uncalibrated_thumb_yaw_updates_motor_and_simulation(hand_class_name, yaw_delta):
    import linkerhand_retarget.motion.linkerforce.hand.linkerforce_o30 as o30
    from linkerhand_retarget.mujoco_display import extract_mujoco_joint_positions

    hand = getattr(o30, hand_class_name)(FakeHandCore())
    hand.smooth_enabled = False
    _o30_disable_motor_constraints(hand)

    pose = [0.0] * 21
    pose[1] = 4.0
    hand.joint_update(pose)
    assert hand.g_jointpositions_arc[1] == pytest.approx(0.0)

    pose[1] += yaw_delta
    pose[2] = 0.7  # Thumb root flexion must not drive thumb yaw.
    hand.joint_update(pose)

    assert hand.g_jointpositions_arc[1] == pytest.approx(1.0)
    yaw_upper = _o30_limits_for(hand_class_name)[1][1]
    assert hand.g_jointpositions[1] == round(255 / yaw_upper)
    positions = extract_mujoco_joint_positions(
        hand.handcore,
        hand.hand_side,
        ["thumb_cmc_roll", "thumb_cmc_yaw"],
        hand,
    )
    assert positions["thumb_cmc_yaw"] == pytest.approx(1.0)

    pose[2] += 0.3
    hand.joint_update(pose)
    assert hand.g_jointpositions_arc[1] == pytest.approx(1.0)


def test_o30_right_thumb_cmc_yaw_uses_right_hand_motion_direction():
    from linkerhand_retarget.motion.linkerforce.config.lffg1.o30_config import (
        ROBOT_FIST_RIGHT,
        ROBOT_OPOSE_RIGHT,
        ROBOT_ORIGINAL_RIGHT,
    )
    from linkerhand_retarget.motion.linkerforce.hand.linkerforce_o30 import RightHand

    hand = RightHand(FakeHandCore())
    hand.smooth_enabled = False
    hand.calibrationoriginal, hand.calibrationopose, hand.calibrationfistpose = _state_calibration()
    hand.initialize_mapper()

    open_yaw = _settle_pose(hand, hand.calibrationoriginal)
    opose_yaw = _settle_pose(hand, hand.calibrationopose)
    fist_yaw = _settle_pose(hand, hand.calibrationfistpose)

    assert hand.mujoco_joint_arc_signs[1] == 1.0
    assert open_yaw == pytest.approx(ROBOT_ORIGINAL_RIGHT[1])
    assert opose_yaw == pytest.approx(ROBOT_OPOSE_RIGHT[1])
    assert fist_yaw == pytest.approx(ROBOT_FIST_RIGHT[1])
    assert open_yaw < opose_yaw < fist_yaw


def test_o30_right_thumb_cmc_roll_uses_right_hand_motion_direction():
    from linkerhand_retarget.motion.linkerforce.config.lffg1.o30_config import (
        ROBOT_FIST_RIGHT,
        ROBOT_OPOSE_RIGHT,
        ROBOT_ORIGINAL_RIGHT,
    )
    from linkerhand_retarget.motion.linkerforce.hand.linkerforce_o30 import RightHand

    hand = RightHand(FakeHandCore())
    hand.smooth_enabled = False
    hand.calibrationoriginal = _pose_with_thumb_roll(5.0)
    hand.calibrationopose = _pose_with_thumb_roll(4.0)
    hand.calibrationfistpose = _pose_with_thumb_roll(3.0)
    hand.initialize_mapper()

    hand.joint_update(hand.calibrationoriginal)
    open_roll = hand.g_jointpositions_arc[0]
    hand.joint_update(hand.calibrationopose)
    opose_roll = hand.g_jointpositions_arc[0]
    hand.joint_update(hand.calibrationfistpose)
    fist_roll = hand.g_jointpositions_arc[0]

    assert hand.mujoco_joint_arc_signs[0] == 1.0
    assert open_roll == pytest.approx(ROBOT_ORIGINAL_RIGHT[0])
    assert opose_roll == pytest.approx(ROBOT_OPOSE_RIGHT[0])
    assert fist_roll == pytest.approx(ROBOT_FIST_RIGHT[0])
    assert open_roll < opose_roll < fist_roll


def test_o30_right_thumb_cmc_roll_maps_realtime_open_to_fist_direction():
    from linkerhand_retarget.motion.linkerforce.config.lffg1.o30_config import (
        ROBOT_FIST_RIGHT,
        ROBOT_ORIGINAL_RIGHT,
    )
    from linkerhand_retarget.motion.linkerforce.hand.linkerforce_o30 import RightHand

    hand = RightHand(FakeHandCore())
    hand.smooth_enabled = False
    hand.calibrationoriginal = _pose_with_thumb_roll(0.7117448943325635)
    hand.calibrationopose = _pose_with_thumb_roll(1.3007013686522553)
    hand.calibrationfistpose = _pose_with_thumb_roll(1.5951796058121013)
    hand.initialize_mapper()

    realtime_open = _pose_with_thumb_roll(0.75)
    for _ in range(10):
        hand.joint_update(realtime_open)
    open_roll = hand.g_jointpositions_arc[0]

    realtime_fist = _pose_with_thumb_roll(1.55)
    for _ in range(20):
        hand.joint_update(realtime_fist)
    fist_roll = hand.g_jointpositions_arc[0]

    assert ROBOT_ORIGINAL_RIGHT[0] <= open_roll <= ROBOT_FIST_RIGHT[0]
    assert ROBOT_ORIGINAL_RIGHT[0] <= fist_roll <= ROBOT_FIST_RIGHT[0]
    assert open_roll < fist_roll


def test_o30_right_thumb_cmc_yaw_maps_realtime_open_to_fist_direction():
    from linkerhand_retarget.motion.linkerforce.config.lffg1.o30_config import (
        ROBOT_FIST_RIGHT,
        ROBOT_ORIGINAL_RIGHT,
    )
    from linkerhand_retarget.motion.linkerforce.hand.linkerforce_o30 import RightHand

    hand = RightHand(FakeHandCore())
    hand.smooth_enabled = False
    hand.calibrationoriginal = _pose_with_thumb_roll(0.7117448943325635)
    hand.calibrationopose = _pose_with_thumb_roll(1.3007013686522553)
    hand.calibrationfistpose = _pose_with_thumb_roll(1.5951796058121013)
    hand.initialize_mapper()

    realtime_open = _pose_with_thumb_roll(0.75)
    for _ in range(10):
        hand.joint_update(realtime_open)
    open_yaw = hand.g_jointpositions_arc[1]

    realtime_fist = _pose_with_thumb_roll(1.55)
    for _ in range(20):
        hand.joint_update(realtime_fist)
    fist_yaw = hand.g_jointpositions_arc[1]

    assert ROBOT_ORIGINAL_RIGHT[1] <= open_yaw <= ROBOT_FIST_RIGHT[1]
    assert ROBOT_ORIGINAL_RIGHT[1] <= fist_yaw <= ROBOT_FIST_RIGHT[1]
    assert open_yaw < fist_yaw


def test_o30_right_hand_keeps_dynamicweight_output_without_thumb_overrides(monkeypatch):
    from linkerhand_retarget.motion.linkerforce.hand.linkerforce_o30 import RightHand

    hand = RightHand(FakeHandCore())
    hand.smooth_enabled = False
    hand.calibrationoriginal = [0.0] * 21
    hand.calibrationopose = [1.0] * 21
    hand.calibrationfistpose = [2.0] * 21
    hand.initialize_mapper()

    base_arc = [round(index * 0.1, 3) for index in range(20)]

    def fake_map_glove_to_robot(_joint_arc):
        return list(base_arc)

    monkeypatch.setattr(hand.multi_state_mapper, "map_glove_to_robot", fake_map_glove_to_robot)

    hand.joint_update([0.5] * 21)

    assert hand.g_jointpositions_arc[0] == pytest.approx(base_arc[0])
    assert hand.g_jointpositions_arc[1] == pytest.approx(base_arc[1])


def test_o30_side_roll_mapping_uses_opose_anchor_without_enabling_fist():
    from linkerhand_retarget.motion.linkerforce.config.lffg1.o30_config import (
        MULTI_SEGMENT_CONFIG,
        ROBOT_ORIGINAL_RIGHT,
        ROBOT_OPOSE_RIGHT,
    )
    from linkerhand_retarget.motion.linkerforce.hand.linkerforce_o30 import RightHand

    assert MULTI_SEGMENT_CONFIG["states"] == ["original", "opose"]

    hand = RightHand(FakeHandCore())
    hand.smooth_enabled = False
    hand.calibrationoriginal, hand.calibrationopose, hand.calibrationfistpose = _state_calibration()
    hand.initialize_mapper()

    assert hand.multi_state_mapper.state_order == ["original", "opose"]

    hand.joint_update(hand.calibrationoriginal)
    original_arcs = list(hand.g_jointpositions_arc)
    hand.joint_update(hand.calibrationopose)
    opose_arcs = list(hand.g_jointpositions_arc)

    for arc_idx in (4, 8, 12, 16):
        assert original_arcs[arc_idx] == pytest.approx(ROBOT_ORIGINAL_RIGHT[arc_idx])
        assert opose_arcs[arc_idx] == pytest.approx(ROBOT_OPOSE_RIGHT[arc_idx])

    for arc_idx in (4, 12, 16):
        assert ROBOT_OPOSE_RIGHT[arc_idx] != pytest.approx(ROBOT_ORIGINAL_RIGHT[arc_idx])
        assert opose_arcs[arc_idx] != pytest.approx(original_arcs[arc_idx])


def test_o30_right_ring_side_roll_uses_urdf_open_and_opose_targets():
    from linkerhand_retarget.motion.linkerforce.config.lffg1.o30_config import (
        ROBOT_FIST_RIGHT,
        ROBOT_ORIGINAL_RIGHT,
        ROBOT_OPOSE_RIGHT,
    )
    from linkerhand_retarget.motion.linkerforce.hand.linkerforce_o30 import RightHand

    hand = RightHand(FakeHandCore())
    hand.smooth_enabled = False
    hand.calibrationoriginal, hand.calibrationopose, hand.calibrationfistpose = _state_calibration()
    hand.initialize_mapper()

    hand.joint_update(hand.calibrationoriginal)
    original_ring_roll = hand.g_jointpositions_arc[12]
    hand.joint_update(hand.calibrationopose)
    opose_ring_roll = hand.g_jointpositions_arc[12]
    hand.joint_update(hand.calibrationfistpose)
    fist_ring_roll = hand.g_jointpositions_arc[12]

    assert ROBOT_ORIGINAL_RIGHT[12] == pytest.approx(0.29)
    assert ROBOT_OPOSE_RIGHT[12] == pytest.approx(0.0)
    assert ROBOT_FIST_RIGHT[12] == pytest.approx(0.0)
    assert original_ring_roll == pytest.approx(ROBOT_ORIGINAL_RIGHT[12])
    assert opose_ring_roll == pytest.approx(ROBOT_OPOSE_RIGHT[12])
    assert fist_ring_roll == pytest.approx(ROBOT_FIST_RIGHT[12])
    assert original_ring_roll > opose_ring_roll
