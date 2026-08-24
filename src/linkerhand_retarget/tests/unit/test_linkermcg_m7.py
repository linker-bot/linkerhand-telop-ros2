import json
import sys
import types
from types import SimpleNamespace

import pytest

from linkerhand_retarget.linkerhand.constants import MotionSource, RobotName
from linkerhand_retarget.motion.linkermcg_m7.protocol import (
    LinkerMcgM7UdpClient,
    M7MotionData,
    parse_stroke_envelope,
)
from linkerhand_retarget.motion.linkermcg_m7.hand.direct_hand import (
    DirectHand,
    expected_dof_for_robot,
)


class FakeLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []
        self.errors = []

    def info(self, message):
        self.infos.append(message)

    def warn(self, message):
        self.warnings.append(message)

    def error(self, message):
        self.errors.append(message)


def install_ros_stubs(monkeypatch):
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda args=None: None
    rclpy.shutdown = lambda: None
    rclpy.spin = lambda node, executor: None
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = type("Node", (), {})
    rclpy_executors = types.ModuleType("rclpy.executors")
    rclpy_executors.MultiThreadedExecutor = type("MultiThreadedExecutor", (), {})
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")

    class JointState:
        def __init__(self):
            self.header = SimpleNamespace(stamp=None)
            self.name = []
            self.position = []
            self.velocity = []

    sensor_msgs_msg.JointState = JointState
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.PoseArray = type("PoseArray", (), {})
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = type("String", (), {})
    rcl_interfaces = types.ModuleType("rcl_interfaces")
    rcl_interfaces_msg = types.ModuleType("rcl_interfaces.msg")
    rcl_interfaces_msg.ParameterDescriptor = type("ParameterDescriptor", (), {})
    ament_index_python = types.ModuleType("ament_index_python")
    ament_index_python_packages = types.ModuleType("ament_index_python.packages")
    ament_index_python_packages.get_package_share_directory = lambda _name: "/tmp"

    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy_node)
    monkeypatch.setitem(sys.modules, "rclpy.executors", rclpy_executors)
    monkeypatch.setitem(sys.modules, "sensor_msgs", sensor_msgs)
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", sensor_msgs_msg)
    monkeypatch.setitem(sys.modules, "geometry_msgs", geometry_msgs)
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", geometry_msgs_msg)
    monkeypatch.setitem(sys.modules, "std_msgs", std_msgs)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", std_msgs_msg)
    monkeypatch.setitem(sys.modules, "rcl_interfaces", rcl_interfaces)
    monkeypatch.setitem(sys.modules, "rcl_interfaces.msg", rcl_interfaces_msg)
    monkeypatch.setitem(sys.modules, "ament_index_python", ament_index_python)
    monkeypatch.setitem(sys.modules, "ament_index_python.packages", ament_index_python_packages)


def test_motion_source_registers_linkermcg_m7():
    assert MotionSource["linkermcg_m7"] == MotionSource.linkermcg_m7


def test_parse_m7_flat_envelope_keeps_documented_order():
    payload = {
        "schemaId": "linker.stroke20.flat.v1",
        "handType": "LinkerHand/O20",
        "dof": 20,
        "timestampMs": 1710000000789,
        "labels": [
            "thumb_pitch", "index_pitch", "middle_pitch", "ring_pitch", "pinky_pitch",
            "thumb_side", "index_side", "middle_side", "ring_side", "pinky_side",
            "thumb_roll", "index_roll", "middle_roll", "ring_roll", "pinky_roll",
            "thumb_end_pitch", "index_end_pitch", "middle_end_pitch", "ring_end_pitch", "pinky_end_pitch",
        ],
        "leftHand": list(range(20)),
        "rightHand": list(range(20, 40)),
    }

    envelope = parse_stroke_envelope(json.dumps(payload).encode("utf-8"))

    assert envelope.schema_id == "linker.stroke20.flat.v1"
    assert envelope.hand_type == "LinkerHand/O20"
    assert envelope.dof == 20
    assert envelope.left_hand == [float(value) for value in range(20)]
    assert envelope.right_hand == [float(value) for value in range(20, 40)]


def test_parse_m7_minimal_left_right_payload_infers_o6_schema():
    payload = {
        "leftHand": [0, 1, 2, 3, 4, 5],
        "rightHand": [10, 11, 12, 13, 14, 15],
    }

    envelope = parse_stroke_envelope(json.dumps(payload).encode("utf-8"))

    assert envelope.schema_id == "linker.stroke6.flat.v1"
    assert envelope.hand_type == "LinkerHand/O6"
    assert envelope.dof == 6
    assert envelope.labels == ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    assert envelope.left_hand == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    assert envelope.right_hand == [10.0, 11.0, 12.0, 13.0, 14.0, 15.0]


def test_parse_m7_flat_envelope_rejects_dof_length_mismatch():
    payload = {
        "schemaId": "linker.stroke6.flat.v1",
        "handType": "LinkerHand/O6",
        "dof": 6,
        "timestampMs": 1710000000123,
        "labels": ["thumb_pitch"] * 6,
        "leftHand": [1, 2, 3, 4, 5],
        "rightHand": [1, 2, 3, 4, 5, 6],
    }

    with pytest.raises(ValueError, match="leftHand length"):
        parse_stroke_envelope(json.dumps(payload).encode("utf-8"))


def test_m7_motion_data_updates_both_hands_from_envelope():
    envelope = parse_stroke_envelope(
        json.dumps(
            {
                "schemaId": "linker.stroke6.flat.v1",
                "handType": "LinkerHand/O6",
                "dof": 6,
                "timestampMs": 1710000000123,
                "labels": [
                    "thumb_pitch",
                    "thumb_side",
                    "index_pitch",
                    "middle_pitch",
                    "ring_pitch",
                    "pinky_pitch",
                ],
                "leftHand": [10, 11, 12, 13, 14, 15],
                "rightHand": [20, 21, 22, 23, 24, 25],
            }
        ).encode("utf-8")
    )
    data = M7MotionData()

    data.update_from_envelope(envelope, frame_index=8)

    assert data.is_update is True
    assert data.frame_index == 8
    assert data.dof == 6
    assert data.jointangle_lHand == [10.0, 11.0, 12.0, 13.0, 14.0, 15.0]
    assert data.jointangle_rHand == [20.0, 21.0, 22.0, 23.0, 24.0, 25.0]


def test_m7_udp_client_warns_when_no_status_or_json_arrives_after_connect_timeout():
    logger = FakeLogger()
    client = LinkerMcgM7UdpClient(
        host="127.0.0.1",
        port=9011,
        logger=logger,
        first_json_timeout_sec=2.0,
        no_json_warn_interval_sec=5.0,
    )
    client.udp_running = True
    client._started_at = 100.0

    client._warn_if_no_json_frame(now=101.9)
    assert logger.warnings == []

    client._warn_if_no_json_frame(now=102.0)
    assert len(logger.warnings) == 1
    assert "no heartbeat/status packet or M7 stroke JSON received" in logger.warnings[0]

    client._warn_if_no_json_frame(now=103.0)
    assert len(logger.warnings) == 1

    client._warn_if_no_json_frame(now=107.1)
    assert len(logger.warnings) == 2


def test_m7_udp_client_warns_when_only_heartbeat_arrives_without_json():
    logger = FakeLogger()
    client = LinkerMcgM7UdpClient(
        host="127.0.0.1",
        port=9011,
        logger=logger,
        first_json_timeout_sec=2.0,
        no_json_warn_interval_sec=5.0,
    )
    client.udp_running = True
    client._started_at = 100.0
    client._last_status_packet_at = 101.5

    client._warn_if_no_json_frame(now=102.0)

    assert len(logger.warnings) == 1
    assert "heartbeat/status packets are arriving" in logger.warnings[0]
    assert "no M7 stroke JSON received" in logger.warnings[0]


@pytest.mark.parametrize(
    ("robot_name", "expected"),
    [
        (RobotName.o6, 6),
        (RobotName.l6, 6),
        (RobotName.l10, 10),
        (RobotName.l10v7, 10),
        (RobotName.l20lite, 10),
        (RobotName.l20, 20),
        (RobotName.o20, 20),
        (RobotName.o30, 20),
        (RobotName.g20, 20),
        (RobotName.l25, 20),
    ],
)
def test_expected_dof_for_supported_m7_robot_models(robot_name, expected):
    assert expected_dof_for_robot(robot_name) == expected


def test_expected_dof_accepts_same_named_robot_enum_from_runtime_import():
    from linkerhand.constants import RobotName as RuntimeRobotName

    assert RuntimeRobotName is not RobotName
    assert expected_dof_for_robot(RuntimeRobotName.o20) == 20
    assert expected_dof_for_robot(RuntimeRobotName.o30) == 20


def test_expected_dof_rejects_retired_o30_variant_name():
    retired_name = "o30" + "i"

    assert retired_name not in RobotName.__members__


def test_direct_hand_mapping_copies_m7_command_order_without_old_reorder():
    hand = DirectHand(length=6)

    hand.joint_update([0, 1, 2, 3, 4, 5, 99])
    hand.speed_update()

    assert hand.g_jointpositions == [0, 1, 2, 3, 4, 5]
    assert hand.g_jointvelocity == [255, 255, 255, 255, 255, 255]


def test_handretarget_routes_linkermcg_m7_motion_source(monkeypatch):
    install_ros_stubs(monkeypatch)

    from linkerhand_retarget import handretarget as handretarget_module
    from linkerhand_retarget.motion.linkermcg_m7 import retarget as m7_retarget_module

    calls = {}

    class FakeRetarget:
        def __init__(self, *args, **kwargs):
            calls["args"] = args
            calls["kwargs"] = kwargs

        def process(self):
            calls["process"] = True
            return True

    monkeypatch.setattr(m7_retarget_module, "Retarget", FakeRetarget)

    node = handretarget_module.HandRetargetNode.__new__(
        handretarget_module.HandRetargetNode
    )
    node.motion_type = handretarget_module.MotionSource.linkermcg_m7
    node.udp_ip = "192.168.1.50"
    node.udp_port = 9011
    node.robot_name_r = handretarget_module.RobotName.o20
    node.robot_name_l = handretarget_module.RobotName.o20
    node.handcore = SimpleNamespace()
    node.lefthandprint = True
    node.righthandprint = False
    node.retarget = None
    node.get_logger = lambda: FakeLogger()
    node._start_mujoco_display_if_enabled = lambda: calls.__setitem__("mujoco", True)

    assert node.retargetrun() is True
    assert calls["kwargs"]["ip"] == "192.168.1.50"
    assert calls["kwargs"]["port"] == 9011
    assert calls["kwargs"]["lefthand"] == handretarget_module.RobotName.o20
    assert calls["kwargs"]["righthand"] == handretarget_module.RobotName.o20
    assert calls["process"] is True
    assert calls["mujoco"] is True


def test_m7_retarget_accepts_package_robot_enum(monkeypatch):
    install_ros_stubs(monkeypatch)

    from linkerhand_retarget.motion.linkermcg_m7.retarget import Retarget

    class FakeNode:
        def create_publisher(self, *_args, **_kwargs):
            return SimpleNamespace(publish=lambda _msg: None)

        def create_timer(self, *_args, **_kwargs):
            return object()

        def get_logger(self):
            return FakeLogger()

    retarget = Retarget(
        FakeNode(),
        ip="127.0.0.1",
        port=9011,
        lefthand=RobotName.o20,
        righthand=RobotName.o20,
        handcore=SimpleNamespace(hand_numjoints_l=20, hand_numjoints_r=20),
        lefthandpubprint=False,
        righthandpubprint=False,
    )

    assert retarget.expected_dof_l == 20
    assert retarget.expected_dof_r == 20
    assert len(retarget.lefthand.g_jointpositions) == 20
    assert len(retarget.righthand.g_jointpositions) == 20


def test_m7_retarget_process_callback_publishes_frame_once(monkeypatch):
    install_ros_stubs(monkeypatch)

    from linkerhand_retarget.motion.linkermcg_m7.retarget import Retarget

    published = {"left": [], "right": []}

    class FakePublisher:
        def __init__(self, side):
            self.side = side

        def publish(self, msg):
            published[self.side].append(msg)

    class FakeStamp:
        nanoseconds = 1_000_000_000

        def to_msg(self):
            return "stamp"

    class FakeNode:
        def create_publisher(self, _msg_type, topic, _qos):
            side = "left" if "left" in topic else "right"
            return FakePublisher(side)

        def create_timer(self, *_args, **_kwargs):
            return object()

        def get_clock(self):
            return SimpleNamespace(now=lambda: FakeStamp())

        def get_logger(self):
            return FakeLogger()

    retarget = Retarget(
        FakeNode(),
        ip="127.0.0.1",
        port=9011,
        lefthand=RobotName.o6,
        righthand=RobotName.o6,
        handcore=SimpleNamespace(hand_numjoints_l=6, hand_numjoints_r=6),
        lefthandpubprint=False,
        righthandpubprint=False,
    )
    envelope = parse_stroke_envelope(
        json.dumps(
            {
                "schemaId": "linker.stroke6.flat.v1",
                "handType": "LinkerHand/O6",
                "dof": 6,
                "timestampMs": 1710000000123,
                "labels": [
                    "thumb_pitch",
                    "thumb_side",
                    "index_pitch",
                    "middle_pitch",
                    "ring_pitch",
                    "pinky_pitch",
                ],
                "leftHand": [0, 1, 2, 3, 4, 5],
                "rightHand": [250, 251, 252, 253, 254, 255],
            }
        ).encode("utf-8")
    )
    mocap_data = M7MotionData()
    mocap_data.update_from_envelope(envelope, frame_index=1)
    retarget.udp_datacapture = SimpleNamespace(realmocapdata=mocap_data)

    retarget.process_callback()
    retarget.process_callback()

    assert len(published["left"]) == 1
    assert len(published["right"]) == 1
    assert published["left"][0].position == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    assert published["right"][0].position == [250.0, 251.0, 252.0, 253.0, 254.0, 255.0]
