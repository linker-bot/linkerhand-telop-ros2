import json
import inspect
import sys
import types
from types import SimpleNamespace

import pytest


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


def test_m11_protocol_exposes_own_type_names():
    from linkerhand_retarget.motion.linkermcg_m11 import protocol as m11_protocol

    assert m11_protocol.M11MotionData.__name__ == "M11MotionData"
    assert m11_protocol.LinkerMcgM11UdpClient.__name__ == "LinkerMcgM11UdpClient"
    assert m11_protocol.LinkerMcgM11UdpClient.motion_label == "M11"
    assert m11_protocol.LinkerMcgM11UdpClient.thread_name == "linkermcg_m11_udp"
    assert m11_protocol.LinkerMcgM11UdpClient.motion_data_class.__name__ == "M11MotionData"


def test_m11_protocol_accepts_documented_o20_16_dof_degrees():
    from linkerhand_retarget.motion.linkermcg_m11 import protocol as m11_protocol

    payload = {
        "schemaId": "linker.o20.targetpos16.flat.v1",
        "handType": "LinkerHand/O20",
        "dof": 16,
        "timestampMs": 1710000000789,
        "labels": [
            "thumb_mcp", "thumb_ip", "thumb_abd", "thumb_cmc",
            "index_abd", "index_mcp", "index_pip",
            "middle_abd", "middle_mcp", "middle_pip",
            "ring_abd", "ring_mcp", "ring_pip",
            "pinky_abd", "pinky_mcp", "pinky_dip",
        ],
        "leftHand": [0, 10, -30, 40, -20, 60, 70, 0, 90, 100, -15, 120, 130, -10, 150, 160],
        "rightHand": [1, 11, -29, 41, -19, 61, 71, 1, 91, 101, -14, 121, 131, -9, 151, 161],
    }

    envelope = m11_protocol.parse_stroke_envelope(json.dumps(payload).encode("utf-8"))

    assert envelope.schema_id == "linker.o20.targetpos16.flat.v1"
    assert envelope.hand_type == "LinkerHand/O20"
    assert envelope.dof == 16
    assert envelope.left_hand[2] == -30.0
    assert envelope.right_hand[13] == -9.0


def test_m11_protocol_source_does_not_reference_m7():
    from linkerhand_retarget.motion.linkermcg_m11 import protocol as m11_protocol

    source = inspect.getsource(m11_protocol)
    assert "linkermcg_m7" not in source


def test_m11_retarget_exports_own_retargter_class(monkeypatch):
    install_ros_stubs(monkeypatch)

    from linkerhand_retarget.motion.linkermcg_m11 import retarget as m11_retarget_module

    assert m11_retarget_module.Retarget.__name__ == "Retarget"
    assert m11_retarget_module.Retarget.motion_label == "M11"
    assert m11_retarget_module.Retarget.udp_client_class.__name__ == "LinkerMcgM11UdpClient"
    source = inspect.getsource(m11_retarget_module)
    assert "linkermcg_m7" not in source


def test_m11_direct_hand_mapping_and_expected_dof():
    from linkerhand_retarget.linkerhand.constants import RobotName
    from linkerhand_retarget.motion.linkermcg_m11.hand.direct_hand import (
        DirectHand,
        expected_dof_for_robot,
    )

    assert expected_dof_for_robot(RobotName.o20) == 16
    assert expected_dof_for_robot(RobotName.o30) == 20

    hand = DirectHand(length=16)
    hand.joint_update([0, 10, -30, 40, -20, 60, 70, 0, 90, 100, -15, 120, 130, -10, 150, 160, 999])
    hand.speed_update()

    assert hand.g_jointpositions == [0.0, 10.0, -30.0, 40.0, -20.0, 60.0, 70.0, 0.0, 90.0, 100.0, -15.0, 120.0, 130.0, -10.0, 150.0, 160.0]
    assert hand.g_jointvelocity == [255] * 16


def test_handretarget_routes_linkermcg_m11_motion_source(monkeypatch):
    install_ros_stubs(monkeypatch)

    from linkerhand_retarget import handretarget as handretarget_module
    from linkerhand_retarget.motion.linkermcg_m11 import retarget as m11_retarget_module

    calls = {}

    class FakeRetarget:
        def __init__(self, *args, **kwargs):
            calls["args"] = args
            calls["kwargs"] = kwargs

        def process(self):
            calls["process"] = True
            return True

    monkeypatch.setattr(m11_retarget_module, "Retarget", FakeRetarget)

    node = handretarget_module.HandRetargetNode.__new__(
        handretarget_module.HandRetargetNode
    )
    node.motion_type = handretarget_module.MotionSource.linkermcg_m11
    node.udp_ip = "192.168.1.50"
    node.udp_port = 9011
    node.robot_name_r = handretarget_module.RobotName.o20
    node.robot_name_l = handretarget_module.RobotName.o20
    node.handcore = types.SimpleNamespace()
    node.lefthandprint = True
    node.righthandprint = False
    node.retarget = None
    node.get_logger = lambda: type("Logger", (), {"info": lambda self, message: None})()
    node._start_mujoco_display_if_enabled = lambda: calls.__setitem__("mujoco", True)

    assert node.retargetrun() is True
    assert calls["kwargs"]["ip"] == "192.168.1.50"
    assert calls["kwargs"]["port"] == 9011
    assert calls["kwargs"]["lefthand"] == handretarget_module.RobotName.o20
    assert calls["kwargs"]["righthand"] == handretarget_module.RobotName.o20
    assert calls["process"] is True
    assert calls["mujoco"] is True


def test_m11_retarget_publishes_documented_o20_16_dof_frame(monkeypatch):
    install_ros_stubs(monkeypatch)

    from linkerhand_retarget.linkerhand.constants import RobotName
    from linkerhand_retarget.motion.linkermcg_m11.protocol import (
        M11MotionData,
        parse_stroke_envelope,
    )
    from linkerhand_retarget.motion.linkermcg_m11.retarget import Retarget

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
            return type("Logger", (), {"info": lambda self, message: None, "warn": lambda self, message: None})()

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
    envelope = parse_stroke_envelope(
        json.dumps(
            {
                "schemaId": "linker.o20.targetpos16.flat.v1",
                "handType": "LinkerHand/O20",
                "dof": 16,
                "timestampMs": 1710000000789,
                "labels": [
                    "thumb_mcp", "thumb_ip", "thumb_abd", "thumb_cmc",
                    "index_abd", "index_mcp", "index_pip",
                    "middle_abd", "middle_mcp", "middle_pip",
                    "ring_abd", "ring_mcp", "ring_pip",
                    "pinky_abd", "pinky_mcp", "pinky_dip",
                ],
                "leftHand": [0, 10, -30, 40, -20, 60, 70, 0, 90, 100, -15, 120, 130, -10, 150, 160],
                "rightHand": [1, 11, -29, 41, -19, 61, 71, 1, 91, 101, -14, 121, 131, -9, 151, 161],
            }
        ).encode("utf-8")
    )
    mocap_data = M11MotionData()
    mocap_data.update_from_envelope(envelope, frame_index=1)
    retarget.udp_datacapture = SimpleNamespace(realmocapdata=mocap_data)

    retarget.process_callback()

    assert retarget.expected_dof_l == 16
    assert retarget.expected_dof_r == 16
    assert published["left"][0].position == [0.0, 10.0, -30.0, 40.0, -20.0, 60.0, 70.0, 0.0, 90.0, 100.0, -15.0, 120.0, 130.0, -10.0, 150.0, 160.0]
    assert published["right"][0].position == [1.0, 11.0, -29.0, 41.0, -19.0, 61.0, 71.0, 1.0, 91.0, 101.0, -14.0, 121.0, 131.0, -9.0, 151.0, 161.0]


def test_m11_retarget_accepts_o30_schema(monkeypatch):
    install_ros_stubs(monkeypatch)

    from linkerhand_retarget.linkerhand.constants import RobotName
    from linkerhand_retarget.motion.linkermcg_m11.protocol import (
        M11MotionData,
        parse_stroke_envelope,
    )
    from linkerhand_retarget.motion.linkermcg_m11.retarget import Retarget

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
            return type(
                "Logger",
                (),
                {"info": lambda self, message: None, "warn": lambda self, message: None},
            )()

    retarget = Retarget(
        FakeNode(),
        ip="127.0.0.1",
        port=9011,
        lefthand=RobotName.o30,
        righthand=RobotName.o30,
        handcore=SimpleNamespace(hand_numjoints_l=20, hand_numjoints_r=20),
        lefthandpubprint=False,
        righthandpubprint=False,
    )
    envelope = parse_stroke_envelope(
        json.dumps(
            {
                "schemaId": "linker.o30.stroke20.flat.v1",
                "handType": "LinkerHand/O30",
                "dof": 20,
                "timestampMs": 1710000000900,
                "labels": [
                    "thumb_roll",
                    "thumb_yaw",
                    "index_yaw",
                    "middle_yaw",
                    "ring_yaw",
                    "little_yaw",
                    "thumb_root1",
                    "index_root1",
                    "middle_root1",
                    "ring_root1",
                    "little_root1",
                    "index_root2",
                    "middle_root2",
                    "ring_root2",
                    "little_root2",
                    "thumb_tip",
                    "index_tip",
                    "middle_tip",
                    "ring_tip",
                    "little_tip",
                ],
                "leftHand": list(range(20)),
                "rightHand": list(range(20, 40)),
            }
        ).encode("utf-8")
    )
    mocap_data = M11MotionData()
    mocap_data.update_from_envelope(envelope, frame_index=1)
    retarget.udp_datacapture = SimpleNamespace(realmocapdata=mocap_data)

    retarget.process_callback()

    assert published["left"][0].position == [float(value) for value in range(20)]
    assert published["right"][0].position == [float(value) for value in range(20, 40)]


def test_m11_retarget_rejects_o30_configured_hand_when_schema_is_l20(monkeypatch):
    install_ros_stubs(monkeypatch)

    from linkerhand_retarget.linkerhand.constants import RobotName
    from linkerhand_retarget.motion.linkermcg_m11.protocol import (
        M11MotionData,
        parse_stroke_envelope,
    )
    from linkerhand_retarget.motion.linkermcg_m11.retarget import Retarget

    published = {"left": [], "right": []}
    warnings = []

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
            return type("Logger", (), {"info": lambda self, message: None, "warn": lambda self, message: warnings.append(message)})()

    retarget = Retarget(
        FakeNode(),
        ip="127.0.0.1",
        port=9011,
        lefthand=RobotName.o30,
        righthand=RobotName.o30,
        handcore=SimpleNamespace(hand_numjoints_l=20, hand_numjoints_r=20),
        lefthandpubprint=False,
        righthandpubprint=False,
    )
    envelope = parse_stroke_envelope(
        json.dumps(
            {
                "schemaId": "linker.stroke20.flat.v1",
                "handType": "LinkerHand/L20",
                "dof": 20,
                "timestampMs": 1710000000789,
                "labels": [f"joint{i + 1}" for i in range(20)],
                "leftHand": list(range(20)),
                "rightHand": list(range(20, 40)),
            }
        ).encode("utf-8")
    )
    mocap_data = M11MotionData()
    mocap_data.update_from_envelope(envelope, frame_index=1)
    retarget.udp_datacapture = SimpleNamespace(realmocapdata=mocap_data)

    retarget.process_callback()

    assert published == {"left": [], "right": []}
    assert any("schemaId=linker.stroke20.flat.v1" in message for message in warnings)
