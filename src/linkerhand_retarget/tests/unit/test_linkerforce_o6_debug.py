import sys
import types
from types import SimpleNamespace

import pytest

from linkerhand_retarget.linkerhand.constants import RobotName


def test_o6_right_pinky_trace_logs_every_raw_poslist_frame(monkeypatch):
    def install_ros_stubs():
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

    install_ros_stubs()

    from linkerhand_retarget.motion.linkerforce.retarget import Retarget

    warnings = []

    class FakeLogger:
        def warn(self, msg):
            warnings.append(msg)

    retarget = Retarget.__new__(Retarget)
    retarget.node = SimpleNamespace(get_logger=lambda: FakeLogger())
    retarget.righthandtype = RobotName.o6
    retarget.righthand = SimpleNamespace(glove_version="v2")
    retarget.debug_enabled = True
    retarget.debug_raw_jump_threshold = 0.35
    retarget.debug_last_o6_pinky_raw_r = None

    first = [0.0] * 21
    first[18] = 2.0
    first[19] = 0.2
    first[20] = 0.3
    second = first.copy()
    second[18] = 2.8

    first_trace = retarget._capture_o6_right_pinky_raw_jump(
        first, left_valid=False, right_valid=True
    )
    trace = retarget._capture_o6_right_pinky_raw_jump(
        second, left_valid=False, right_valid=True
    )

    assert first_trace is not None
    assert first_trace["is_jump"] is False
    assert first_trace["raw_previous"] is None
    assert trace is not None
    assert trace["is_jump"] is True
    assert trace["raw_indices"] == [18, 19, 20]
    assert trace["raw_delta"] == pytest.approx([0.8, 0.0, 0.0])
    assert trace["max_index"] == 18
    assert len(warnings) == 2
    assert "[O6右手小指根部原始数据跟踪]" in warnings[0]
    assert "is_jump=False" in warnings[0]
    assert "[O6右手小指根部原始数据跟踪]" in warnings[1]
    assert "is_jump=True" in warnings[1]
    assert "raw_idx=[18, 19, 20]" in warnings[1]
    assert "raw_delta=[0.800000" in warnings[1]
    assert "max_idx=18" in warnings[1]
    assert "left_valid=False" in warnings[1]


def test_o6_right_pinky_publish_trace_logs_before_topic_publish(monkeypatch):
    def install_ros_stubs():
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

    install_ros_stubs()

    from linkerhand_retarget.motion.linkerforce.retarget import Retarget

    warnings = []

    class FakeLogger:
        def warn(self, msg):
            warnings.append(msg)

    retarget = Retarget.__new__(Retarget)
    retarget.node = SimpleNamespace(get_logger=lambda: FakeLogger())
    retarget.pubprintcount = 12
    msg = SimpleNamespace(
        name=[f"joint{i + 1}" for i in range(6)],
        position=[10.0, 20.0, 30.0, 40.0, 50.0, 60.0],
        velocity=[255.0, 255.0, 255.0, 255.0, 255.0, 128.0],
    )
    trace = {
        "raw_indices": [18, 19, 20],
        "raw_previous": [2.0, 0.2, 0.3],
        "raw_current": [2.8, 0.2, 0.3],
        "raw_delta": [0.8, 0.0, 0.0],
        "max_index": 18,
        "max_delta": 0.8,
        "is_jump": True,
        "threshold": 0.35,
        "left_valid": False,
        "right_valid": True,
        "glove_version": "v2",
    }

    retarget._trace_o6_right_publish_before_topic(trace, msg)

    assert len(warnings) == 1
    assert "[O6右手小指根部发布前跟踪]" in warnings[0]
    assert "publish_position=[10.000000, 20.000000, 30.000000, 40.000000, 50.000000, 60.000000]" in warnings[0]
    assert "pinky_motor=60.000000" in warnings[0]
    assert "pinky_velocity=128.000000" in warnings[0]
    assert "pub_count=12" in warnings[0]
