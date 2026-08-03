import sys
import types
from types import SimpleNamespace

import pytest

from linkerhand_retarget.linkerhand.constants import RobotName


def test_o6_right_pinky_trace_captures_raw_poslist_without_verbose_warning(monkeypatch):
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
    assert warnings == []


def test_o6_right_publish_trace_does_not_log_frame_rate(monkeypatch):
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

    infos = []

    class FakeLogger:
        def warn(self, msg):
            raise AssertionError(f"unexpected warning: {msg}")

        def info(self, msg):
            infos.append(msg)

    retarget = Retarget.__new__(Retarget)
    retarget.node = SimpleNamespace(get_logger=lambda: FakeLogger())
    retarget.pubprintcount = 12
    retarget.debug_o6_publish_rate_start_time = None
    retarget.debug_o6_publish_rate_last_time = None
    retarget.debug_o6_publish_rate_total_count = 0
    retarget.debug_o6_publish_rate_window_count = 0
    retarget.debug_o6_publish_rate_interval = 1.0
    retarget.debug_last_force04_r = [10.0, 20.0, 30.0, 40.0, 50.0]
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

    monotonic_values = iter([100.0, 100.4, 101.2])
    monkeypatch.setattr("linkerhand_retarget.motion.linkerforce.retarget.time.monotonic", lambda: next(monotonic_values))

    retarget._trace_o6_right_publish_frame_rate(trace)
    retarget._trace_o6_right_publish_frame_rate(trace)
    retarget._trace_o6_right_publish_frame_rate(trace)

    assert infos == []


def test_endpoint_motor_jump_logs_previous_and_current_raw_data(monkeypatch):
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
    retarget.debug_enabled = True
    retarget.debug_endpoint_jump_tolerance = 0.5

    previous_raw = [1.0, 2.0, 3.0]
    current_raw = [4.0, 5.0, 6.0]
    next_raw = [7.0, 8.0, 9.0]

    retarget._trace_endpoint_motor_jump(
        "right",
        previous_raw,
        [255.0, 10.0],
    )
    retarget._trace_endpoint_motor_jump(
        "right",
        current_raw,
        [0.0, 10.0],
    )
    retarget._trace_endpoint_motor_jump(
        "right",
        next_raw,
        [255.0, 10.0],
    )

    assert len(warnings) == 2
    assert "方向=255->0" in warnings[0]
    assert "方向=0->255" in warnings[1]
    assert "prev=255.000000" in warnings[0]
    assert "current=0.000000" in warnings[0]
    assert "motor_previous=" not in warnings[0]
    assert "motor_current=" not in warnings[0]
    assert "raw_previous=[1.0, 2.0, 3.0]" in warnings[0]
    assert "raw_current=[4.0, 5.0, 6.0]" in warnings[0]
    assert "raw_previous=[4.0, 5.0, 6.0]" in warnings[1]
    assert "raw_current=[7.0, 8.0, 9.0]" in warnings[1]
