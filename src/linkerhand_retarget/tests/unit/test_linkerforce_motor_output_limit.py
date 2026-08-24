from pathlib import Path
import importlib
import sys
import types
from types import SimpleNamespace

import pytest
import yaml


def _install_ros_stubs():
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = type("Node", (), {})
    rclpy_executors = types.ModuleType("rclpy.executors")
    rclpy_executors.SingleThreadedExecutor = type("SingleThreadedExecutor", (), {})
    rclpy_callback_groups = types.ModuleType("rclpy.callback_groups")
    rclpy_callback_groups.MutuallyExclusiveCallbackGroup = type(
        "MutuallyExclusiveCallbackGroup",
        (),
        {},
    )
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

    sys.modules.setdefault("rclpy", rclpy)
    sys.modules.setdefault("rclpy.node", rclpy_node)
    sys.modules.setdefault("rclpy.executors", rclpy_executors)
    sys.modules.setdefault("rclpy.callback_groups", rclpy_callback_groups)
    sys.modules.setdefault("sensor_msgs", sensor_msgs)
    sys.modules.setdefault("sensor_msgs.msg", sensor_msgs_msg)
    sys.modules.setdefault("std_msgs", std_msgs)
    sys.modules.setdefault("std_msgs.msg", std_msgs_msg)


_install_ros_stubs()

from linkerhand_retarget.motion.linkerforce import retarget as retarget_module
from linkerhand_retarget.motion.linkerforce.retarget import Retarget
from linkerhand_retarget.motion.linkerforce.hand.linkerforce_o6 import (
    LeftHand as O6LeftHand,
    RightHand as O6RightHand,
)


MODEL_CONFIG_MODULES = (
    "g20_config",
    "l10_config",
    "l20_config",
    "l6_config",
    "o20_config",
    "o30_config",
    "o6_config",
    "o7_config",
)


def test_motor_output_limit_log_file_uses_linkerforce_log_directory():
    assert retarget_module.MOTOR_OUTPUT_LIMIT_LOG_FILE_PATH.parent.name == "log"
    assert retarget_module.MOTOR_OUTPUT_LIMIT_LOG_FILE_PATH.name == "motor_output_limit.log"


def test_motor_output_runtime_limit_caps_each_motor_step_over_absolute_limit():
    retarget = Retarget.__new__(Retarget)
    retarget.motor_output_frame_rate = 30.0
    retarget.motor_output_runtime_limits_l = [
        {"max_runtime_ms": 600.0},
        {"max_runtime_ms": 600.0},
        {"max_runtime_ms": 600.0},
    ]
    retarget.last_limited_motor_l = None

    assert retarget._apply_motor_output_runtime_limits("left", [100, 100, 100]) == [
        100,
        100,
        100,
    ]

    assert retarget._apply_motor_output_runtime_limits("left", [114, 115, 200]) == [
        114,
        100,
        100,
    ]

    assert retarget._apply_motor_output_runtime_limits("left", [99, 86, 0]) == [
        114,
        86,
        100,
    ]


def test_first_motor_output_runtime_frame_uses_realtime_value_as_baseline():
    retarget = Retarget.__new__(Retarget)
    retarget.motor_output_frame_rate = 30.0
    retarget.motor_output_runtime_limits_l = [{"max_runtime_ms": 600.0}]
    retarget.last_limited_motor_l = None

    # The first real-time value must be accepted, even though it is far from 255.
    assert retarget._apply_motor_output_runtime_limits("left", [0]) == [0]
    assert retarget._apply_motor_output_runtime_limits("left", [20]) == [0]


def test_single_frame_motor_spike_is_suppressed_until_the_next_frame_confirms_it():
    retarget = Retarget.__new__(Retarget)
    retarget.motor_output_frame_rate = 30.0
    retarget.motor_output_runtime_limits_l = [{"max_runtime_ms": 600.0}]
    retarget.last_limited_motor_l = None

    assert retarget._apply_motor_output_runtime_limits("left", [100]) == [100]
    assert retarget._apply_motor_output_runtime_limits("left", [200]) == [100]
    assert retarget._apply_motor_output_runtime_limits("left", [100]) == [100]


def test_confirmed_motor_motion_advances_after_one_frame_delay():
    retarget = Retarget.__new__(Retarget)
    retarget.motor_output_frame_rate = 30.0
    retarget.motor_output_runtime_limits_l = [{"max_runtime_ms": 600.0}]
    retarget.last_limited_motor_l = None

    assert retarget._apply_motor_output_runtime_limits("left", [100]) == [100]
    assert retarget._apply_motor_output_runtime_limits("left", [200]) == [100]
    assert retarget._apply_motor_output_runtime_limits("left", [220]) == [114]
    assert retarget._apply_motor_output_runtime_limits("left", [240]) == [128]


@pytest.mark.parametrize("hand_class", (O6LeftHand, O6RightHand))
def test_o6_first_mapper_smooth_frame_uses_realtime_motor_values(hand_class):
    hand = hand_class.__new__(hand_class)
    hand.smooth_enabled = True
    hand.smooth_alpha = 0.5
    hand.smooth_positions = [255.0, 255.0]
    hand.max_step = 20

    assert hand._apply_smooth([40, 80]) == [40, 80]


def test_motor_output_runtime_limit_keeps_hand_history_separate():
    retarget = Retarget.__new__(Retarget)
    retarget.motor_output_frame_rate = 30.0
    retarget.motor_output_runtime_limits_l = [{"max_runtime_ms": 1000.0}]
    retarget.motor_output_runtime_limits_r = [{"max_runtime_ms": 1000.0}]
    retarget.last_limited_motor_l = None
    retarget.last_limited_motor_r = None

    assert retarget._apply_motor_output_runtime_limits("left", [10]) == [10]
    assert retarget._apply_motor_output_runtime_limits("right", [100]) == [100]
    assert retarget._apply_motor_output_runtime_limits("left", [20]) == [10]
    assert retarget._apply_motor_output_runtime_limits("right", [105]) == [105]


def test_motor_output_runtime_limit_records_cutoff_to_single_file(monkeypatch, tmp_path):
    log_file = tmp_path / "motor_output_limit.log"
    monkeypatch.setattr(
        retarget_module,
        "MOTOR_OUTPUT_LIMIT_LOG_FILE_PATH",
        log_file,
    )

    retarget = Retarget.__new__(Retarget)
    retarget.motor_output_limit_debug = True
    retarget.motor_output_frame_rate = 30.0
    retarget.motor_output_runtime_limits_l = [{"max_runtime_ms": 600.0}]
    retarget.last_limited_motor_l = None

    assert retarget._apply_motor_output_runtime_limits("left", [100]) == [100]
    assert retarget._apply_motor_output_runtime_limits("left", [115]) == [100]

    log_text = log_file.read_text(encoding="utf-8")
    assert "timestamp=" in log_text
    assert "event=motor_output_limit" in log_text
    assert "hand=left" in log_text
    assert "motor=0" in log_text
    assert "previous=100.000" in log_text
    assert "current=115.000" in log_text
    assert "max_delta=14.167" in log_text


def test_motor_output_frame_rate_is_read_from_base_config():
    retarget = Retarget.__new__(Retarget)
    retarget.baseconfig = {"motor_output": {"frame_rate": 60}}

    assert retarget._get_motor_output_frame_rate() == 60.0


def test_process_timer_uses_motor_output_frame_rate_for_topic_publish():
    created_timer = {}

    class FakeNode:
        def create_timer(self, interval, callback, callback_group=None):
            created_timer["interval"] = interval
            created_timer["callback"] = callback
            created_timer["callback_group"] = callback_group

        def get_logger(self):
            return SimpleNamespace(error=lambda _message: None)

    retarget = Retarget.__new__(Retarget)
    retarget.node = FakeNode()
    retarget.calibration = False
    retarget.motor_output_frame_rate = 60.0
    retarget.retarget_callback_group = "group"
    retarget.linkerforce_init = lambda: None
    retarget._start_touch_subscription_worker = lambda: None
    retarget._start_force_feedback_worker = lambda: None
    retarget.process_callback = lambda: None

    assert retarget.process() is True
    assert created_timer["interval"] == pytest.approx(1.0 / 60.0)
    assert created_timer["callback"] == retarget.process_callback
    assert created_timer["callback_group"] == "group"


def test_base_config_defines_motor_output_frame_rate():
    config_path = (
        Path(__file__).parents[2]
        / "linkerhand_retarget"
        / "config"
        / "base_config.yml"
    )

    base_config = yaml.safe_load(config_path.read_text(encoding="utf-8"))

    assert base_config["motor_output"]["frame_rate"] == 30


def test_base_config_defines_output_runtime_limits_without_switch():
    config_path = (
        Path(__file__).parents[2]
        / "linkerhand_retarget"
        / "config"
        / "base_config.yml"
    )

    base_config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    runtime_limits = base_config["motor_output"]["OUTPUT_RUNTIME_LIMITS"]

    assert runtime_limits == {"max_runtime_ms": 300.0}


def test_base_config_defines_motor_output_limit_debug_switch():
    config_path = (
        Path(__file__).parents[2]
        / "linkerhand_retarget"
        / "config"
        / "base_config.yml"
    )

    base_config = yaml.safe_load(config_path.read_text(encoding="utf-8"))

    assert isinstance(base_config["debug"]["motor_output_limit_debug"], bool)


@pytest.mark.parametrize("module_name", MODEL_CONFIG_MODULES)
def test_model_config_does_not_define_output_runtime_limits(module_name):
    module = importlib.import_module(
        f"linkerhand_retarget.motion.linkerforce.config.{module_name}"
    )
    assert not hasattr(module, "OUTPUT_RUNTIME_LIMITS")
    assert not hasattr(module, "MOTOR_OUTPUT_RUNTIME_LIMITS")


def test_base_config_output_runtime_limits_expand_to_motor_count():
    retarget = Retarget.__new__(Retarget)
    retarget.baseconfig = {
        "motor_output": {
            "OUTPUT_RUNTIME_LIMITS": {
                "max_runtime_ms": 300.0,
            }
        }
    }

    runtime_limits = retarget._load_motor_output_runtime_limits(
        retarget_module.RobotName.o6
    )

    assert len(runtime_limits) == 6
    assert runtime_limits == [
        {"max_runtime_ms": 300.0}
        for _ in range(6)
    ]
    assert runtime_limits[0] is not runtime_limits[1]


def test_missing_output_runtime_limits_default_to_300_ms():
    retarget = Retarget.__new__(Retarget)
    retarget.baseconfig = {"motor_output": {}}

    runtime_limits = retarget._load_motor_output_runtime_limits(
        retarget_module.RobotName.o6
    )

    assert runtime_limits == [
        {"max_runtime_ms": 300.0}
        for _ in range(6)
    ]
