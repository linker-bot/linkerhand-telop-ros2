import json
import importlib
import sys
import threading
import time
import types
from types import SimpleNamespace


def install_ros_stubs(monkeypatch):
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


class FakeLogger:
    def __init__(self):
        self.errors = []
        self.warnings = []

    def error(self, message):
        self.errors.append(message)

    def warn(self, message):
        self.warnings.append(message)


class TrackingLock:
    def __init__(self):
        self._lock = threading.Lock()
        self.held = False

    def __enter__(self):
        self._lock.acquire()
        self.held = True
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.held = False
        self._lock.release()
        return False


class GuardedSerialPort:
    def __init__(self, lock):
        self.lock = lock
        self.writes = []

    def write(self, data):
        assert not self.lock.held
        self.writes.append(data)


def make_touch_payload(value):
    matrix = [[value, 0], [0, value / 2]]
    return json.dumps(
        {
            "thumb_matrix": matrix,
            "index_matrix": matrix,
            "middle_matrix": matrix,
            "ring_matrix": matrix,
            "little_matrix": matrix,
        }
    )


def test_touch_callback_caches_force_values_without_serial_write(monkeypatch):
    install_ros_stubs(monkeypatch)

    from linkerhand_retarget.motion.linkerforce.retarget import Retarget

    lock = TrackingLock()
    serial_port = GuardedSerialPort(lock)
    retarget = Retarget.__new__(Retarget)
    retarget.node = SimpleNamespace(get_logger=lambda: FakeLogger())
    retarget.forcelock = lock
    retarget.results = {"left": {}, "right": {}}
    retarget.force_reader_right = SimpleNamespace(
        serial_port=serial_port,
        forcelist=[0.0] * 5,
        pack_04_data=lambda: b"force",
    )

    retarget.process_touch_data(make_touch_payload(2.0), "right")

    assert retarget.results["right"]["forcelist"] == [8.0, 8.0, 8.0, 8.0, 8.0]
    assert serial_port.writes == []


def test_touch_callback_clamps_negative_force_values_to_zero(monkeypatch):
    install_ros_stubs(monkeypatch)

    from linkerhand_retarget.motion.linkerforce.retarget import Retarget

    retarget = Retarget.__new__(Retarget)
    retarget.node = SimpleNamespace(get_logger=lambda: FakeLogger())
    retarget.forcelock = TrackingLock()
    retarget.results = {"left": {}, "right": {}}

    matrix = [[-1, -1], [-1, -1]]
    retarget.process_touch_data(
        json.dumps(
            {
                "thumb_matrix": matrix,
                "index_matrix": matrix,
                "middle_matrix": matrix,
                "ring_matrix": matrix,
                "little_matrix": matrix,
            }
        ),
        "right",
    )

    assert retarget.results["right"]["forcelist"] == [0.0, 0.0, 0.0, 0.0, 0.0]


def test_force_feedback_worker_writes_latest_force_outside_callback_lock(monkeypatch):
    install_ros_stubs(monkeypatch)

    from linkerhand_retarget.motion.linkerforce.retarget import Retarget

    lock = TrackingLock()
    serial_port = GuardedSerialPort(lock)
    reader = SimpleNamespace(
        serial_port=serial_port,
        forcelist=[0.0] * 5,
        pack_04_data=lambda: b"force",
    )
    retarget = Retarget.__new__(Retarget)
    retarget.node = SimpleNamespace(get_logger=lambda: FakeLogger())
    retarget.forcelock = lock
    retarget.results = {"left": {}, "right": {"forcelist": [1, 2, 3, 4, 5]}}
    retarget.force_feedback_thread_interval = 0.001
    retarget.force_feedback_running = threading.Event()
    retarget.force_feedback_running.set()
    retarget.force_reader_right = reader
    retarget.force_reader_left = None
    retarget.debug_last_force04_r = None

    sleeps = {"count": 0}

    def fake_sleep(_seconds):
        sleeps["count"] += 1
        retarget.force_feedback_running.clear()

    monkeypatch.setattr("linkerhand_retarget.motion.linkerforce.retarget.time.sleep", fake_sleep)

    retarget._force_feedback_loop()

    assert reader.forcelist == [1, 2, 3, 4, 5]
    assert serial_port.writes == [b"force"]
    assert retarget.debug_last_force04_r == [1.0, 2.0, 3.0, 4.0, 5.0]
    assert sleeps["count"] == 1


def test_start_force_feedback_worker_starts_once(monkeypatch):
    install_ros_stubs(monkeypatch)

    from linkerhand_retarget.motion.linkerforce.retarget import Retarget

    started = []

    class FakeThread:
        def __init__(self, target, daemon):
            self.target = target
            self.daemon = daemon
            self.started = False

        def start(self):
            self.started = True
            started.append(self)

        def is_alive(self):
            return self.started

    monkeypatch.setattr("linkerhand_retarget.motion.linkerforce.retarget.threading.Thread", FakeThread)

    retarget = Retarget.__new__(Retarget)
    retarget.force_feedback_thread = None
    retarget.force_feedback_running = threading.Event()

    retarget._start_force_feedback_worker()
    retarget._start_force_feedback_worker()

    assert len(started) == 1
    assert retarget.force_feedback_running.is_set()


def test_retarget_timer_uses_mutually_exclusive_callback_group(monkeypatch):
    install_ros_stubs(monkeypatch)

    callback_groups = types.ModuleType("rclpy.callback_groups")

    class FakeMutuallyExclusiveCallbackGroup:
        pass

    callback_groups.MutuallyExclusiveCallbackGroup = FakeMutuallyExclusiveCallbackGroup
    monkeypatch.setitem(sys.modules, "rclpy.callback_groups", callback_groups)
    sys.modules.pop("linkerhand_retarget.motion.linkerforce.retarget", None)
    package = importlib.import_module("linkerhand_retarget.motion.linkerforce")
    monkeypatch.delattr(package, "retarget", raising=False)

    retarget_module = importlib.import_module(
        "linkerhand_retarget.motion.linkerforce.retarget"
    )

    assert (
        retarget_module.MutuallyExclusiveCallbackGroup
        is FakeMutuallyExclusiveCallbackGroup
    )

    retarget = retarget_module.Retarget.__new__(retarget_module.Retarget)
    retarget.retarget_callback_group = FakeMutuallyExclusiveCallbackGroup()

    assert isinstance(retarget.retarget_callback_group, FakeMutuallyExclusiveCallbackGroup)


def test_touch_subscriptions_run_on_dedicated_node_and_executor(monkeypatch):
    install_ros_stubs(monkeypatch)

    from linkerhand_retarget.motion.linkerforce.retarget import Retarget

    subscriptions = []
    added_nodes = []
    started_threads = []

    class FakeTouchNode:
        def create_subscription(self, msg_type, topic, callback, qos):
            subscriptions.append((msg_type, topic, callback, qos))
            return SimpleNamespace(topic=topic)

        def destroy_node(self):
            pass

    class FakeExecutor:
        def add_node(self, node):
            added_nodes.append(node)

        def spin(self):
            pass

        def shutdown(self):
            pass

    class FakeThread:
        def __init__(self, target, daemon):
            self.target = target
            self.daemon = daemon

        def start(self):
            started_threads.append(self)

        def is_alive(self):
            return True

        def join(self, timeout):
            pass

    retarget = Retarget.__new__(Retarget)
    retarget.node = SimpleNamespace(context="shared-context")
    retarget.touch_node = None
    retarget.touch_executor = None
    retarget.touch_executor_thread = None

    retarget_module = sys.modules[Retarget.__module__]
    monkeypatch.setattr(retarget_module, "Node", lambda _name, context: FakeTouchNode())
    monkeypatch.setattr(
        retarget_module,
        "SingleThreadedExecutor",
        lambda context: FakeExecutor(),
    )
    monkeypatch.setattr(retarget_module.threading, "Thread", FakeThread)

    retarget._start_touch_subscription_worker()

    assert [item[1] for item in subscriptions] == [
        "/cb_left_hand_matrix_touch",
        "/cb_right_hand_matrix_touch",
    ]
    assert added_nodes == [retarget.touch_node]
    assert len(started_threads) == 1
    assert started_threads[0].target == retarget.touch_executor.spin
