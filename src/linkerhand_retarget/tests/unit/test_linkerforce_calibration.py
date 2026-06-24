import sys
import types
import json
from types import SimpleNamespace


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
    sensor_msgs_msg.JointState = type("JointState", (), {})
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.PoseArray = type("PoseArray", (), {})
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


class FakeProgressBar:
    def __init__(self, *_args, **_kwargs):
        self.n = 0
        self.last_print_n = 0

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc, _traceback):
        return False

    def set_postfix_str(self, _value):
        pass

    def refresh(self):
        pass


class FakeLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []
        self.errors = []

    def info(self, msg):
        self.infos.append(msg)

    def warn(self, msg):
        self.warnings.append(msg)

    def warning(self, msg):
        self.warnings.append(msg)

    def error(self, msg):
        self.errors.append(msg)

    def debug(self, _msg):
        pass


def make_hand():
    return SimpleNamespace(
        calibrationoriginal=None,
        calibrationopose=None,
        calibrationfistpose=None,
        initialized=False,
        initialize_mapper=lambda: None,
    )


def test_calibration_progress_collects_right_only_glove(monkeypatch):
    install_ros_stubs(monkeypatch)

    from linkerhand_retarget.motion.linkerforce import retarget as retarget_module

    current_time = {"value": 0.0}

    def fake_time():
        current_time["value"] += 1.0
        return current_time["value"]

    monkeypatch.setattr(retarget_module, "tqdm", FakeProgressBar)
    monkeypatch.setattr(retarget_module.time, "time", fake_time)
    monkeypatch.setattr(retarget_module.time, "sleep", lambda _seconds: None)

    retarget = retarget_module.Retarget.__new__(retarget_module.Retarget)
    retarget.force_reader_left = None
    retarget.force_reader_right = SimpleNamespace(
        handtype="Right",
        poslist=[float(i) for i in range(20)],
    )

    retarget._calibration_with_progress(stability_window=1, stability_threshold=0.03)

    assert retarget.calibration_data_left == []
    assert len(retarget.calibration_data_right) > 0
    assert retarget.calibration_data_right[-1] == [float(i) for i in range(20)]


def test_run_calibration_completes_with_right_only_glove_opose_then_open(monkeypatch):
    install_ros_stubs(monkeypatch)

    from linkerhand_retarget.motion.linkerforce import retarget as retarget_module

    logger = FakeLogger()
    retarget = retarget_module.Retarget.__new__(retarget_module.Retarget)
    retarget.node = SimpleNamespace(get_logger=lambda: logger)
    retarget.force_reader_left = None
    retarget.force_reader_right = SimpleNamespace(handtype="Right")
    retarget.lefthand = make_hand()
    retarget.righthand = make_hand()
    retarget.show_fist_calibration = False
    retarget.fist_extend_ratio = 0.5
    retarget.calibration_in_progress = False

    samples = iter(
        [
            [10.0, 20.0, 30.0],
            [12.0, 21.0, 33.0],
        ]
    )

    def fake_collect(_stability_window):
        retarget.calibration_data_left = []
        retarget.calibration_data_right = [next(samples)]

    monkeypatch.setattr(retarget, "_calibration_with_progress", fake_collect)
    monkeypatch.setattr(retarget, "_save_to_tmp", lambda: True)

    assert retarget.run_calibration() is True
    assert retarget.lefthand.calibrationoriginal is None
    assert retarget.lefthand.calibrationopose is None
    assert retarget.lefthand.calibrationfistpose is None
    assert retarget.righthand.calibrationopose == [10.0, 20.0, 30.0]
    assert retarget.righthand.calibrationoriginal == [12.0, 21.0, 33.0]
    assert retarget.righthand.calibrationfistpose == [9.0, 19.5, 28.5]


def test_run_calibration_places_fist_first_when_enabled(monkeypatch):
    install_ros_stubs(monkeypatch)

    from linkerhand_retarget.motion.linkerforce import retarget as retarget_module

    logger = FakeLogger()
    retarget = retarget_module.Retarget.__new__(retarget_module.Retarget)
    retarget.node = SimpleNamespace(get_logger=lambda: logger)
    retarget.force_reader_left = None
    retarget.force_reader_right = SimpleNamespace(handtype="Right")
    retarget.lefthand = make_hand()
    retarget.righthand = make_hand()
    retarget.show_fist_calibration = True
    retarget.fist_extend_ratio = 0.5
    retarget.calibration_in_progress = False

    samples = iter(
        [
            [30.0, 31.0, 32.0],
            [20.0, 21.0, 22.0],
            [10.0, 11.0, 12.0],
        ]
    )

    def fake_collect(_stability_window):
        retarget.calibration_data_left = []
        retarget.calibration_data_right = [next(samples)]

    monkeypatch.setattr(retarget, "_calibration_with_progress", fake_collect)
    monkeypatch.setattr(retarget, "_save_to_tmp", lambda: True)

    assert retarget.run_calibration() is True
    assert retarget.righthand.calibrationfistpose == [30.0, 31.0, 32.0]
    assert retarget.righthand.calibrationopose == [20.0, 21.0, 22.0]
    assert retarget.righthand.calibrationoriginal == [10.0, 11.0, 12.0]


def test_save_to_tmp_accepts_right_only_calibration_without_left_reader(monkeypatch, tmp_path):
    install_ros_stubs(monkeypatch)

    from linkerhand_retarget.motion.linkerforce import retarget as retarget_module

    logger = FakeLogger()
    retarget = retarget_module.Retarget.__new__(retarget_module.Retarget)
    retarget.node = SimpleNamespace(get_logger=lambda: logger)
    retarget.force_reader_left = None
    retarget.force_reader_right = SimpleNamespace(handtype="Right")
    retarget.lefthand = make_hand()
    retarget.righthand = make_hand()
    retarget.righthand.calibrationoriginal = [0.0, 0.0, 0.0, 0.0]
    retarget.righthand.calibrationopose = [1.0, 1.0, 1.0, 1.0]
    retarget.righthand.calibrationfistpose = [2.0, 2.0, 2.0, 2.0]

    tmp_file = tmp_path / "jointangle_data.tmp"
    monkeypatch.setattr(retarget_module, "TMP_FILE_PATH", tmp_file)

    assert retarget._save_to_tmp() is True

    data = json.loads(tmp_file.read_text())
    assert data["jointangleoriginal_r"] == [0.0, 0.0, 0.0, 0.0]
    assert data["jointangleopose_r"] == [1.0, 1.0, 1.0, 1.0]
    assert data["jointanglefist_r"] == [2.0, 2.0, 2.0, 2.0]
    assert data["jointangleoriginal_l"] is None
    assert data["jointangleopose_l"] is None
    assert data["jointanglefist_l"] is None


def test_process_stops_without_entering_spin_when_calibration_fails(monkeypatch):
    install_ros_stubs(monkeypatch)

    from linkerhand_retarget import handretarget as handretarget_module

    events = {"process_called": 0, "mujoco_called": 0, "spin_called": 0}

    class FakeRetarget:
        def __init__(self, *_args, **_kwargs):
            pass

        def process(self):
            events["process_called"] += 1
            return False

    class FakeNode:
        def __init__(self):
            self.retarget = FakeRetarget()
            self.mujoco_displays = []

        def get_logger(self):
            return FakeLogger()

        def retargetrun(self):
            result = self.retarget.process()
            if result:
                self._started = True
                self._start_mujoco_display_if_enabled()
            return result

        def _start_mujoco_display_if_enabled(self):
            events["mujoco_called"] += 1

        def destroy_node(self):
            pass

    monkeypatch.setattr(handretarget_module, "HandRetargetNode", FakeNode)
    monkeypatch.setattr(handretarget_module.rclpy, "init", lambda args=None: None)
    monkeypatch.setattr(handretarget_module.rclpy, "shutdown", lambda: None)
    monkeypatch.setattr(handretarget_module.rclpy, "spin", lambda node, executor: events.__setitem__("spin_called", events["spin_called"] + 1))
    monkeypatch.setattr(handretarget_module.signal, "signal", lambda *_args, **_kwargs: None)

    handretarget_module.main()

    assert events["process_called"] == 1
    assert events["mujoco_called"] == 0
    assert events["spin_called"] == 0


def test_real_retargetrun_returns_false_and_skips_mujoco_when_process_fails(monkeypatch):
    install_ros_stubs(monkeypatch)

    from linkerhand_retarget import handretarget as handretarget_module
    from linkerhand.constants import MotionSource, RobotName

    events = {"mujoco_called": 0, "process_called": 0}

    class FakeRetarget:
        def __init__(self, *_args, **_kwargs):
            pass

        def process(self):
            events["process_called"] += 1
            return False

    node = handretarget_module.HandRetargetNode.__new__(
        handretarget_module.HandRetargetNode
    )
    node.motion_type = MotionSource.linkerforce
    node.robot_name_r = RobotName.o20
    node.robot_name_l = RobotName.o20
    node.handcore = object()
    node.lefthandprint = False
    node.righthandprint = False
    node.calibration = "auto_calibrate"
    node.baseconfig = {}
    node.cmd_ports = []
    node.cmd_baudrate = None
    node.cmd_auto_scan = False
    node.retarget = None
    node.get_logger = lambda: FakeLogger()
    node._start_mujoco_display_if_enabled = (
        lambda: events.__setitem__("mujoco_called", events["mujoco_called"] + 1)
    )

    monkeypatch.setattr(
        "linkerhand_retarget.motion.linkerforce.retarget.Retarget",
        FakeRetarget,
    )

    assert node.retargetrun() is False
    assert events["process_called"] == 1
    assert events["mujoco_called"] == 0
