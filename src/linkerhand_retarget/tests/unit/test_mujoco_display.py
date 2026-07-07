from pathlib import Path
import sys
import types
from types import SimpleNamespace

import pytest
import serial.tools.list_ports

from linkerhand_retarget.mujoco_display import (
    MujocoDisplay,
    _module_available,
    build_mujoco_display_plan,
    build_mujoco_display_plans,
    collect_mujoco_urdf_assets,
    detect_loaded_hands,
    extract_mujoco_joint_positions,
    get_urdf_mimic_joint_rules,
    prepare_mujoco_model_xml,
    transform_urdf_for_mujoco_display,
)


def test_mujoco_display_stays_disabled_without_config(tmp_path):
    plan = build_mujoco_display_plan(
        baseconfig={},
        package_dir=tmp_path,
        robot_name_r=SimpleNamespace(name="o20"),
        robot_name_l=SimpleNamespace(name="o20"),
        module_available=lambda _name: True,
    )

    assert plan.should_start is False
    assert plan.warnings == ()


def test_mujoco_display_warns_when_enabled_without_python_module(tmp_path):
    urdf_path = tmp_path / "robot.urdf"
    urdf_path.write_text("<robot/>", encoding="utf-8")

    plan = build_mujoco_display_plan(
        baseconfig={
            "mujoco": {
                "enabled": True,
            }
        },
        package_dir=tmp_path,
        robot_name_r=SimpleNamespace(name="o20"),
        robot_name_l=SimpleNamespace(name="o20"),
        urdf_paths={"right": urdf_path},
        module_available=lambda _name: False,
    )

    assert plan.should_start is False
    assert any("mujoco" in warning for warning in plan.warnings)


def test_module_available_treats_missing_nested_package_as_unavailable():
    assert _module_available("package_that_should_not_exist.viewer") is False


def test_mujoco_display_uses_mapped_urdf_when_available(tmp_path):
    urdf_path = tmp_path / "robot.urdf"
    urdf_path.write_text("<robot/>", encoding="utf-8")

    plan = build_mujoco_display_plan(
        baseconfig={
            "mujoco": {
                "enabled": True,
                "hand": "left",
                "fps": 30,
            }
        },
        package_dir=tmp_path,
        robot_name_r=SimpleNamespace(name="o20"),
        robot_name_l=SimpleNamespace(name="o30"),
        urdf_paths={"left": urdf_path},
        module_available=lambda _name: True,
    )

    assert plan.should_start is True
    assert plan.hand == "left"
    assert plan.model_path == urdf_path
    assert plan.fps == 30
    assert plan.warnings == ()


def test_mujoco_display_plan_reads_display_transform_config(tmp_path):
    urdf_path = tmp_path / "robot.urdf"
    urdf_path.write_text("<robot/>", encoding="utf-8")

    plan = build_mujoco_display_plan(
        baseconfig={
            "mujoco": {
                "enabled": True,
                "hand": "right",
                "model_scale": 5,
                "model_rotate_rpy": [0, -0.785398163397, -1.57079632679],
                "model_translate_xyz": [0, 0, -0.5],
            }
        },
        package_dir=tmp_path,
        robot_name_r=SimpleNamespace(name="o20"),
        robot_name_l=SimpleNamespace(name="o20"),
        urdf_paths={"right": urdf_path},
        module_available=lambda _name: True,
    )

    assert plan.model_scale == 5.0
    assert plan.model_rotate_rpy == (0.0, -0.785398163397, -1.57079632679)
    assert plan.model_translate_xyz == (0.0, 0.0, -0.5)


def test_mujoco_display_derives_urdf_path_from_selected_hand(tmp_path):
    package_dir = tmp_path / "linkerhand_retarget"
    model_dir = package_dir / "assets" / "robots" / "hands" / "linker_hand" / "o20_right"
    model_dir.mkdir(parents=True)
    urdf_path = model_dir / "linkerhand_o20_right.urdf"
    urdf_path.write_text("<robot/>", encoding="utf-8")

    plan = build_mujoco_display_plan(
        baseconfig={
            "mujoco": {
                "enabled": True,
                "hand": "right",
            }
        },
        package_dir=package_dir,
        robot_name_r=SimpleNamespace(name="o20"),
        robot_name_l=SimpleNamespace(name="o30"),
        module_available=lambda _name: True,
    )

    assert plan.should_start is True
    assert plan.model_path == urdf_path


def test_mujoco_display_derives_o30i_urdf_path_from_selected_hand(tmp_path):
    package_dir = tmp_path / "linkerhand_retarget"
    model_dir = package_dir / "assets" / "robots" / "hands" / "linker_hand" / "o30i_left"
    model_dir.mkdir(parents=True)
    urdf_path = model_dir / "linkerhand_o30i_left.urdf"
    urdf_path.write_text("<robot/>", encoding="utf-8")

    plan = build_mujoco_display_plan(
        baseconfig={
            "mujoco": {
                "enabled": True,
                "hand": "left",
            }
        },
        package_dir=package_dir,
        robot_name_r=SimpleNamespace(name="o30i"),
        robot_name_l=SimpleNamespace(name="o30i"),
        module_available=lambda _name: True,
    )

    assert plan.should_start is True
    assert plan.model_path == urdf_path


def test_mujoco_display_builds_two_plans_from_hands_list(tmp_path):
    right_urdf = tmp_path / "right.urdf"
    left_urdf = tmp_path / "left.urdf"
    right_urdf.write_text("<robot/>", encoding="utf-8")
    left_urdf.write_text("<robot/>", encoding="utf-8")

    plans = build_mujoco_display_plans(
        baseconfig={
            "mujoco": {
                "enabled": True,
                "hands": ["right", "left"],
                "fps": 30,
            }
        },
        package_dir=tmp_path,
        robot_name_r=SimpleNamespace(name="o20"),
        robot_name_l=SimpleNamespace(name="o30"),
        urdf_paths={"right": right_urdf, "left": left_urdf},
        module_available=lambda _name: True,
    )

    assert [plan.hand for plan in plans] == ["right", "left"]
    assert [plan.model_path for plan in plans] == [right_urdf, left_urdf]
    assert all(plan.should_start for plan in plans)


def test_mujoco_display_auto_hands_follow_enabled_outputs(tmp_path):
    right_urdf = tmp_path / "right.urdf"
    left_urdf = tmp_path / "left.urdf"
    right_urdf.write_text("<robot/>", encoding="utf-8")
    left_urdf.write_text("<robot/>", encoding="utf-8")

    plans = build_mujoco_display_plans(
        baseconfig={
            "system": {
                "rightpub": True,
                "leftpub": True,
            },
            "mujoco": {
                "enabled": True,
                "hands": "auto",
            },
        },
        package_dir=tmp_path,
        robot_name_r=SimpleNamespace(name="o20"),
        robot_name_l=SimpleNamespace(name="o30"),
        urdf_paths={"right": right_urdf, "left": left_urdf},
        module_available=lambda _name: True,
    )

    assert [plan.hand for plan in plans] == ["right", "left"]


def test_mujoco_display_auto_hands_uses_single_enabled_output(tmp_path):
    left_urdf = tmp_path / "left.urdf"
    left_urdf.write_text("<robot/>", encoding="utf-8")

    plans = build_mujoco_display_plans(
        baseconfig={
            "system": {
                "rightpub": False,
                "leftpub": True,
            },
            "mujoco": {
                "enabled": True,
                "hands": "auto",
            },
        },
        package_dir=tmp_path,
        robot_name_r=SimpleNamespace(name="o20"),
        robot_name_l=SimpleNamespace(name="o30"),
        urdf_paths={"left": left_urdf},
        module_available=lambda _name: True,
    )

    assert len(plans) == 1
    assert plans[0].hand == "left"
    assert plans[0].model_path == left_urdf


def test_detect_loaded_hands_uses_linkerforce_readers():
    retarget = SimpleNamespace(
        force_reader_left=SimpleNamespace(
            handtype="Left",
            connflag=True,
            position_frame_count=1,
            serial_port=SimpleNamespace(is_open=True),
        ),
        force_reader_right=SimpleNamespace(handtype=None),
    )

    assert detect_loaded_hands(retarget) == ("left",)


def test_detect_loaded_hands_uses_handtype_when_reader_slot_is_swapped():
    retarget = SimpleNamespace(
        force_reader_left=SimpleNamespace(
            handtype="Right",
            connflag=True,
            position_frame_count=1,
            serial_port=SimpleNamespace(is_open=True),
        ),
        force_reader_right=SimpleNamespace(handtype=None),
    )

    assert detect_loaded_hands(retarget) == ("right",)


def test_detect_loaded_hands_returns_empty_tuple_when_no_glove_connected():
    retarget = SimpleNamespace(
        force_reader_left=None,
        force_reader_right=None,
    )

    assert detect_loaded_hands(retarget) == ()


def test_detect_loaded_hands_ignores_stale_handtype_without_active_connection():
    retarget = SimpleNamespace(
        force_reader_left=SimpleNamespace(
            handtype="Left",
            connflag=False,
            serial_port=SimpleNamespace(is_open=False),
        ),
        force_reader_right=None,
    )

    assert detect_loaded_hands(retarget) == ()


def test_detect_loaded_hands_ignores_reader_without_position_frames():
    retarget = SimpleNamespace(
        force_reader_left=SimpleNamespace(
            handtype="Left",
            connflag=True,
            position_frame_count=0,
            serial_port=SimpleNamespace(is_open=True),
        ),
        force_reader_right=None,
    )

    assert detect_loaded_hands(retarget) == ()


def test_detect_loaded_hands_keeps_active_connected_reader_with_position_frames():
    retarget = SimpleNamespace(
        force_reader_left=SimpleNamespace(
            handtype="Left",
            connflag=True,
            position_frame_count=1,
            serial_port=SimpleNamespace(is_open=True),
        ),
        force_reader_right=None,
    )

    assert detect_loaded_hands(retarget) == ("left",)


def test_mujoco_display_auto_hands_stays_off_when_no_glove_loaded(tmp_path):
    right_urdf = tmp_path / "right.urdf"
    left_urdf = tmp_path / "left.urdf"
    right_urdf.write_text("<robot/>", encoding="utf-8")
    left_urdf.write_text("<robot/>", encoding="utf-8")

    plans = build_mujoco_display_plans(
        baseconfig={
            "system": {
                "rightpub": True,
                "leftpub": True,
            },
            "mujoco": {
                "enabled": True,
                "hands": "auto",
            },
        },
        package_dir=tmp_path,
        robot_name_r=SimpleNamespace(name="o20"),
        robot_name_l=SimpleNamespace(name="o30"),
        loaded_hands=(),
        urdf_paths={"right": right_urdf, "left": left_urdf},
        module_available=lambda _name: True,
    )

    assert plans == ()


def test_mujoco_display_single_plan_stays_disabled_when_no_glove_loaded(tmp_path):
    plan = build_mujoco_display_plan(
        baseconfig={
            "mujoco": {
                "enabled": True,
                "hands": "auto",
            },
        },
        package_dir=tmp_path,
        robot_name_r=SimpleNamespace(name="o20"),
        robot_name_l=SimpleNamespace(name="o30"),
        loaded_hands=(),
        module_available=lambda _name: True,
    )

    assert plan.enabled is True
    assert plan.should_start is False
    assert plan.model_path is None


def test_mujoco_display_auto_hands_uses_swapped_reader_handtype(tmp_path):
    right_urdf = tmp_path / "right.urdf"
    left_urdf = tmp_path / "left.urdf"
    right_urdf.write_text("<robot/>", encoding="utf-8")
    left_urdf.write_text("<robot/>", encoding="utf-8")
    retarget = SimpleNamespace(
        force_reader_left=SimpleNamespace(
            handtype="Right",
            connflag=True,
            position_frame_count=1,
            serial_port=SimpleNamespace(is_open=True),
        ),
        force_reader_right=SimpleNamespace(handtype=None),
    )

    plans = build_mujoco_display_plans(
        baseconfig={
            "system": {
                "rightpub": True,
                "leftpub": True,
            },
            "mujoco": {
                "enabled": True,
                "hands": "auto",
            },
        },
        package_dir=tmp_path,
        robot_name_r=SimpleNamespace(name="o6"),
        robot_name_l=SimpleNamespace(name="o6"),
        loaded_hands=detect_loaded_hands(retarget),
        urdf_paths={"right": right_urdf, "left": left_urdf},
        module_available=lambda _name: True,
    )

    assert [plan.hand for plan in plans] == ["right"]
    assert plans[0].model_path == right_urdf


def test_mujoco_display_auto_hands_prefers_loaded_hands(tmp_path):
    left_urdf = tmp_path / "left.urdf"
    left_urdf.write_text("<robot/>", encoding="utf-8")

    plans = build_mujoco_display_plans(
        baseconfig={
            "system": {
                "rightpub": True,
                "leftpub": True,
            },
            "mujoco": {
                "enabled": True,
                "hands": "auto",
            },
        },
        package_dir=tmp_path,
        robot_name_r=SimpleNamespace(name="o20"),
        robot_name_l=SimpleNamespace(name="o30"),
        loaded_hands=("left",),
        urdf_paths={"left": left_urdf},
        module_available=lambda _name: True,
    )

    assert len(plans) == 1
    assert plans[0].hand == "left"


def test_linkerforce_candidate_init_seeds_detected_left_reader(monkeypatch):
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

    from linkerhand_retarget.motion.linkerforce import retarget as retarget_module

    created_readers = []

    class FakeSerialPort:
        def __init__(self, reader):
            self.reader = reader
            self.is_open = True

        def write(self, _data):
            if self.reader.detects_on_query and self.reader.version_query_count >= 10:
                self.reader.handtype = "Left"
                self.reader.version = "1.2.3"
                self.reader.connflag = True
                self.reader.position_frame_count = 1

    class FakeForceSerialReader:
        def __init__(self, *_args, **_kwargs):
            self.detects_on_query = len(created_readers) == 0
            self.handtype = None
            self.version = None
            self.connflag = False
            self.position_frame_count = 0
            self.start_count = 0
            self.version_query_count = 0
            self.serial_port = FakeSerialPort(self)
            created_readers.append(self)

        def openserial(self, port, baudrate):
            self.port = port
            self.baudrate = baudrate
            self.serial_port = FakeSerialPort(self)
            return True

        def query_version_sync(self):
            for _ in range(10):
                self.version_query_count += 1
                self.serial_port.write(self.pack_01_data())
                if self.handtype:
                    return True
            return False

        def start(self):
            self.start_count += 1
            if self.handtype:
                self.connflag = True
                self.position_frame_count = 1

        def stop(self):
            pass

        def pack_01_data(self):
            return b"version-query"

    class FakeLogger:
        def info(self, _msg):
            pass

        def warn(self, _msg):
            pass

        def error(self, _msg):
            pass

        def debug(self, _msg):
            pass

    monkeypatch.setattr(retarget_module, "ForceSerialReader", FakeForceSerialReader)
    monkeypatch.setattr(retarget_module.os.path, "exists", lambda _path: True)
    monkeypatch.setattr(retarget_module.time, "sleep", lambda _seconds: None)

    retarget = retarget_module.Retarget.__new__(retarget_module.Retarget)
    retarget.node = SimpleNamespace(get_logger=lambda: FakeLogger())
    retarget.lefthand = SimpleNamespace(set_glove_version=lambda _version: None)
    retarget.righthand = SimpleNamespace(set_glove_version=lambda _version: None)
    retarget.leftport = None
    retarget.leftbaudrate = None
    retarget.rightport = None
    retarget.rightbaudrate = None
    retarget.force_reader_left = None
    retarget.force_reader_right = None

    left_found, right_found = retarget._init_from_candidates(
        ["/dev/ttyUSB1"],
        [2000000],
        [],
        False,
        lambda _level, _msg: None,
    )

    assert left_found is True
    assert right_found is False
    assert created_readers[0].version_query_count == 10
    assert created_readers[0].start_count == 0
    assert retarget.force_reader_left is created_readers[1]
    assert retarget.force_reader_left.handtype == "Left"
    assert retarget.force_reader_left.version == "1.2.3"
    assert detect_loaded_hands(retarget) == ("left",)


def test_linkerforce_auto_scan_detects_actual_right_hand_before_saved_ports(monkeypatch):
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

    from linkerhand_retarget.motion.linkerforce import retarget as retarget_module

    created_readers = []

    class FakeSerialPort:
        def __init__(self, reader):
            self.reader = reader

        def write(self, _data):
            if self.reader.detects_on_query and self.reader.version_query_count >= 10:
                self.reader.handtype = "Right"
                self.reader.version = "2.1.5"

    class FakeForceSerialReader:
        def __init__(self, *_args, **_kwargs):
            self.detects_on_query = len(created_readers) == 0
            self.handtype = None
            self.version = None
            self.start_count = 0
            self.version_query_count = 0
            self.serial_port = FakeSerialPort(self)
            created_readers.append(self)

        def openserial(self, port, baudrate):
            self.port = port
            self.baudrate = baudrate
            return port == "/dev/ttyUSB0"

        def query_version_sync(self):
            for _ in range(10):
                self.version_query_count += 1
                self.serial_port.write(self.pack_01_data())
                if self.handtype:
                    return True
            return False

        def start(self):
            self.start_count += 1

        def stop(self):
            pass

        def pack_01_data(self):
            return b"version-query"

    class FakeLogger:
        def info(self, _msg):
            pass

        def warn(self, _msg):
            pass

        def error(self, _msg):
            pass

        def debug(self, _msg):
            pass

    monkeypatch.setattr(retarget_module, "ForceSerialReader", FakeForceSerialReader)
    monkeypatch.setattr(retarget_module.os.path, "exists", lambda path: path == "/dev/ttyUSB0")
    monkeypatch.setattr(retarget_module.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(
        serial.tools.list_ports,
        "comports",
        lambda: [SimpleNamespace(device="/dev/ttyUSB0")],
    )

    retarget = retarget_module.Retarget.__new__(retarget_module.Retarget)
    retarget.node = SimpleNamespace(get_logger=lambda: FakeLogger())
    retarget.baseconfig = {
        "serial": {
            "auto_scan": True,
            "baudrates": [2000000],
            "exclude_ports": [],
            "left": {"port": "/dev/ttyUSB0", "baudrate": 2000000},
            "right": {"port": "/dev/ttyUSB1", "baudrate": 2000000},
        },
        "calibration": {},
    }
    retarget.cmd_ports = None
    retarget.cmd_baudrate = None
    retarget.cmd_auto_scan = None
    retarget.calibration = False
    retarget.lefthand = SimpleNamespace(
        set_glove_version=lambda _version: None,
        initialize_mapper=lambda: None,
    )
    retarget.righthand = SimpleNamespace(
        set_glove_version=lambda _version: None,
        initialize_mapper=lambda: None,
    )
    retarget.leftport = None
    retarget.leftbaudrate = None
    retarget.rightport = None
    retarget.rightbaudrate = None
    retarget.force_reader_left = None
    retarget.force_reader_right = None
    retarget._save_serial_to_config = lambda _left_found, _right_found: None
    retarget._load_from_tmp = lambda: True

    retarget.linkerforce_init()

    assert retarget.force_reader_left is None
    assert retarget.force_reader_right is created_readers[1]
    assert retarget.force_reader_right.handtype == "Right"
    assert retarget.rightport == "/dev/ttyUSB0"
    assert retarget.rightbaudrate == 2000000


def test_mujoco_display_keeps_legacy_hand_config(tmp_path):
    right_urdf = tmp_path / "right.urdf"
    right_urdf.write_text("<robot/>", encoding="utf-8")

    plans = build_mujoco_display_plans(
        baseconfig={
            "mujoco": {
                "enabled": True,
                "hand": "right",
            }
        },
        package_dir=tmp_path,
        robot_name_r=SimpleNamespace(name="o20"),
        robot_name_l=SimpleNamespace(name="o30"),
        urdf_paths={"right": right_urdf},
        module_available=lambda _name: True,
    )

    assert len(plans) == 1
    assert plans[0].hand == "right"
    assert plans[0].model_path == right_urdf


def test_collect_mujoco_urdf_assets_maps_mesh_basename(tmp_path):
    urdf_path = tmp_path / "robot.urdf"
    mesh_dir = tmp_path / "meshes"
    mesh_dir.mkdir()
    mesh_path = mesh_dir / "hand_base_link.STL"
    mesh_bytes = b"solid hand_base_link\nendsolid hand_base_link\n"
    mesh_path.write_bytes(mesh_bytes)
    urdf_path.write_text(
        """
<robot name="test">
  <link name="base">
    <visual>
      <geometry>
        <mesh filename="meshes/hand_base_link.STL" />
      </geometry>
    </visual>
  </link>
</robot>
""",
        encoding="utf-8",
    )

    assets = collect_mujoco_urdf_assets(urdf_path)

    assert assets == {"hand_base_link.STL": mesh_bytes}


def test_prepare_mujoco_model_xml_injects_urdf_compiler_options(tmp_path):
    urdf_path = tmp_path / "robot.urdf"
    urdf_path.write_text('<robot name="test"><link name="base" /></robot>', encoding="utf-8")

    xml = prepare_mujoco_model_xml(urdf_path)

    robot = transform_urdf_for_mujoco_display(xml)
    compiler = robot.find("./mujoco/compiler")

    assert compiler is not None
    assert compiler.attrib["balanceinertia"] == "true"
    assert compiler.attrib["boundmass"] == "1e-06"
    assert compiler.attrib["boundinertia"] == "1e-09"


def test_prepare_mujoco_model_xml_applies_display_transform(tmp_path):
    urdf_path = tmp_path / "robot.urdf"
    urdf_path.write_text(
        """
<robot name="test">
  <link name="base">
    <visual>
      <origin xyz="1 2 3" rpy="0 0 0" />
      <geometry>
        <mesh filename="mesh.stl" />
      </geometry>
    </visual>
  </link>
</robot>
""",
        encoding="utf-8",
    )

    xml = prepare_mujoco_model_xml(
        urdf_path,
        model_scale=2.5,
        model_rotate_rpy=(1.57079632679, 0.0, 0.0),
    )
    robot = transform_urdf_for_mujoco_display(xml)

    assert robot.find("./link[@name='mujoco_display_world']") is not None
    transform_joint = robot.find("./joint[@name='mujoco_display_transform']")
    assert transform_joint is not None
    assert transform_joint.attrib["type"] == "fixed"
    assert transform_joint.find("child").attrib["link"] == "base"
    assert transform_joint.find("origin").attrib["rpy"] == "1.57079632679 0 0"
    assert robot.find(".//mesh").attrib["scale"] == "2.5 2.5 2.5"
    assert robot.find(".//visual/origin").attrib["xyz"] == "2.5 5 7.5"


def test_prepare_mujoco_model_xml_applies_display_translation(tmp_path):
    urdf_path = tmp_path / "robot.urdf"
    urdf_path.write_text('<robot name="test"><link name="base" /></robot>', encoding="utf-8")

    xml = prepare_mujoco_model_xml(
        urdf_path,
        model_translate_xyz=(0.0, 0.0, -0.5),
    )
    robot = transform_urdf_for_mujoco_display(xml)

    transform_joint = robot.find("./joint[@name='mujoco_display_transform']")
    assert transform_joint is not None
    assert transform_joint.find("origin").attrib["xyz"] == "0 0 -0.5"


def test_prepare_mujoco_model_xml_keeps_o20_right_ring_roll_axis_unchanged(tmp_path):
    model_dir = tmp_path / "o20_right"
    model_dir.mkdir()
    urdf_path = model_dir / "linkerhand_o20_right.urdf"
    urdf_path.write_text(
        """
        <robot name="o20_right">
          <link name="base" />
          <link name="index" />
          <link name="ring" />
          <joint name="index_mcp_roll" type="revolute">
            <parent link="base" />
            <child link="index" />
            <axis xyz="1 0 0" />
            <limit lower="-0.35" upper="0.09" effort="1" velocity="1" />
          </joint>
          <joint name="ring_mcp_roll" type="revolute">
            <parent link="base" />
            <child link="ring" />
            <axis xyz="-1 0 0" />
            <limit lower="-0.2" upper="0.2" effort="1" velocity="1" />
          </joint>
        </robot>
        """,
        encoding="utf-8",
    )

    xml = prepare_mujoco_model_xml(urdf_path)
    robot = transform_urdf_for_mujoco_display(xml)

    index_axis = robot.find("./joint[@name='index_mcp_roll']/axis")
    ring_axis = robot.find("./joint[@name='ring_mcp_roll']/axis")
    assert index_axis.attrib["xyz"] == "1 0 0"
    assert ring_axis.attrib["xyz"] == "-1 0 0"


def test_extract_mujoco_joint_positions_uses_latest_handcore_qpos():
    handcore = SimpleNamespace(
        last_qpos_r=[0.1, 0.2, 0.3],
        sourcedataindex_r=[2, "None", 1],
        urdfdataindex_r=[1, "None", 0],
    )

    positions = extract_mujoco_joint_positions(
        handcore,
        hand="right",
        movable_joint_names=["joint0", "joint1"],
    )

    assert positions == {"joint1": 0.3, "joint0": 0.2}


def test_extract_mujoco_joint_positions_prefers_hand_arc_radians():
    hand = SimpleNamespace(
        g_jointpositions_arc=[9.0, 0.1, 9.1, 0.2, 9.2, 0.3],
        mujoco_joint_arc_indices=[1, 3, 5],
        mujoco_joint_arc_signs=[1.0, -1.0, 1.0],
    )
    handcore = SimpleNamespace(
        last_qpos_r=[9.1, 9.2, 9.3],
        sourcedataindex_r=[2, 1, 0],
        urdfdataindex_r=[0, 1, 2],
    )

    positions = extract_mujoco_joint_positions(
        handcore,
        hand="right",
        movable_joint_names=["thumb_cmc_roll", "thumb_cmc_yaw", "thumb_cmc_pitch"],
        hand_model=hand,
    )

    assert positions == {
        "thumb_cmc_roll": 0.1,
        "thumb_cmc_yaw": -0.2,
        "thumb_cmc_pitch": 0.3,
    }


def test_extract_mujoco_joint_positions_applies_piecewise_arc_remap():
    hand = SimpleNamespace(
        g_jointpositions_arc=[1.0],
        mujoco_joint_arc_indices=[0],
        mujoco_joint_arc_signs=[1.0],
        mujoco_joint_arc_remaps=[
            (
                (0.0, 0.0),
                (1.0, 10.0),
                (2.0, 11.0),
            )
        ],
    )

    positions = extract_mujoco_joint_positions(
        None,
        hand="right",
        movable_joint_names=["thumb_cmc_roll"],
        hand_model=hand,
    )

    assert positions["thumb_cmc_roll"] == pytest.approx(10.0)


def test_get_urdf_mimic_joint_rules_reads_multiplier_and_offset(tmp_path):
    urdf_path = tmp_path / "robot.urdf"
    urdf_path.write_text(
        """
        <robot name="test">
          <link name="base" />
          <joint name="driver" type="revolute">
            <parent link="base" /><child link="a" />
          </joint>
          <joint name="follower" type="revolute">
            <parent link="a" /><child link="b" />
            <mimic joint="driver" multiplier="1.83" offset="0.1" />
          </joint>
        </robot>
        """,
        encoding="utf-8",
    )

    assert get_urdf_mimic_joint_rules(urdf_path) == {
        "follower": ("driver", 1.83, 0.1)
    }


def test_mujoco_display_updates_named_joint_qpos_and_syncs():
    class FakeMujoco:
        class mjtObj:
            mjOBJ_JOINT = object()

        def __init__(self):
            self.forward_count = 0

        def mj_name2id(self, model, _obj_type, name):
            return model.name_to_id.get(name, -1)

        def mj_forward(self, _model, _data):
            self.forward_count += 1

    class FakeModel:
        name_to_id = {"thumb_cmc_roll": 0, "index_mcp_pitch": 1}
        jnt_qposadr = [1, 3]

    class FakeData:
        def __init__(self):
            self.qpos = [0.0] * 4

    class FakeLock:
        def __enter__(self):
            return self

        def __exit__(self, _exc_type, _exc, _traceback):
            return False

    class FakeViewer:
        def __init__(self):
            self.sync_count = 0

        def lock(self):
            return FakeLock()

        def is_running(self):
            return True

        def sync(self):
            self.sync_count += 1

    fake_mujoco = FakeMujoco()
    display = MujocoDisplay(Path("robot.urdf"), hand="right", mujoco_module=fake_mujoco)
    display.model = FakeModel()
    display.data = FakeData()
    display.viewer = FakeViewer()

    display.update_joint_positions(
        {
            "thumb_cmc_roll": 1.2,
            "missing_joint": 9.9,
            "index_mcp_pitch": 0.4,
        }
    )

    assert display.data.qpos == [0.0, 1.2, 0.0, 0.4]
    assert fake_mujoco.forward_count == 1
    assert display.viewer.sync_count == 1


def test_mujoco_display_applies_mimic_joints_from_urdf(tmp_path):
    urdf_path = tmp_path / "robot.urdf"
    urdf_path.write_text(
        """
        <robot name="test">
          <link name="base" />
          <joint name="thumb_cmc_pitch" type="revolute">
            <parent link="base" /><child link="thumb1" />
          </joint>
          <joint name="thumb_ip" type="revolute">
            <parent link="thumb1" /><child link="thumb2" />
            <mimic joint="thumb_cmc_pitch" multiplier="1.83" offset="0.1" />
          </joint>
          <joint name="index_mcp_pitch" type="revolute">
            <parent link="base" /><child link="index1" />
          </joint>
          <joint name="index_dip" type="revolute">
            <parent link="index1" /><child link="index2" />
            <mimic joint="index_mcp_pitch" multiplier="0.89" offset="0" />
          </joint>
        </robot>
        """,
        encoding="utf-8",
    )

    class FakeMujoco:
        class mjtObj:
            mjOBJ_JOINT = object()

        def __init__(self):
            self.forward_count = 0

        def mj_name2id(self, model, _obj_type, name):
            return model.name_to_id.get(name, -1)

        def mj_forward(self, _model, _data):
            self.forward_count += 1

    class FakeModel:
        name_to_id = {
            "thumb_cmc_pitch": 0,
            "thumb_ip": 1,
            "index_mcp_pitch": 2,
            "index_dip": 3,
        }
        jnt_qposadr = [0, 1, 2, 3]

    class FakeData:
        def __init__(self):
            self.qpos = [0.0] * 4

    class FakeViewer:
        sync_count = 0

        def is_running(self):
            return True

        def sync(self):
            self.sync_count += 1

    fake_mujoco = FakeMujoco()
    display = MujocoDisplay(urdf_path, hand="right", mujoco_module=fake_mujoco)
    display.model = FakeModel()
    display.data = FakeData()
    display.viewer = FakeViewer()

    display.update_joint_positions(
        {
            "thumb_cmc_pitch": 2.0,
            "index_mcp_pitch": 3.0,
        }
    )

    assert display.data.qpos == pytest.approx([2.0, 3.76, 3.0, 2.67])
    assert fake_mujoco.forward_count == 1
    assert display.viewer.sync_count == 1
