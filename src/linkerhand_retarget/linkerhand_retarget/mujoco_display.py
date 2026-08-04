import importlib.util
import json
import subprocess
import sys
import xml.etree.ElementTree as ET
from contextlib import nullcontext
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable, Mapping, Optional, Sequence, Tuple


_MUJOCO_DISPLAY_MIN_MASS = 1e-6
_MUJOCO_DISPLAY_MIN_INERTIA = 1e-9


@dataclass(frozen=True)
class MujocoDisplayPlan:
    should_start: bool
    enabled: bool
    hand: str
    model_path: Optional[Path]
    fps: int
    warnings: Tuple[str, ...]
    model_scale: float = 1.0
    model_rotate_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    model_translate_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)


def _module_available(name: str) -> bool:
    try:
        return importlib.util.find_spec(name) is not None
    except ModuleNotFoundError:
        return False


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def _robot_name_value(robot_name) -> str:
    return getattr(robot_name, "name", str(robot_name))


def _derive_urdf_path(package_dir: Path, hand: str, robot_name_r, robot_name_l) -> Path:
    robot_name = _robot_name_value(robot_name_l if hand == "left" else robot_name_r)
    return (
        package_dir
        / "assets"
        / "robots"
        / "hands"
        / "linker_hand"
        / f"{robot_name}_{hand}"
        / f"linkerhand_{robot_name}_{hand}.urdf"
    )


def _normalize_auto_hands(baseconfig: Mapping, loaded_hands: Optional[Iterable[str]]) -> Tuple[str, ...]:
    if loaded_hands is not None:
        loaded_hands = tuple(loaded_hands)
        if not loaded_hands:
            return ()
        normalized = tuple(
            hand
            for hand in ("right", "left")
            if hand in {str(item).strip().lower() for item in loaded_hands}
        )
        if normalized:
            return normalized

    system_config = baseconfig.get("system", {}) if isinstance(baseconfig, Mapping) else {}
    hands = []
    if _as_bool(system_config.get("rightpub", True)):
        hands.append("right")
    if _as_bool(system_config.get("leftpub", True)):
        hands.append("left")
    return tuple(hands)


def _normalize_hands(
    baseconfig: Mapping,
    mujoco_config: Mapping,
    loaded_hands: Optional[Iterable[str]],
) -> Tuple[str, ...]:
    hands = mujoco_config.get("hands")
    if hands is None:
        return (str(mujoco_config.get("hand", "right")).strip().lower(),)
    if isinstance(hands, str) and hands.strip().lower() == "auto":
        return _normalize_auto_hands(baseconfig, loaded_hands)
    if isinstance(hands, str):
        hands = [hands]

    normalized = []
    for hand in hands if isinstance(hands, Iterable) else []:
        hand = str(hand).strip().lower()
        if hand and hand not in normalized:
            normalized.append(hand)
    return tuple(normalized or ["right"])


def _mapped_urdf_path(urdf_paths: Optional[Mapping[str, Path]], hand: str) -> Optional[Path]:
    if not urdf_paths:
        return None
    urdf_path = urdf_paths.get(hand)
    return Path(urdf_path) if urdf_path else None


def _as_float(value, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _normalize_rpy(value) -> Tuple[float, float, float]:
    if value is None:
        return (0.0, 0.0, 0.0)
    if isinstance(value, str):
        value = value.replace(",", " ").split()
    if not isinstance(value, Iterable):
        return (0.0, 0.0, 0.0)

    rpy = [_as_float(item, 0.0) for item in list(value)[:3]]
    while len(rpy) < 3:
        rpy.append(0.0)
    return tuple(rpy)


def _normalize_xyz(value) -> Tuple[float, float, float]:
    return _normalize_rpy(value)


def _format_float(value: float) -> str:
    if abs(float(value)) < 1e-12:
        value = 0.0
    text = f"{float(value):.12f}".rstrip("0").rstrip(".")
    return text if text else "0"


def _format_xyz(values: Sequence[float]) -> str:
    return " ".join(_format_float(value) for value in values)


def _scale_xyz_text(text: str, scale: float) -> str:
    values = [_as_float(item, 0.0) * scale for item in text.split()]
    while len(values) < 3:
        values.append(0.0)
    return _format_xyz(values[:3])


def _scale_mesh(mesh: ET.Element, scale: float):
    current = mesh.attrib.get("scale")
    if current:
        values = [_as_float(item, 1.0) * scale for item in current.split()]
        while len(values) < 3:
            values.append(scale)
    else:
        values = [scale, scale, scale]
    mesh.set("scale", _format_xyz(values[:3]))


def _find_root_link_name(root: ET.Element) -> Optional[str]:
    link_names = [link.attrib.get("name") for link in root.findall("link")]
    link_names = [name for name in link_names if name]
    child_names = {
        child.attrib.get("link")
        for child in root.findall("./joint/child")
        if child.attrib.get("link")
    }
    for link_name in link_names:
        if link_name not in child_names:
            return link_name
    return link_names[0] if link_names else None


def _apply_display_transform(
    root: ET.Element,
    model_scale: float,
    model_rotate_rpy: Tuple[float, float, float],
    model_translate_xyz: Tuple[float, float, float],
) -> ET.Element:
    if abs(model_scale - 1.0) > 1e-9:
        for origin in root.findall(".//origin"):
            xyz = origin.attrib.get("xyz")
            if xyz:
                origin.set("xyz", _scale_xyz_text(xyz, model_scale))
        for mesh in root.findall(".//mesh"):
            _scale_mesh(mesh, model_scale)

    has_rotation = any(abs(value) > 1e-9 for value in model_rotate_rpy)
    has_translation = any(abs(value) > 1e-9 for value in model_translate_xyz)
    if has_rotation or has_translation:
        root_link_name = _find_root_link_name(root)
        if root_link_name and root.find("./link[@name='mujoco_display_world']") is None:
            root.insert(0, ET.Element("link", {"name": "mujoco_display_world"}))
            joint = ET.Element(
                "joint",
                {"name": "mujoco_display_transform", "type": "fixed"},
            )
            ET.SubElement(
                joint,
                "origin",
                {
                    "xyz": _format_xyz(model_translate_xyz),
                    "rpy": _format_xyz(model_rotate_rpy),
                },
            )
            ET.SubElement(joint, "parent", {"link": "mujoco_display_world"})
            ET.SubElement(joint, "child", {"link": root_link_name})
            root.insert(1, joint)
    return root


def detect_loaded_hands(retarget) -> Optional[Tuple[str, ...]]:
    if retarget is None:
        return None

    hands = []
    readers = (
        getattr(retarget, "force_reader_right", None),
        getattr(retarget, "force_reader_left", None),
    )

    for reader in readers:
        if reader is None:
            continue
        if not bool(getattr(reader, "connflag", False)):
            continue
        if int(getattr(reader, "position_frame_count", 0) or 0) <= 0:
            continue
        serial_port = getattr(reader, "serial_port", None)
        if serial_port is not None and not bool(getattr(serial_port, "is_open", False)):
            continue
        handtype = str(getattr(reader, "handtype", "") or "").strip().lower()
        if handtype == "right" and "right" not in hands:
            hands.append("right")
        elif handtype == "left" and "left" not in hands:
            hands.append("left")

    if hands:
        return tuple(hands)
    return ()


def collect_mujoco_urdf_assets(model_path: Path) -> Mapping[str, bytes]:
    model_path = Path(model_path)
    if model_path.suffix.lower() != ".urdf":
        return {}

    root = ET.parse(model_path).getroot()
    assets = {}
    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib.get("filename")
        if not filename:
            continue
        if filename.startswith("package://"):
            continue

        mesh_path = Path(filename)
        if not mesh_path.is_absolute():
            mesh_path = model_path.parent / mesh_path
        if mesh_path.exists():
            assets[mesh_path.name] = mesh_path.read_bytes()
    return assets


def transform_urdf_for_mujoco_display(xml) -> ET.Element:
    if isinstance(xml, ET.Element):
        return xml
    return ET.fromstring(xml)


def prepare_mujoco_model_xml(
    model_path: Path,
    model_scale: float = 1.0,
    model_rotate_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    model_translate_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Optional[str]:
    model_path = Path(model_path)
    if model_path.suffix.lower() != ".urdf":
        return None

    root = ET.parse(model_path).getroot()
    mujoco_elem = root.find("mujoco")
    if mujoco_elem is None:
        mujoco_elem = ET.Element("mujoco")
        root.insert(0, mujoco_elem)

    compiler = mujoco_elem.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(mujoco_elem, "compiler")
    compiler.set("balanceinertia", "true")
    compiler.set("boundmass", str(_MUJOCO_DISPLAY_MIN_MASS))
    compiler.set("boundinertia", str(_MUJOCO_DISPLAY_MIN_INERTIA))

    _apply_display_transform(
        root,
        float(model_scale),
        _normalize_rpy(model_rotate_rpy),
        _normalize_xyz(model_translate_xyz),
    )
    return ET.tostring(root, encoding="unicode")


def get_urdf_movable_joint_names(model_path: Path) -> Tuple[str, ...]:
    model_path = Path(model_path)
    if model_path.suffix.lower() != ".urdf" or not model_path.exists():
        return ()

    root = ET.parse(model_path).getroot()
    return tuple(
        joint.attrib["name"]
        for joint in root.findall("joint")
        if joint.attrib.get("type") != "fixed" and joint.attrib.get("name")
    )


def get_urdf_mimic_joint_rules(model_path: Path) -> Mapping[str, Tuple[str, float, float]]:
    model_path = Path(model_path)
    if model_path.suffix.lower() != ".urdf" or not model_path.exists():
        return {}

    root = ET.parse(model_path).getroot()
    rules = {}
    for joint in root.findall("joint"):
        joint_name = joint.attrib.get("name")
        mimic = joint.find("mimic")
        if not joint_name or mimic is None:
            continue

        source_joint = mimic.attrib.get("joint")
        if not source_joint:
            continue
        multiplier = _as_float(mimic.attrib.get("multiplier"), 1.0)
        offset = _as_float(mimic.attrib.get("offset"), 0.0)
        rules[joint_name] = (source_joint, multiplier, offset)
    return rules


def _optional_index(value) -> Optional[int]:
    if value is None or value == "None":
        return None
    return int(value)


def _normalize_arc_remap(remap):
    if remap is None:
        return None

    if len(remap) == 4 and not isinstance(remap[0], Iterable):
        source_a, source_b, target_a, target_b = [float(item) for item in remap]
        return ((source_a, target_a), (source_b, target_b))

    return tuple((float(source), float(target)) for source, target in remap)


def _apply_arc_remap(value: float, remap) -> float:
    points = _normalize_arc_remap(remap)
    if not points or len(points) < 2:
        return value

    for (source_a, target_a), (source_b, target_b) in zip(points, points[1:]):
        if min(source_a, source_b) <= value <= max(source_a, source_b):
            if abs(source_b - source_a) < 1e-12:
                return target_b
            ratio = (value - source_a) / (source_b - source_a)
            ratio = min(1.0, max(0.0, ratio))
            return target_a + ratio * (target_b - target_a)

    return min(points, key=lambda point: abs(value - point[0]))[1]


def _infer_six_motor_arc_index(joint_name: str) -> Optional[int]:
    name = joint_name.lower()
    if "thumb" in name:
        if "roll" in name or "yaw" in name:
            return 1
        return 0
    if "index" in name:
        return 2
    if "middle" in name:
        return 3
    if "ring" in name:
        return 4
    if "pinky" in name or "little" in name:
        return 5
    return None


def _infer_ten_motor_arc_index(joint_name: str) -> Optional[int]:
    name = joint_name.lower()
    if "thumb" in name:
        if "roll" in name:
            return 9
        if "yaw" in name:
            return 1
        return 0
    if "index" in name:
        return 6 if "roll" in name else 2
    if "middle" in name:
        return 3
    if "ring" in name:
        return 7 if "roll" in name else 4
    if "pinky" in name or "little" in name:
        return 8 if "roll" in name else 5
    return None


def _infer_twenty_motor_arc_index(joint_name: str, position: int, joint_count: int) -> Optional[int]:
    by_position = (
        0, 1, 2, 3, 3,
        5, 6, 7, 7,
        9, 10, 11, 11,
        13, 14, 15, 15,
        17, 18, 19, 19,
    )
    if joint_count == len(by_position) and position < len(by_position):
        return by_position[position]

    name = joint_name.lower()
    if "thumb" in name:
        if "joint0" in name or "roll" in name:
            return 0
        if "joint1" in name or "yaw" in name:
            return 1
        if "joint2" in name or "pitch" in name:
            return 2
        return 3
    if "index" in name:
        if "joint0" in name or "roll" in name:
            return 5
        if "joint1" in name or "pitch" in name:
            return 6
        return 7
    if "middle" in name:
        if "joint0" in name or "roll" in name:
            return 9
        if "joint1" in name or "pitch" in name:
            return 10
        return 11
    if "ring" in name:
        if "joint0" in name or "roll" in name:
            return 13
        if "joint1" in name or "pitch" in name:
            return 14
        return 15
    if "pinky" in name or "little" in name:
        if "joint0" in name or "roll" in name:
            return 17
        if "joint1" in name or "pitch" in name:
            return 18
        return 19
    return None


def _infer_mujoco_joint_positions_from_arcs(
    arc_values: Sequence[float],
    movable_joint_names: Sequence[str],
) -> Mapping[str, float]:
    arc_count = len(arc_values)
    joint_count = len(movable_joint_names)
    if arc_count == joint_count:
        return {
            joint_name: float(value)
            for joint_name, value in zip(movable_joint_names, arc_values)
        }

    positions = {}
    for position, joint_name in enumerate(movable_joint_names):
        arc_idx = None
        if arc_count == 6:
            arc_idx = _infer_six_motor_arc_index(joint_name)
        elif arc_count == 10:
            arc_idx = _infer_ten_motor_arc_index(joint_name)
        elif arc_count == 20:
            arc_idx = _infer_twenty_motor_arc_index(joint_name, position, joint_count)

        if arc_idx is None or arc_idx >= arc_count:
            continue
        positions[joint_name] = float(arc_values[arc_idx])

    return positions


def extract_mujoco_joint_positions(
    handcore,
    hand: str,
    movable_joint_names: Sequence[str],
    hand_model=None,
) -> Mapping[str, float]:
    arc_values = getattr(hand_model, "g_jointpositions_arc", None)
    if arc_values is not None:
        arc_indices = getattr(hand_model, "mujoco_joint_arc_indices", None)
        arc_signs = getattr(hand_model, "mujoco_joint_arc_signs", None)
        arc_remaps = getattr(hand_model, "mujoco_joint_arc_remaps", None)
        arc_mirrors = getattr(hand_model, "mujoco_joint_arc_mirrors", None)
        if arc_indices:
            positions = {}
            for idx, (joint_name, arc_idx) in enumerate(zip(movable_joint_names, arc_indices)):
                arc_idx = _optional_index(arc_idx)
                if arc_idx is None or arc_idx >= len(arc_values):
                    continue
                sign = float(arc_signs[idx]) if arc_signs and idx < len(arc_signs) else 1.0
                value = float(arc_values[arc_idx]) * sign
                remap = arc_remaps[idx] if arc_remaps and idx < len(arc_remaps) else None
                if remap is not None:
                    value = _apply_arc_remap(value, remap)
                else:
                    mirror = arc_mirrors[idx] if arc_mirrors and idx < len(arc_mirrors) else None
                    if mirror is not None:
                        lower, upper = mirror
                        lower = float(lower)
                        upper = float(upper)
                        value = lower + upper - value
                        value = min(upper, max(lower, value))
                positions[joint_name] = value
            if positions:
                return positions

        positions = _infer_mujoco_joint_positions_from_arcs(arc_values, movable_joint_names)
        if positions:
            return positions

    if hand == "left":
        qpos = getattr(handcore, "last_qpos_l", None)
        source_indices = getattr(handcore, "sourcedataindex_l", ())
        urdf_indices = getattr(handcore, "urdfdataindex_l", ())
    else:
        qpos = getattr(handcore, "last_qpos_r", None)
        source_indices = getattr(handcore, "sourcedataindex_r", ())
        urdf_indices = getattr(handcore, "urdfdataindex_r", ())

    if qpos is None:
        return {}

    positions = {}
    for source_idx, urdf_idx in zip(source_indices, urdf_indices):
        source_idx = _optional_index(source_idx)
        urdf_idx = _optional_index(urdf_idx)
        if source_idx is None or urdf_idx is None:
            continue
        if source_idx >= len(qpos) or urdf_idx >= len(movable_joint_names):
            continue
        positions[movable_joint_names[urdf_idx]] = float(qpos[source_idx])
    return positions


def _build_plan_for_hand(
    mujoco_config: Mapping,
    enabled: bool,
    hand: str,
    package_dir,
    robot_name_r,
    robot_name_l,
    urdf_paths: Optional[Mapping[str, Path]],
    module_available: Callable[[str], bool],
) -> MujocoDisplayPlan:
    warnings = []

    if not enabled:
        return MujocoDisplayPlan(False, False, hand, None, 30, ())

    if hand not in {"right", "left"}:
        warnings.append(
            f"MuJoCo display hand must be 'right' or 'left', got '{hand}'."
        )
        hand = "right"

    fps = int(mujoco_config.get("fps", 30) or 30)
    if fps <= 0:
        warnings.append(f"MuJoCo display fps must be positive, got {fps}; using 30.")
        fps = 30
    model_scale = _as_float(mujoco_config.get("model_scale", mujoco_config.get("scale", 1.0)), 1.0)
    if model_scale <= 0:
        warnings.append(f"MuJoCo display model_scale must be positive, got {model_scale}; using 1.0.")
        model_scale = 1.0
    model_rotate_rpy = _normalize_rpy(
        mujoco_config.get("model_rotate_rpy", mujoco_config.get("rotate_rpy"))
    )
    model_translate_xyz = _normalize_xyz(
        mujoco_config.get("model_translate_xyz", mujoco_config.get("translate_xyz"))
    )

    model_path = _mapped_urdf_path(urdf_paths, hand)
    if model_path is None:
        model_path = _derive_urdf_path(Path(package_dir), hand, robot_name_r, robot_name_l)

    if not module_available("mujoco"):
        warnings.append("MuJoCo display is enabled but Python module 'mujoco' is not installed.")
    if not module_available("mujoco.viewer"):
        warnings.append(
            "MuJoCo display is enabled but Python module 'mujoco.viewer' is not available."
        )
    if not model_path.exists():
        warnings.append(f"MuJoCo display model file does not exist: {model_path}")

    should_start = not warnings
    return MujocoDisplayPlan(
        should_start,
        True,
        hand,
        model_path,
        fps,
        tuple(warnings),
        model_scale=model_scale,
        model_rotate_rpy=model_rotate_rpy,
        model_translate_xyz=model_translate_xyz,
    )


def build_mujoco_display_plans(
    baseconfig: Mapping,
    package_dir,
    robot_name_r,
    robot_name_l,
    loaded_hands: Optional[Iterable[str]] = None,
    urdf_paths: Optional[Mapping[str, Path]] = None,
    module_available: Callable[[str], bool] = _module_available,
) -> Tuple[MujocoDisplayPlan, ...]:
    mujoco_config = baseconfig.get("mujoco", {}) if isinstance(baseconfig, Mapping) else {}
    enabled = _as_bool(mujoco_config.get("enabled", False))

    if not enabled:
        return (
            MujocoDisplayPlan(False, False, "right", None, 30, ()),
        )

    return tuple(
        _build_plan_for_hand(
            mujoco_config,
            enabled,
            hand,
            package_dir,
            robot_name_r,
            robot_name_l,
            urdf_paths,
            module_available,
        )
        for hand in _normalize_hands(baseconfig, mujoco_config, loaded_hands)
    )


def build_mujoco_display_plan(
    baseconfig: Mapping,
    package_dir,
    robot_name_r,
    robot_name_l,
    loaded_hands: Optional[Iterable[str]] = None,
    urdf_paths: Optional[Mapping[str, Path]] = None,
    module_available: Callable[[str], bool] = _module_available,
) -> MujocoDisplayPlan:
    plans = build_mujoco_display_plans(
        baseconfig,
        package_dir,
        robot_name_r,
        robot_name_l,
        loaded_hands=loaded_hands,
        urdf_paths=urdf_paths,
        module_available=module_available,
    )
    if plans:
        return plans[0]

    mujoco_config = baseconfig.get("mujoco", {}) if isinstance(baseconfig, Mapping) else {}
    enabled = _as_bool(mujoco_config.get("enabled", False))
    fps = int(mujoco_config.get("fps", 30) or 30)
    return MujocoDisplayPlan(False, enabled, "right", None, fps, ())


class MujocoDisplay:
    def __init__(
        self,
        model_path: Path,
        fps: int = 30,
        hand: str = "right",
        model_scale: float = 1.0,
        model_rotate_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        model_translate_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        mujoco_module=None,
    ):
        self.model_path = Path(model_path)
        self.fps = fps
        self.hand = hand
        self.model_scale = model_scale
        self.model_rotate_rpy = _normalize_rpy(model_rotate_rpy)
        self.model_translate_xyz = _normalize_xyz(model_translate_xyz)
        self.movable_joint_names = get_urdf_movable_joint_names(self.model_path)
        self.mimic_joint_rules = get_urdf_mimic_joint_rules(self.model_path)
        self._mujoco = mujoco_module
        self.model = None
        self.data = None
        self.viewer = None

    def start(self):
        if self._mujoco is None:
            import mujoco
            self._mujoco = mujoco
        import mujoco.viewer

        assets = collect_mujoco_urdf_assets(self.model_path)
        model_xml = prepare_mujoco_model_xml(
            self.model_path,
            model_scale=self.model_scale,
            model_rotate_rpy=self.model_rotate_rpy,
            model_translate_xyz=self.model_translate_xyz,
        )
        if model_xml is None:
            self.model = self._mujoco.MjModel.from_xml_path(str(self.model_path), assets=assets)
        else:
            self.model = self._mujoco.MjModel.from_xml_string(model_xml, assets=assets)
        self.data = self._mujoco.MjData(self.model)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        return self

    def update_joint_positions(self, joint_positions: Mapping[str, float]) -> bool:
        if self.model is None or self.data is None or self.viewer is None:
            return False
        if hasattr(self.viewer, "is_running") and not self.viewer.is_running():
            return False

        lock = self.viewer.lock() if hasattr(self.viewer, "lock") else nullcontext()
        updated = False
        joint_positions = dict(joint_positions)
        for joint_name, (source_joint, multiplier, offset) in self.mimic_joint_rules.items():
            if source_joint in joint_positions:
                joint_positions[joint_name] = joint_positions[source_joint] * multiplier + offset
        with lock:
            for joint_name, value in joint_positions.items():
                joint_id = self._mujoco.mj_name2id(
                    self.model,
                    self._mujoco.mjtObj.mjOBJ_JOINT,
                    joint_name,
                )
                if joint_id < 0:
                    continue
                qpos_addr = int(self.model.jnt_qposadr[joint_id])
                self.data.qpos[qpos_addr] = float(value)
                updated = True
            if updated:
                self._mujoco.mj_forward(self.model, self.data)
                self.viewer.sync()
        return updated

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None


class MujocoDisplayProcess:
    def __init__(
        self,
        model_path: Path,
        fps: int = 30,
        hand: str = "right",
        model_scale: float = 1.0,
        model_rotate_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        model_translate_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        popen_factory=None,
    ):
        self.model_path = Path(model_path)
        self.fps = fps
        self.hand = hand
        self.model_scale = model_scale
        self.model_rotate_rpy = _normalize_rpy(model_rotate_rpy)
        self.model_translate_xyz = _normalize_xyz(model_translate_xyz)
        self.movable_joint_names = get_urdf_movable_joint_names(self.model_path)
        self.mimic_joint_rules = get_urdf_mimic_joint_rules(self.model_path)
        self._popen_factory = popen_factory or subprocess.Popen
        self.process = None

    def start(self):
        config = {
            "model_path": str(self.model_path),
            "fps": int(self.fps),
            "hand": self.hand,
            "model_scale": float(self.model_scale),
            "model_rotate_rpy": self.model_rotate_rpy,
            "model_translate_xyz": self.model_translate_xyz,
        }
        self.process = self._popen_factory(
            [
                sys.executable,
                "-m",
                "linkerhand_retarget.mujoco_display_worker",
                json.dumps(config),
            ],
            stdin=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        return self

    def update_joint_positions(self, joint_positions: Mapping[str, float]) -> bool:
        if self.process is None or self.process.stdin is None:
            return False
        if self.process.poll() is not None:
            return False

        try:
            self.process.stdin.write(
                json.dumps(
                    {
                        "command": "update",
                        "positions": {
                            name: float(value)
                            for name, value in joint_positions.items()
                        },
                    }
                )
                + "\n"
            )
            self.process.stdin.flush()
        except (BrokenPipeError, OSError):
            return False
        return True

    def close(self):
        if self.process is None:
            return

        if self.process.stdin is not None:
            try:
                self.process.stdin.write(json.dumps({"command": "close"}) + "\n")
                self.process.stdin.flush()
                self.process.stdin.close()
            except (BrokenPipeError, OSError):
                pass

        try:
            self.process.wait(timeout=2.0)
        except (OSError, subprocess.TimeoutExpired):
            try:
                self.process.terminate()
                self.process.wait(timeout=1.0)
            except (OSError, subprocess.TimeoutExpired):
                try:
                    self.process.kill()
                except OSError:
                    pass
        self.process = None
