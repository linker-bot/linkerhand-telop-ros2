"""
LinkerForce L30 手型映射模块 - ROS2版本
占位实现：复用 O30 的标定 mapper，按 L30 固件输出顺序生成 18 位 motor 表。
"""
import xml.etree.ElementTree as ET
from pathlib import Path

from .linkerforce_o30 import (
    LeftHand as _O30LeftHand,
    RightHand as _O30RightHand,
)

L30_TOPIC_JOINT_NAMES = (
    "拇指指根",
    "拇指指尖",
    "拇指侧摆",
    "拇指旋转",
    "无名指侧摆",
    "无名指指尖",
    "无名指指根",
    "中指指根",
    "中指指尖",
    "小指指根",
    "小指指尖",
    "小指侧摆",
    "中指侧摆",
    "食指侧摆",
    "食指指根",
    "食指指尖",
    "手腕",
    "未使用17",
)
L30_MOTOR_QPOS_INDICES = (
    0, 19, 1, 18,
    5, 14, 4,
    12, 10,
    17, 6, 16,
    13, 9,
    8, 2,
    None, None,
)
L30_MOTOR_ARC_INDICES = (
    2, 3, 1, 0,
    12, 14, 13,
    9, 10,
    17, 18, 16,
    8, 4,
    5, 6,
    None, None,
)
L30_MOTOR_URDF_INDICES = (
    19, 20, 17, 18,
    5, 7, 6,
    10, 11,
    2, 3, 1,
    9, 13,
    14, 15,
    0, None,
)
L30_MOTOR_TARGET_LIMITS = (
    (0, 1000), (0, 1500), (0, 1000), (0, 900),
    (-200, 200), (0, 1500), (0, 1600),
    (0, 1600), (0, 1500),
    (0, 1600), (0, 1500), (-200, 200),
    (-200, 200), (-200, 200),
    (0, 1600), (0, 1500),
    (-1000, 1000), None,
)
L30_MOTOR_DEFAULTS = tuple(0 for _ in L30_MOTOR_TARGET_LIMITS)
L30_MUJOCO_JOINT_ARC_INDICES = (
    None,
    16, 17, 18, 18,
    12, 13, 14, 14,
    8, 9, 10, 10,
    4, 5, 6, 6,
    1, 0, 2, 3,
)
L30_MUJOCO_JOINT_ARC_SIGNS = (1.0,) * len(L30_MUJOCO_JOINT_ARC_INDICES)


def _clamp(value, lower, upper):
    return min(upper, max(lower, value))


def _scale_value(value, source_min, source_max, target_min, target_max):
    if abs(source_max - source_min) < 1e-9:
        return float(target_max)
    ratio = (value - source_min) / (source_max - source_min)
    ratio = max(0.0, min(1.0, ratio))
    return float(target_min + ratio * (target_max - target_min))


def _load_urdf_joint_limits(hand: str):
    package_dir = Path(__file__).resolve().parents[3]
    urdf_path = (
        package_dir
        / "assets"
        / "robots"
        / "hands"
        / "linker_hand"
        / f"l30_{hand}"
        / f"linkerhand_l30_{hand}.urdf"
    )
    root = ET.parse(urdf_path).getroot()
    limits = []
    for joint in root.findall("joint"):
        if joint.attrib.get("type") == "fixed":
            continue
        limit = joint.find("limit")
        if limit is None:
            limits.append((float("-inf"), float("inf")))
            continue
        limits.append((float(limit.attrib["lower"]), float(limit.attrib["upper"])))
    return tuple(limits)


L30_URDF_JOINT_LIMITS_RIGHT = _load_urdf_joint_limits("right")
L30_URDF_JOINT_LIMITS_LEFT = _load_urdf_joint_limits("left")


def _map_l30_values_to_motor(source_values, source_indices, hand: str):
    joint_limits = L30_URDF_JOINT_LIMITS_LEFT if hand == "left" else L30_URDF_JOINT_LIMITS_RIGHT
    jointpositions = list(L30_MOTOR_DEFAULTS)
    for index, (source_idx, urdf_idx, target_limits) in enumerate(
        zip(source_indices, L30_MOTOR_URDF_INDICES, L30_MOTOR_TARGET_LIMITS)
    ):
        if source_idx is None or urdf_idx is None or target_limits is None:
            continue
        if source_idx >= len(source_values) or urdf_idx >= len(joint_limits):
            continue
        lower, upper = joint_limits[urdf_idx]
        value = _clamp(source_values[source_idx], lower, upper)
        target_min, target_max = target_limits
        jointpositions[index] = int(round(_scale_value(value, lower, upper, target_min, target_max)))
    return jointpositions


def _map_l30_qpos_to_motor(qpos, hand: str):
    return _map_l30_values_to_motor(qpos, L30_MOTOR_QPOS_INDICES, hand)


def _map_l30_arc_to_motor(arc_values, hand: str):
    return _map_l30_values_to_motor(arc_values, L30_MOTOR_ARC_INDICES, hand)


class _L30PlaceholderHandMixin:
    def _configure_l30_placeholder(self):
        output_length = len(L30_MOTOR_TARGET_LIMITS)
        self.g_jointpositions = list(L30_MOTOR_DEFAULTS)
        self.topic_joint_names = L30_TOPIC_JOINT_NAMES
        self.g_jointvelocity = [255] * output_length
        self.last_jointpositions = list(self.g_jointpositions)
        self.last_jointvelocity = [255] * output_length
        self.handstate = [0] * output_length
        self.smooth_positions = [float(value) for value in self.g_jointpositions]
        self.motor_constraints = [{"enabled": False} for _ in range(output_length)]
        self.mujoco_joint_arc_indices = L30_MUJOCO_JOINT_ARC_INDICES
        self.mujoco_joint_arc_signs = L30_MUJOCO_JOINT_ARC_SIGNS
        self.mujoco_joint_arc_remaps = (None,) * len(L30_MUJOCO_JOINT_ARC_INDICES)
        self.mujoco_joint_arc_mirrors = (None,) * len(L30_MUJOCO_JOINT_ARC_INDICES)

    def _apply_motor_constraints(self):
        for i, constraint in enumerate(self.motor_constraints[: len(self.g_jointpositions)]):
            if constraint.get("enabled", False):
                min_val = constraint.get("min", 0)
                max_val = constraint.get("max", 255)
                self.g_jointpositions[i] = int(max(min_val, min(max_val, self.g_jointpositions[i])))

    def _set_g_jointpositions_from_qpos(self, qpos):
        self.g_jointpositions = _map_l30_qpos_to_motor(qpos, self.hand_side)
        return self.g_jointpositions

    def _set_g_jointpositions_from_arc(self):
        self.g_jointpositions = _map_l30_arc_to_motor(self.g_jointpositions_arc, self.hand_side)
        return self.g_jointpositions


class RightHand(_L30PlaceholderHandMixin, _O30RightHand):
    def __init__(self, handcore, length=18, is_debug: bool = False):
        _ = length
        super().__init__(handcore, length=20, is_debug=is_debug)
        self._configure_l30_placeholder()


class LeftHand(_L30PlaceholderHandMixin, _O30LeftHand):
    def __init__(self, handcore, length=18, is_debug: bool = False):
        _ = length
        super().__init__(handcore, length=20, is_debug=is_debug)
        self._configure_l30_placeholder()
