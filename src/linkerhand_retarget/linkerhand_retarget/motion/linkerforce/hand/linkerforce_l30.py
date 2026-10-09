"""LinkerForce L30 mapping and output adaptation.

The L30 output protocol is different from O30.  The calibrated mapper
algorithm is shared with O30, but all L30 mapping, pose, and constraint data
is defined here so that L30 does not inherit O30 tuning by accident.
"""
import copy
import xml.etree.ElementTree as ET
from pathlib import Path

from linkerhand.calibration_checklist import normalize_calibration_filter_config

from ..config.lffg1.l30_config import (
    CALIBRATION_FILTER_CONFIG,
    FINGER_CONFIGS,
    L30_MOTOR_ARC_INDICES,
    L30_MOTOR_DEFAULTS,
    L30_MOTOR_QPOS_INDICES,
    L30_MOTOR_TARGET_LIMITS,
    L30_MOTOR_URDF_INDICES,
    L30_MUJOCO_JOINT_ARC_INDICES,
    L30_MUJOCO_JOINT_ARC_SIGNS,
    L30_ROBOT_FIST_LEFT,
    L30_ROBOT_FIST_RIGHT,
    L30_ROBOT_OPOSE_LEFT,
    L30_ROBOT_OPOSE_RIGHT,
    L30_ROBOT_ORIGINAL_LEFT,
    L30_ROBOT_ORIGINAL_RIGHT,
    L30_TOPIC_JOINT_NAMES,
    MAPPING_ORDER,
    MOTOR_CONSTRAINTS,
    MULTI_SEGMENT_CONFIG_FROZEN,
)
from .linkerforce_o30 import (
    O30StableDynamicWeightMultiStateLinearMapper as _L30StableMapper,
    LeftHand as _O30LeftHand,
    RightHand as _O30RightHand,
)
from .simple_linear_mapper import (
    apply_linkerforce_product_config,
    load_linkerforce_config,
    resolve_linkerforce_finger_configs,
    resolve_linkerforce_motor_constraints,
)



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


class _L30HandMixin:
    @staticmethod
    def _fallback_thumb_yaw(joint_arc):
        return joint_arc[2] * -2.5

    @staticmethod
    def _sync_fallback_arcs(_qpos):
        pass

    def _configure_l30(self, is_debug: bool):
        output_length = len(L30_MOTOR_TARGET_LIMITS)
        self.product_type = None
        self.product_family = "LFFG1"
        self.glove_version = "v2"
        self.product_config_module = load_linkerforce_config(
            "l30", self.product_type, self.glove_version
        )
        self.calibration_filter_config = normalize_calibration_filter_config(
            CALIBRATION_FILTER_CONFIG
        )
        self.g_jointpositions = list(L30_MOTOR_DEFAULTS)
        self.topic_joint_names = L30_TOPIC_JOINT_NAMES
        self.g_jointvelocity = [255] * output_length
        self.last_jointpositions = list(self.g_jointpositions)
        self.last_jointvelocity = [255] * output_length
        self.handstate = [0] * output_length
        self.smooth_positions = [float(value) for value in self.g_jointpositions]
        self.mujoco_joint_arc_indices = L30_MUJOCO_JOINT_ARC_INDICES
        self.mujoco_joint_arc_signs = L30_MUJOCO_JOINT_ARC_SIGNS
        self.mujoco_joint_arc_remaps = (None,) * len(L30_MUJOCO_JOINT_ARC_INDICES)
        self.mujoco_joint_arc_mirrors = (None,) * len(L30_MUJOCO_JOINT_ARC_INDICES)

        if self.hand_side == "left":
            self.robot_original = copy.deepcopy(L30_ROBOT_ORIGINAL_LEFT)
            self.robot_opose = copy.deepcopy(L30_ROBOT_OPOSE_LEFT)
            self.robot_fist = copy.deepcopy(L30_ROBOT_FIST_LEFT)
        else:
            self.robot_original = copy.deepcopy(L30_ROBOT_ORIGINAL_RIGHT)
            self.robot_opose = copy.deepcopy(L30_ROBOT_OPOSE_RIGHT)
            self.robot_fist = copy.deepcopy(L30_ROBOT_FIST_RIGHT)

        self.multi_state_mapper = _L30StableMapper(
            resolve_linkerforce_finger_configs(FINGER_CONFIGS, None, self.glove_version),
            MAPPING_ORDER,
            is_debug=is_debug,
        )
        self.motor_constraints = resolve_linkerforce_motor_constraints(
            MOTOR_CONSTRAINTS,
            self.hand_side,
            self.product_type,
            self.glove_version,
        )

    def set_glove_version(self, version: str):
        if version:
            self.set_product_version(self.product_type, version)

    def set_product_version(self, product_type: str, firmware_version: str):
        apply_linkerforce_product_config(
            self,
            "l30",
            self.hand_side,
            product_type,
            firmware_version,
        )

    def initialize_mapper(self) -> bool:
        self.multi_state_mapper.add_state(
            "original",
            self._to_list(self.calibrationoriginal),
            self.robot_original,
        )
        self.multi_state_mapper.add_state(
            "opose",
            self._to_list(self.calibrationopose),
            self.robot_opose,
        )
        self.multi_state_mapper.add_state(
            "fist",
            self._to_list(self.calibrationfistpose),
            self.robot_fist,
        )
        self.multi_state_mapper.set_state_order(list(MULTI_SEGMENT_CONFIG_FROZEN))
        return True

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


class RightHand(_L30HandMixin, _O30RightHand):
    def __init__(self, handcore, length=18, is_debug: bool = False):
        _ = length
        super().__init__(handcore, length=20, is_debug=is_debug)
        self._configure_l30(is_debug)


class LeftHand(_L30HandMixin, _O30LeftHand):
    def __init__(self, handcore, length=18, is_debug: bool = False):
        _ = length
        super().__init__(handcore, length=20, is_debug=is_debug)
        self._configure_l30(is_debug)
