import math
from typing import Iterable

_ROBOT_DOF_BY_NAME = {
    "o6": 6,
    "l6": 6,
    "l10": 10,
    "l10v7": 10,
    "l20lite": 10,
    "l20": 20,
    "o20": 20,
    "o30": 20,
    "g20": 20,
    "l25": 20,
}


def expected_dof_for_robot(robot_name) -> int:
    robot_key = getattr(robot_name, "name", str(robot_name))
    try:
        return _ROBOT_DOF_BY_NAME[robot_key]
    except KeyError as exc:
        raise ValueError(f"LinkerMCG M 系列暂不支持机械手型号: {robot_key}") from exc


def _clamp_motor_value(value) -> int:
    try:
        value = float(value)
    except (TypeError, ValueError):
        value = 0.0
    if not math.isfinite(value):
        value = 0.0
    return int(round(max(0.0, min(255.0, value))))


class DirectHand:
    """M 系列行程数据已是机械手下发顺序，这里只做长度截断和 0..255 保护。"""

    def __init__(self, handcore=None, length=6):
        self.handcore = handcore
        self.length = int(length)
        self.g_jointpositions = [255] * self.length
        self.g_jointvelocity = [255] * self.length
        self.last_jointpositions = [255] * self.length
        self.last_jointvelocity = [255] * self.length

    def joint_update(self, joint_arc: Iterable[float]):
        values = list(joint_arc)[: self.length]
        if len(values) < self.length:
            values.extend([0] * (self.length - len(values)))
        self.g_jointpositions = [_clamp_motor_value(value) for value in values]

    def speed_update(self):
        self.g_jointvelocity = [255] * self.length
        self.last_jointpositions = list(self.g_jointpositions)
        self.last_jointvelocity = list(self.g_jointvelocity)


class RightHand(DirectHand):
    pass


class LeftHand(DirectHand):
    pass
