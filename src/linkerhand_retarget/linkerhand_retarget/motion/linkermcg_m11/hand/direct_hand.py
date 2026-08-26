"""LinkerMCG M11 hand helpers."""

import math
from typing import Iterable

from linkerhand_retarget.motion.linkermcg_m_common.hand.direct_hand import (
    DirectHand as _BaseDirectHand,
)

_M11_ROBOT_DOF_BY_NAME = {
    "o6": 6,
    "l6": 6,
    "l10": 10,
    "l10v7": 10,
    "l20lite": 10,
    "l20": 20,
    "o20": 16,
    "o30": 20,
    "g20": 20,
    "l25": 20,
}


def expected_dof_for_robot(robot_name) -> int:
    robot_key = getattr(robot_name, "name", str(robot_name))
    try:
        return _M11_ROBOT_DOF_BY_NAME[robot_key]
    except KeyError as exc:
        raise ValueError(f"LinkerMCG M11 暂不支持机械手型号: {robot_key}") from exc


def _coerce_finite_float(value) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(number):
        return 0.0
    return number


class DirectHand(_BaseDirectHand):
    """M11 values are direct hand commands; O20 16路 uses signed degrees."""

    def joint_update(self, joint_arc: Iterable[float]):
        if self.length != 16:
            super().joint_update(joint_arc)
            return

        values = list(joint_arc)[: self.length]
        if len(values) < self.length:
            values.extend([0.0] * (self.length - len(values)))
        self.g_jointpositions = [_coerce_finite_float(value) for value in values]


class RightHand(DirectHand):
    pass


class LeftHand(DirectHand):
    pass

__all__ = ["DirectHand", "LeftHand", "RightHand", "expected_dof_for_robot"]
