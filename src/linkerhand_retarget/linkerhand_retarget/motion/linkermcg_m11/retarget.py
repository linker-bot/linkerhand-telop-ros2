"""LinkerMCG M11 retargeting built on the shared M-series core."""

from linkerhand_retarget.motion.linkermcg_m_common.retarget import Retarget as _BaseRetarget

from .hand.direct_hand import LeftHand, RightHand
from .hand.direct_hand import expected_dof_for_robot
from .protocol import LinkerMcgM11UdpClient


class Retarget(_BaseRetarget):
    motion_label = "M11"
    udp_client_class = LinkerMcgM11UdpClient
    right_hand_class = RightHand
    left_hand_class = LeftHand
    expected_dof_func = staticmethod(expected_dof_for_robot)
    _schema_by_robot_name = {
        "o6": {"linker.stroke6.flat.v1"},
        "l6": {"linker.stroke6.flat.v1"},
        "l10": {"linker.stroke10.flat.v1"},
        "l10v7": {"linker.stroke10.flat.v1"},
        "l20lite": {"linker.stroke10.flat.v1"},
        "l20": {"linker.stroke20.flat.v1"},
        "g20": {"linker.stroke20.flat.v1"},
        "l25": {"linker.stroke20.flat.v1"},
        "o20": {"linker.o20.targetpos16.flat.v1"},
        "o30": {"linker.o30.stroke20.flat.v1"},
    }

    def _payload_mismatch_detail(self, robot_name, mocapdata) -> str:
        robot_key = getattr(robot_name, "name", str(robot_name))
        expected_schemas = self._schema_by_robot_name.get(robot_key)
        if not expected_schemas:
            return ""

        schema_id = getattr(mocapdata, "schema_id", "")
        if schema_id not in expected_schemas:
            expected = "/".join(sorted(expected_schemas))
            return f"schemaId={schema_id}, expected={expected}"
        return ""


__all__ = ["Retarget"]
