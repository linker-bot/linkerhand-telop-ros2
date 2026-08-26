"""LinkerMCG M11 protocol."""

from linkerhand_retarget.motion.linkermcg_m_common.protocol import (
    DOF_HAND_TYPE_MAP,
    DOF_SCHEMA_MAP,
    LinkerMcgUdpClient,
    MotionData as _MotionData,
    SCHEMA_DOF_MAP,
    StrokeEnvelope,
    _coerce_float_list,
    _coerce_labels,
    _decode_payload,
    _infer_minimal_stroke_envelope,
    _is_status_payload,
    _require_int,
    _require_str,
    parse_stroke_envelope,
)


class M11MotionData(_MotionData):
    """M11 motion envelope."""


class LinkerMcgM11UdpClient(LinkerMcgUdpClient):
    motion_label = "M11"
    motion_data_class = M11MotionData
    thread_name = "linkermcg_m11_udp"


__all__ = [
    "DOF_HAND_TYPE_MAP",
    "DOF_SCHEMA_MAP",
    "LinkerMcgM11UdpClient",
    "M11MotionData",
    "SCHEMA_DOF_MAP",
    "StrokeEnvelope",
    "_coerce_float_list",
    "_coerce_labels",
    "_decode_payload",
    "_infer_minimal_stroke_envelope",
    "_is_status_payload",
    "_require_int",
    "_require_str",
    "parse_stroke_envelope",
]
