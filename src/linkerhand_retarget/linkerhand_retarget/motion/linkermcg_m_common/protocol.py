import json
import math
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Any, List, Optional, Sequence


SCHEMA_DOF_MAP = {
    "linker.stroke6.flat.v1": 6,
    "linker.stroke10.flat.v1": 10,
    "linker.stroke20.flat.v1": 20,
    "linker.o20.targetpos16.flat.v1": 16,
    "linker.o30.stroke20.flat.v1": 20,
}

DOF_SCHEMA_MAP = {
    6: "linker.stroke6.flat.v1",
    10: "linker.stroke10.flat.v1",
    20: "linker.stroke20.flat.v1",
}
DOF_HAND_TYPE_MAP = {
    6: "LinkerHand/O6",
    10: "LinkerHand/L10",
    16: "LinkerHand/O20",
    20: "LinkerHand/L20",
}


@dataclass(frozen=True)
class StrokeEnvelope:
    schema_id: str
    hand_type: str
    dof: int
    timestamp_ms: int
    labels: List[str]
    left_hand: List[float]
    right_hand: List[float]


@dataclass
class MotionData:
    is_update: bool = False
    frame_index: int = 0
    frequency: float = 0.0
    timestamp_ms: int = 0
    schema_id: str = ""
    hand_type: str = ""
    dof: int = 0
    labels: List[str] = field(default_factory=list)
    jointangle_rHand: List[float] = field(default_factory=list)
    jointangle_lHand: List[float] = field(default_factory=list)

    def update_from_envelope(self, envelope: StrokeEnvelope, frame_index: int):
        self.is_update = True
        self.frame_index = int(frame_index)
        self.timestamp_ms = int(envelope.timestamp_ms)
        self.schema_id = envelope.schema_id
        self.hand_type = envelope.hand_type
        self.dof = int(envelope.dof)
        self.labels = list(envelope.labels)
        self.jointangle_lHand = list(envelope.left_hand)
        self.jointangle_rHand = list(envelope.right_hand)


def _decode_payload(payload: Any) -> dict:
    if isinstance(payload, dict):
        return payload
    if isinstance(payload, bytes):
        payload = payload.decode("utf-8", errors="replace")
    if isinstance(payload, str):
        return json.loads(payload)
    raise ValueError(f"unsupported LinkerMCG payload type: {type(payload).__name__}")


def _require_str(data: dict, key: str) -> str:
    value = data.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"{key} must be a non-empty string")
    return value


def _require_int(data: dict, key: str) -> int:
    try:
        return int(data[key])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"{key} must be an integer") from exc


def _coerce_float_list(name: str, values: Any, expected_len: int) -> List[float]:
    if not isinstance(values, Sequence) or isinstance(values, (str, bytes, bytearray)):
        raise ValueError(f"{name} must be an array")
    if len(values) != expected_len:
        raise ValueError(f"{name} length must be {expected_len}, got {len(values)}")

    result = []
    for index, value in enumerate(values):
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name}[{index}] must be numeric") from exc
        if not math.isfinite(number):
            raise ValueError(f"{name}[{index}] must be finite")
        result.append(number)
    return result


def _coerce_labels(values: Any, expected_len: int) -> List[str]:
    if not isinstance(values, Sequence) or isinstance(values, (str, bytes, bytearray)):
        raise ValueError("labels must be an array")
    if len(values) != expected_len:
        raise ValueError(f"labels length must be {expected_len}, got {len(values)}")
    return [str(value) for value in values]


def _infer_minimal_stroke_envelope(data: dict) -> Optional[StrokeEnvelope]:
    if "leftHand" not in data or "rightHand" not in data:
        return None
    if any(key in data for key in ("schemaId", "handType", "dof", "timestampMs", "labels")):
        return None

    left_values = data.get("leftHand")
    right_values = data.get("rightHand")
    if (
        not isinstance(left_values, Sequence)
        or isinstance(left_values, (str, bytes, bytearray))
        or not isinstance(right_values, Sequence)
        or isinstance(right_values, (str, bytes, bytearray))
    ):
        return None
    if len(left_values) != len(right_values):
        raise ValueError(f"leftHand length must match rightHand length: {len(left_values)} != {len(right_values)}")

    dof = len(left_values)
    schema_id = DOF_SCHEMA_MAP.get(dof)
    if schema_id is None:
        raise ValueError(f"unsupported minimal LinkerMCG dof length: {dof}")

    return StrokeEnvelope(
        schema_id=schema_id,
        hand_type=DOF_HAND_TYPE_MAP[dof],
        dof=dof,
        timestamp_ms=int(time.time() * 1000),
        labels=[f"joint{index + 1}" for index in range(dof)],
        left_hand=_coerce_float_list("leftHand", left_values, dof),
        right_hand=_coerce_float_list("rightHand", right_values, dof),
    )


def parse_stroke_envelope(payload: Any) -> StrokeEnvelope:
    data = _decode_payload(payload)
    inferred = _infer_minimal_stroke_envelope(data)
    if inferred is not None:
        return inferred

    schema_id = _require_str(data, "schemaId")
    hand_type = _require_str(data, "handType")
    dof = _require_int(data, "dof")
    timestamp_ms = _require_int(data, "timestampMs")

    schema_dof = SCHEMA_DOF_MAP.get(schema_id)
    if schema_dof is None:
        raise ValueError(f"unsupported schemaId: {schema_id}")
    if dof != schema_dof:
        raise ValueError(f"dof must match {schema_id}: expected {schema_dof}, got {dof}")

    return StrokeEnvelope(
        schema_id=schema_id,
        hand_type=hand_type,
        dof=dof,
        timestamp_ms=timestamp_ms,
        labels=_coerce_labels(data.get("labels"), dof),
        left_hand=_coerce_float_list("leftHand", data.get("leftHand"), dof),
        right_hand=_coerce_float_list("rightHand", data.get("rightHand"), dof),
    )


def _is_status_payload(payload: bytes) -> bool:
    try:
        data = _decode_payload(payload)
    except Exception:
        text = payload.decode("utf-8", errors="replace").strip().upper()
        return text in {"CONNECT", "HEARTBEAT", "PING", "DISCONNECT"}
    status = str(data.get("status", "")).strip().lower()
    action = str(data.get("action", "")).strip().lower()
    return bool(status or action)


class LinkerMcgUdpClient:
    motion_label = "M"
    motion_data_class = MotionData
    thread_name = "linkermcg_udp"

    def __init__(
        self,
        host="127.0.0.1",
        port=9011,
        buffer_size=4096,
        logger=None,
        first_json_timeout_sec=2.0,
        no_json_warn_interval_sec=5.0,
    ):
        self.socket_udp: Optional[socket.socket] = None
        self.udp_thread: Optional[threading.Thread] = None
        self.udp_running = False
        self.isconnect = False
        self.target_host = host
        self.target_port = int(port)
        self.target_address = (host, int(port))
        self.buffer_size = int(buffer_size)
        self.realmocapdata = self.motion_data_class()
        self.data_lock = threading.Lock()
        self.frame_counter = 0
        self.logger = logger
        self.last_error = ""
        self._last_frame_time = None
        self._parse_error_count = 0
        self.first_json_timeout_sec = float(first_json_timeout_sec)
        self.no_json_warn_interval_sec = float(no_json_warn_interval_sec)
        self._started_at: Optional[float] = None
        self._last_no_json_warn_at: Optional[float] = None
        self._last_status_packet_at: Optional[float] = None

    def _log_info(self, message: str):
        if self.logger is not None:
            self.logger.info(message)
        else:
            print(message)

    def _log_warn(self, message: str):
        if self.logger is not None:
            self.logger.warn(message)
        else:
            print(message)

    def _record_status_packet(self, now: Optional[float] = None):
        self._last_status_packet_at = time.time() if now is None else float(now)

    def _warn_if_no_json_frame(self, now: Optional[float] = None):
        if not self.udp_running or self.realmocapdata.is_update or self._started_at is None:
            return

        current_time = time.time() if now is None else float(now)
        elapsed = current_time - self._started_at
        if elapsed < self.first_json_timeout_sec:
            return
        if (
            self._last_no_json_warn_at is not None
            and current_time - self._last_no_json_warn_at < self.no_json_warn_interval_sec
        ):
            return

        self._last_no_json_warn_at = current_time
        if self._last_status_packet_at is None:
            detail = (
                "no heartbeat/status packet or "
                f"{self.motion_label} stroke JSON received"
            )
        else:
            status_age = max(0.0, current_time - self._last_status_packet_at)
            detail = (
                "heartbeat/status packets are arriving "
                f"(last {status_age:.1f}s ago), but no "
                f"{self.motion_label} stroke JSON received"
            )
        self._log_warn(
            f"LinkerMCG {self.motion_label} UDP connected but {detail} "
            f"after {elapsed:.1f}s; "
            "check sender protocol and required fields: "
            "schemaId, handType, dof, timestampMs, labels, leftHand, rightHand"
        )

    def udp_initial(self) -> bool:
        try:
            self.socket_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket_udp.settimeout(0.5)
            self.socket_udp.sendto(b"CONNECT", self.target_address)
            self._started_at = time.time()
            self._last_no_json_warn_at = None
            self._last_status_packet_at = None
            self.udp_running = True
            self.isconnect = True
            self.udp_thread = threading.Thread(
                target=self._udp_process,
                name=self.thread_name,
                daemon=True,
            )
            self.udp_thread.start()
            self._log_info(
                f"LinkerMCG {self.motion_label} UDP client started: "
                f"{self.target_host}:{self.target_port}"
            )
            return True
        except OSError as exc:
            self.last_error = str(exc)
            self.isconnect = False
            if self.socket_udp:
                self.socket_udp.close()
                self.socket_udp = None
            self._log_warn(
                f"LinkerMCG {self.motion_label} UDP initialization failed: {exc}"
            )
            return False

    def _udp_process(self):
        while self.udp_running:
            try:
                payload, _ = self.socket_udp.recvfrom(self.buffer_size)
            except socket.timeout:
                self._warn_if_no_json_frame()
                continue
            except OSError as exc:
                if self.udp_running:
                    self.last_error = str(exc)
                    self._log_warn(
                        f"LinkerMCG {self.motion_label} UDP receive failed: {exc}"
                    )
                break

            try:
                envelope = parse_stroke_envelope(payload)
            except Exception as exc:
                if _is_status_payload(payload):
                    self._record_status_packet()
                else:
                    self._parse_error_count += 1
                    if self._parse_error_count == 1 or self._parse_error_count % 100 == 0:
                        self._log_warn(
                            f"LinkerMCG {self.motion_label} UDP packet ignored: {exc}"
                        )
                self._warn_if_no_json_frame()
                continue

            now = time.time()
            with self.data_lock:
                self.frame_counter += 1
                if self._last_frame_time:
                    interval = now - self._last_frame_time
                    if interval > 0:
                        self.realmocapdata.frequency = 1.0 / interval
                self._last_frame_time = now
                self.realmocapdata.update_from_envelope(envelope, self.frame_counter)

    def udp_close(self) -> bool:
        self.udp_running = False
        if self.socket_udp:
            try:
                self.socket_udp.sendto(b"DISCONNECT", self.target_address)
            except OSError:
                pass
            self.socket_udp.close()
            self.socket_udp = None
        if self.udp_thread and self.udp_thread.is_alive():
            self.udp_thread.join(timeout=1.0)
        self.isconnect = False
        return True

    def udp_is_connect(self) -> bool:
        return self.isconnect

    def udp_recv_mocap_data(self, mocap_data: MotionData) -> bool:
        with self.data_lock:
            mocap_data.is_update = self.realmocapdata.is_update
            mocap_data.frame_index = self.realmocapdata.frame_index
            mocap_data.frequency = self.realmocapdata.frequency
            mocap_data.timestamp_ms = self.realmocapdata.timestamp_ms
            mocap_data.schema_id = self.realmocapdata.schema_id
            mocap_data.hand_type = self.realmocapdata.hand_type
            mocap_data.dof = self.realmocapdata.dof
            mocap_data.labels = list(self.realmocapdata.labels)
            mocap_data.jointangle_rHand = list(self.realmocapdata.jointangle_rHand)
            mocap_data.jointangle_lHand = list(self.realmocapdata.jointangle_lHand)
        return True
