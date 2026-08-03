import pytest
import array
import math
import struct
import threading
from pathlib import Path
from linkerhand_retarget.linkerhand.constants import HandType
from linkerhand_retarget.linkerhand.linkerforce import (
    CircularBuffer,
    CommandCode,
    ForceSerialReader,
    FrameHandler,
    FrameParser,
    FrameParseState,
    SerialConnection,
    BUFFER_SIZE,
    FRAME_HEADER,
    LINKERFORCE_ABNORMAL_LOG_FILE_PATH,
    PINKY_JUMP_LOG_FILE_PATH,
    VERSION_QUERY_RETRY_COUNT,
)


class TestCircularBuffer:
    def test_initialization(self):
        buf = CircularBuffer()
        assert buf.data_len == 0
        assert buf.read_pos == 0
        assert buf.write_pos == 0

    def test_write_single_byte(self):
        buf = CircularBuffer()
        buf.write([0x5D])
        assert buf.data_len == 1
        assert buf.read_pos == 0
        assert buf.write_pos == 1

    def test_read_byte(self):
        buf = CircularBuffer()
        buf.write([0x5D, 0x01])
        byte = buf.read_byte()
        assert byte == 0x5D
        assert buf.data_len == 1

    def test_read_empty_buffer(self):
        buf = CircularBuffer()
        byte = buf.read_byte()
        assert byte is None

    def test_write_multiple_bytes(self):
        buf = CircularBuffer()
        data = [0x01, 0x02, 0x03, 0x04, 0x05]
        buf.write(data)
        assert buf.data_len == 5

    def test_read_write_sequence(self):
        buf = CircularBuffer()
        buf.write([10, 20, 30])
        assert buf.read_byte() == 10
        assert buf.read_byte() == 20
        assert buf.read_byte() == 30
        assert buf.read_byte() is None

    def test_buffer_wrap_around(self):
        buf = CircularBuffer()
        for i in range(BUFFER_SIZE + 10):
            buf.write([i % 256])
        assert buf.data_len == BUFFER_SIZE


class TestFrameParser:
    def test_initialization(self):
        parser = FrameParser()
        assert parser.state == FrameParseState.HEADER
        assert parser.expected_len == 0
        assert parser.current_pos == 0

    def test_reset(self):
        parser = FrameParser()
        parser.state = FrameParseState.DATA
        parser.current_pos = 5
        parser.reset()
        assert parser.state == FrameParseState.HEADER
        assert parser.current_pos == 0

    def test_process_byte_finds_header(self):
        parser = FrameParser()
        result = parser.process_byte(FRAME_HEADER)
        assert parser.state == FrameParseState.CMD

    def test_process_byte_accumulates_data(self):
        parser = FrameParser()
        parser.process_byte(FRAME_HEADER)
        parser.process_byte(0x01)
        parser.process_byte(0x03)
        for i in range(3):
            parser.process_byte(i)
        parser.process_byte(0)

    def test_state_transitions(self):
        parser = FrameParser()
        assert parser.state == FrameParseState.HEADER
        
        parser.process_byte(FRAME_HEADER)
        assert parser.state == FrameParseState.CMD
        
        parser.process_byte(0x01)
        assert parser.state == FrameParseState.LENGTH
        
        parser.process_byte(0x02)
        assert parser.state == FrameParseState.DATA

    def test_process_multiple_frames(self):
        parser = FrameParser()
        frame1 = [FRAME_HEADER, 0x01, 0x02, 0xAA, 0xBB]
        for byte in frame1:
            parser.process_byte(byte)
        
        parser.reset()
        frame2 = [FRAME_HEADER, 0x02, 0x01, 0xCC]
        for byte in frame2:
            parser.process_byte(byte)

    def test_invalid_command_candidate_resyncs_to_next_valid_frame_header(self):
        parser = FrameParser()
        valid_frame = FrameHandler.pack_data(
            CommandCode.POSITION_QUERY.value,
            struct.pack("<21f", *range(21)),
        )
        stream = bytes([FRAME_HEADER, 0x66, 0x43, 0x26, 0x83, 0x71, 0x43]) + valid_frame

        parsed_commands = []
        for byte in stream:
            if parser.process_byte(byte):
                parsed_commands.append(parser.frame_buf[1])
                parser.reset()

        assert parsed_commands == [CommandCode.POSITION_QUERY.value]


class TestFrameHandlerPositionFrames:
    @staticmethod
    def _position_payload(degrees):
        return array.array("B", struct.pack("<21f", *degrees))

    def test_position_frame_rejects_non_21_channel_payload_without_overwriting_poslist(self):
        handler = FrameHandler(HandType.right)
        previous = [42.0] * 21
        handler.poslist = previous
        payload = array.array("B", struct.pack("<20f", *range(20)))

        result = handler._handle_position(payload)

        assert result is None
        assert handler.poslist == previous

    def test_position_frame_rejects_non_finite_values_without_overwriting_poslist(self):
        handler = FrameHandler(HandType.right)
        previous = [42.0] * 21
        handler.poslist = previous
        values = [0.0] * 20 + [math.nan]
        payload = array.array("B", struct.pack("<21f", *values))

        result = handler._handle_position(payload)

        assert result is None
        assert handler.poslist == previous

    def test_a6_position_frame_rejects_non_21_channel_payload_without_overwriting_poslist(self):
        handler = FrameHandler(HandType.right)
        previous = [42.0] * 21
        handler.poslist = previous
        payload = array.array("B", struct.pack("<5h", *range(5)))

        result = handler._handle_a6_position(payload)

        assert result is None
        assert handler.poslist == previous

    def test_a6_position_frame_accepts_21_channel_payload(self):
        handler = FrameHandler(HandType.right)
        values = list(range(21))
        payload = array.array("B", struct.pack("<21h", *values))

        result = handler._handle_a6_position(payload)

        assert result == {"poslist": handler.poslist, "force_response": True}
        assert len(handler.poslist) == 21

    def test_right_pinky_end_jump_does_not_log_raw_radian_threshold_noise(self, monkeypatch, tmp_path):
        messages = []
        log_file = tmp_path / "linkerforce_abnormal.log"
        monkeypatch.setattr(
            "linkerhand_retarget.linkerhand.linkerforce.LINKERFORCE_ABNORMAL_LOG_FILE_PATH",
            log_file,
        )

        class FakeLogger:
            def log(self, level, message):
                messages.append((level, message))

        handler = FrameHandler(HandType.right, logger=FakeLogger())
        first = [0.0] * 21
        second = first.copy()
        second[20] = 30.0

        assert handler._handle_position(self._position_payload(first)) is not None
        assert messages == []

        assert handler._handle_position(self._position_payload(second)) is not None

        assert messages == []
        assert not log_file.exists()

    def test_default_abnormal_log_path_is_under_linkerforce_tmp(self):
        assert isinstance(LINKERFORCE_ABNORMAL_LOG_FILE_PATH, Path)
        assert LINKERFORCE_ABNORMAL_LOG_FILE_PATH.name == "linkerforce_abnormal.log"
        assert LINKERFORCE_ABNORMAL_LOG_FILE_PATH.parent.name == "tmp"
        assert PINKY_JUMP_LOG_FILE_PATH == LINKERFORCE_ABNORMAL_LOG_FILE_PATH

    def test_left_pinky_end_jump_does_not_log_right_hand_trace(self):
        messages = []

        class FakeLogger:
            def log(self, level, message):
                messages.append((level, message))

        handler = FrameHandler(HandType.left, logger=FakeLogger())
        first = [0.0] * 21
        second = first.copy()
        second[20] = 30.0

        handler._handle_position(self._position_payload(first))
        handler._handle_position(self._position_payload(second))

        assert messages == []

    def test_unknown_command_logs_raw_frame_context(self, monkeypatch, tmp_path):
        messages = []
        log_file = tmp_path / "linkerforce_abnormal.log"
        monkeypatch.setattr(
            "linkerhand_retarget.linkerhand.linkerforce.LINKERFORCE_ABNORMAL_LOG_FILE_PATH",
            log_file,
        )

        class FakeLogger:
            def log(self, level, message):
                messages.append((level, message))

        handler = FrameHandler(HandType.right, logger=FakeLogger())
        frame = array.array("B", FrameHandler.pack_data(0x66, b"\x01\x02"))

        result = handler.handle_frame(frame)

        assert result is None
        assert messages == [
            (
                "warn",
                "Unknown command: 0x66, len=2, checksum=0xC8, frame=5D 66 02 01 02 C8",
            )
        ]
        log_text = log_file.read_text(encoding="utf-8")
        assert "[LinkerForce异常帧]" in log_text
        assert "Unknown command: 0x66" in log_text
        assert "frame=5D 66 02 01 02 C8" in log_text


class TestConstants:
    def test_buffer_size(self):
        assert BUFFER_SIZE == 1024

    def test_frame_header(self):
        assert FRAME_HEADER == 0x5D

    def test_frame_parse_states(self):
        assert FrameParseState.HEADER.value == 0
        assert FrameParseState.CMD.value == 1
        assert FrameParseState.LENGTH.value == 2
        assert FrameParseState.DATA.value == 3
        assert FrameParseState.CHECKSUM.value == 4


class TestForceSerialReaderVersionQuery:
    def test_sync_version_query_retries_until_retry_limit_without_thread(self, monkeypatch):
        monkeypatch.setattr(
            "linkerhand_retarget.linkerhand.linkerforce.time.sleep",
            lambda _seconds: None,
        )

        payload = struct.pack("<IB", 10203, 1)
        version_frame = FrameHandler.pack_data(CommandCode.VERSION_QUERY.value, payload)

        class FakeSerialPort:
            def __init__(self):
                self.write_count = 0
                self.buffer = bytearray()
                self.is_open = True

            @property
            def in_waiting(self):
                return len(self.buffer)

            def write(self, _data):
                self.write_count += 1
                if self.write_count == VERSION_QUERY_RETRY_COUNT:
                    self.buffer.extend(version_frame)

            def read(self, size):
                data = bytes(self.buffer[:size])
                del self.buffer[:size]
                return data

            def reset_input_buffer(self):
                self.buffer.clear()

            def reset_output_buffer(self):
                pass

        reader = ForceSerialReader(HandType.right)
        serial_port = FakeSerialPort()
        reader.serial_port = serial_port

        assert reader.query_version_sync(response_wait=0) is True
        assert serial_port.write_count == VERSION_QUERY_RETRY_COUNT
        assert reader.handtype == "Right"
        assert reader.version == "1.2.3"


class TestSerialConnectionClose:
    def test_open_requests_exclusive_serial_access(self, monkeypatch):
        serial_kwargs = {}

        class FakeSerialPort:
            def __init__(self, **kwargs):
                serial_kwargs.update(kwargs)

        monkeypatch.setattr(
            "linkerhand_retarget.linkerhand.linkerforce.serial.Serial",
            lambda **kwargs: FakeSerialPort(**kwargs),
        )

        connection = SerialConnection()

        assert connection.open("/dev/ttyUSB0", 2000000) is True
        assert serial_kwargs["port"] == "/dev/ttyUSB0"
        assert serial_kwargs["baudrate"] == 2000000
        assert serial_kwargs["exclusive"] is True

    def test_close_cancels_pending_read_before_closing_port(self):
        events = []

        class FakeSerialPort:
            is_open = True

            def cancel_read(self):
                events.append("cancel_read")

            def close(self):
                events.append("close")
                self.is_open = False

        connection = SerialConnection()
        connection.serial_port = FakeSerialPort()

        connection.close()

        assert events == ["cancel_read", "close"]
        assert connection.serial_port is None
        assert not connection.running.is_set()

    def test_run_exits_without_logging_error_after_stop(self, monkeypatch):
        monkeypatch.setattr(
            "linkerhand_retarget.linkerhand.linkerforce.time.sleep",
            lambda _seconds: None,
        )

        messages = []

        class FakeLogger:
            def log(self, level, message):
                messages.append((level, message))

        class ClosedPort:
            @property
            def in_waiting(self):
                raise OSError("port closed")

        connection = SerialConnection(logger=FakeLogger(), isdebug=True)
        connection.serial_port = ClosedPort()
        connection.running.set()
        connection.running.clear()

        connection._run(None, None)

        assert messages == []

    def test_run_stops_without_read_error_log_on_bad_file_descriptor(self, monkeypatch):
        monkeypatch.setattr(
            "linkerhand_retarget.linkerhand.linkerforce.time.sleep",
            lambda _seconds: None,
        )

        messages = []

        class FakeLogger:
            def log(self, level, message):
                messages.append((level, message))

        class BadFileDescriptorPort:
            @property
            def in_waiting(self):
                raise OSError(9, "Bad file descriptor")

        connection = SerialConnection(logger=FakeLogger(), isdebug=True)
        connection.serial_port = BadFileDescriptorPort()
        connection.running.set()

        thread = threading.Thread(target=connection._run, args=(None, None), daemon=True)
        thread.start()
        thread.join(timeout=0.2)
        was_alive = thread.is_alive()
        connection.running.clear()
        thread.join(timeout=0.2)

        assert not was_alive
        assert messages == []

    def test_run_writes_query_data_through_configured_writer(self, monkeypatch):
        monkeypatch.setattr(
            "linkerhand_retarget.linkerhand.linkerforce.QUERY_INTERVAL",
            0,
        )
        monkeypatch.setattr(
            "linkerhand_retarget.linkerhand.linkerforce.time.sleep",
            lambda _seconds: None,
        )

        events = []
        connection = SerialConnection()

        class FakeSerialPort:
            @property
            def in_waiting(self):
                return 0

            def write(self, data):
                events.append(("direct", data))
                connection.running.clear()

        def locked_writer(data):
            events.append(("locked", data))
            connection.running.clear()
            return True

        connection.serial_port = FakeSerialPort()
        connection.running.set()
        connection._write_callback = locked_writer

        connection._run(None, lambda: b"query")

        assert events == [("locked", b"query")]


class TestForceSerialReaderWrites:
    def test_query_serial_port_requests_exclusive_serial_access(self, monkeypatch):
        serial_kwargs = {}

        class FakeSerialPort:
            is_open = True

            def __init__(self, port, baudrate, **kwargs):
                serial_kwargs.update({"port": port, "baudrate": baudrate, **kwargs})

            def reset_input_buffer(self):
                pass

            def reset_output_buffer(self):
                pass

            def write(self, _data):
                pass

            @property
            def in_waiting(self):
                return 0

            def close(self):
                self.is_open = False

        monkeypatch.setattr(
            "linkerhand_retarget.linkerhand.linkerforce.serial.Serial",
            lambda port, baudrate, **kwargs: FakeSerialPort(port, baudrate, **kwargs),
        )
        monkeypatch.setattr(
            "linkerhand_retarget.linkerhand.linkerforce.time.sleep",
            lambda _seconds: None,
        )

        reader = ForceSerialReader(HandType.right)
        reader.baudrates = [2000000]

        assert reader.query_serial_port("/dev/ttyUSB0", timeout=0)[0] is False
        assert serial_kwargs["port"] == "/dev/ttyUSB0"
        assert serial_kwargs["baudrate"] == 2000000
        assert serial_kwargs["exclusive"] is True

    def test_force_feedback_write_uses_reader_write_lock(self):
        events = []

        class TrackingLock:
            def __enter__(self):
                events.append("lock_enter")

            def __exit__(self, _exc_type, _exc, _traceback):
                events.append("lock_exit")
                return False

        class FakeSerialPort:
            def write(self, data):
                events.append(("write", data))

        reader = ForceSerialReader(HandType.right)
        reader.serial_port = FakeSerialPort()
        reader._serial_write_lock = TrackingLock()
        reader.pack_04_data = lambda: b"force"

        assert reader.write_force_feedback() is True
        assert events == ["lock_enter", ("write", b"force"), "lock_exit"]

    def test_write_packet_uses_reader_write_lock(self):
        events = []

        class TrackingLock:
            def __enter__(self):
                events.append("lock_enter")

            def __exit__(self, _exc_type, _exc, _traceback):
                events.append("lock_exit")
                return False

        class FakeSerialPort:
            def write(self, data):
                events.append(("write", data))

        reader = ForceSerialReader(HandType.right)
        reader.serial_port = FakeSerialPort()
        reader._serial_write_lock = TrackingLock()

        assert reader.write_packet(b"version") is True
        assert events == ["lock_enter", ("write", b"version"), "lock_exit"]

    def test_sync_version_query_stops_after_first_success(self, monkeypatch):
        monkeypatch.setattr(
            "linkerhand_retarget.linkerhand.linkerforce.time.sleep",
            lambda _seconds: None,
        )

        payload = struct.pack("<IB", 10203, 1)
        version_frame = FrameHandler.pack_data(CommandCode.VERSION_QUERY.value, payload)

        class FakeSerialPort:
            def __init__(self):
                self.write_count = 0
                self.buffer = bytearray(version_frame)
                self.is_open = True

            @property
            def in_waiting(self):
                return len(self.buffer)

            def write(self, _data):
                self.write_count += 1

            def read(self, size):
                data = bytes(self.buffer[:size])
                del self.buffer[:size]
                return data

            def reset_input_buffer(self):
                pass

            def reset_output_buffer(self):
                pass

        reader = ForceSerialReader(HandType.right)
        serial_port = FakeSerialPort()
        reader.serial_port = serial_port

        assert reader.query_version_sync(response_wait=0) is True
        assert serial_port.write_count == 1
        assert reader.handtype == "Right"
        assert reader.version == "1.2.3"
