import pytest
import array
from linkerhand_retarget.linkerhand.linkerforce import CircularBuffer, FrameParser, FrameParseState, BUFFER_SIZE, FRAME_HEADER


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
