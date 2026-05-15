import pytest
from linkerhand_retarget.linkerhand.linkermcgcore import HaoCunData, HaoCunScoketUdp, HandData, NODES_HAND, LOG_FILE_PATH


class TestHaoCunData:
    def test_initialization(self):
        data = HaoCunData()
        assert data.is_update == False
        assert data.frame_index == 0
        assert data.frequency == 0

    def test_jointangle_arrays_length(self):
        data = HaoCunData()
        assert len(data.jointangle_rHand) == NODES_HAND
        assert len(data.jointangle_lHand) == NODES_HAND


class TestHaoCunScoketUdp:
    def test_initialization_default(self):
        udp = HaoCunScoketUdp()
        assert udp.socket_udp is None
        assert udp.isconnect == False

    def test_initialization_custom_params(self):
        udp = HaoCunScoketUdp(host='192.168.1.1', port=8000, buffer_size=4096)
        assert udp.udp_thread is None
        assert udp.udp_running == False

    def test_is_use_face_blendshapes_default_false(self):
        udp = HaoCunScoketUdp()
        assert udp.is_use_face_blend_shapes_arkit == False


class TestHandData:
    def test_hand_data_creation(self):
        data = HandData(
            pitch=[0]*5,
            side=[0]*5,
            roll=[0]*5,
            two_pitch=[0]*5,
            end_pitch=[0]*5
        )
        assert len(data.pitch) == 5
        assert len(data.side) == 5


class TestConstants:
    def test_nodes_hand_value(self):
        assert NODES_HAND == 25

    def test_log_file_path(self):
        assert LOG_FILE_PATH == "/tmp/a.log"
