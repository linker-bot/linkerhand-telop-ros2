import pytest
from linkerhand_retarget.linkerhand.vtrdyncore import MocapData, VtrdynSocketUdp, NODES_BODY, NODES_HAND, NODES_FACEBS_ARKIT, NODES_FACEBS_AUDIO


class TestMocapData:
    def test_initialization(self):
        mocap = MocapData()
        assert mocap.is_update == False
        assert mocap.frame_index == 0
        assert mocap.frequency == 0

    def test_body_arrays_length(self):
        mocap = MocapData()
        assert len(mocap.sensor_state_body) == NODES_BODY
        assert len(mocap.position_body) == NODES_BODY
        assert len(mocap.quaternion_body) == NODES_BODY
        assert len(mocap.gyr_body) == NODES_BODY
        assert len(mocap.acc_body) == NODES_BODY
        assert len(mocap.velocity_body) == NODES_BODY

    def test_hand_arrays_length(self):
        mocap = MocapData()
        assert len(mocap.sensor_state_r_hand) == NODES_HAND
        assert len(mocap.position_rHand) == NODES_HAND
        assert len(mocap.quaternion_rHand) == NODES_HAND

    def test_face_blendshapes_length(self):
        mocap = MocapData()
        assert len(mocap.face_blend_shapes_arkit) == NODES_FACEBS_ARKIT
        assert len(mocap.face_blend_shapes_audio) == NODES_FACEBS_AUDIO

    def test_eyeball_quaternion_length(self):
        mocap = MocapData()
        assert len(mocap.local_quat_right_eyeball) == 4
        assert len(mocap.local_quat_left_eyeball) == 4


class TestVtrdynSocketUdp:
    def test_initialization(self):
        udp = VtrdynSocketUdp()
        assert udp.socket_udp is None
        assert udp.isconnect == False

    def test_initialization_with_debug(self):
        udp = VtrdynSocketUdp(debug=True)
        assert udp.debug == True

    def test_mocap_data_initialized(self):
        udp = VtrdynSocketUdp()
        assert udp.mocap_data_realtime is not None

    def test_data_lock_initialized(self):
        udp = VtrdynSocketUdp()
        assert udp.data_lock is not None

    def test_send_running_default_false(self):
        udp = VtrdynSocketUdp()
        assert udp.send_running == False

    def test_thread_initialized(self):
        udp = VtrdynSocketUdp()
        assert udp.send_thread is None


class TestConstants:
    def test_nodes_body_value(self):
        assert NODES_BODY == 23

    def test_nodes_hand_value(self):
        assert NODES_HAND == 20

    def test_nodes_facebs_arkit_value(self):
        assert NODES_FACEBS_ARKIT == 52

    def test_nodes_facebs_audio_value(self):
        assert NODES_FACEBS_AUDIO == 26
