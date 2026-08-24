import pytest
from linkerhand_retarget.linkerhand.udexrealcore import (
    UdexRealData, MotionData, Bone, Parameter, DeviceData,
    NODES_HAND, NO_DATA_TIMEOUT
)
from linkerhand_retarget.motion.udexreal.retarget import Retarget


def test_udexreal_retarget_process_returns_true_after_udp_initialization(monkeypatch):
    retarget = Retarget.__new__(Retarget)
    monkeypatch.setattr(retarget, "initialize_udp", lambda: True)

    assert retarget.process() is True


class TestUdexRealData:
    def test_initialization(self):
        data = UdexRealData()
        assert data.is_update == False
        assert data.frame_index == 0
        assert data.frequency == 0

    def test_jointangle_arrays_length(self):
        data = UdexRealData()
        assert len(data.jointangle_rHand) == NODES_HAND
        assert len(data.jointangle_lHand) == NODES_HAND

    def test_jointderict_arrays_length(self):
        data = UdexRealData()
        assert len(data.jointderict_rHand) == NODES_HAND
        assert len(data.jointderict_lHand) == NODES_HAND

    def test_jointderict_default_values(self):
        data = UdexRealData()
        assert all(v == 1 for v in data.jointderict_rHand)
        assert all(v == 1 for v in data.jointderict_lHand)

    def test_timeout_attributes(self):
        data = UdexRealData()
        assert data.last_data_time == 0.0
        assert data.is_data_timeout == False


class TestBone:
    def test_bone_creation(self):
        bone = Bone(
            Name="test_bone",
            Parent=1,
            Location=[0.0, 0.0, 0.0],
            Rotation=[0.0, 0.0, 0.0, 1.0],
            Scale=[1.0, 1.0, 1.0]
        )
        assert bone.Name == "test_bone"
        assert bone.Parent == 1
        assert bone.Location == [0.0, 0.0, 0.0]


class TestParameter:
    def test_parameter_creation(self):
        param = Parameter(Name="test_param", Value=1.0)
        assert param.Name == "test_param"
        assert param.Value == 1.0

    def test_parameter_int_value(self):
        param = Parameter(Name="int_param", Value=10)
        assert param.Value == 10

    def test_parameter_bool_value(self):
        param = Parameter(Name="bool_param", Value=True)
        assert param.Value == True


class TestDeviceData:
    def test_device_data_creation(self):
        bone = Bone(Name="bone1", Parent=0, Location=[0,0,0], Rotation=[0,0,0,1], Scale=[1,1,1])
        param = Parameter(Name="param1", Value=1.0)
        device = DeviceData(Bones=[bone], Parameter=[param])
        
        assert len(device.Bones) == 1
        assert len(device.Parameter) == 1


class TestMotionData:
    def test_initialization_empty(self):
        motion = MotionData({})
        assert motion.devices == {}

    def test_get_device_not_found(self):
        motion = MotionData({})
        result = motion.get_device("nonexistent")
        assert result is None

    def test_list_sequence_params_empty(self):
        motion = MotionData({})
        with pytest.raises(ValueError):
            motion.list_sequence_params("nonexistent", "prefix")


class TestConstants:
    def test_nodes_hand_value(self):
        assert NODES_HAND == 24

    def test_no_data_timeout_value(self):
        assert NO_DATA_TIMEOUT == 1.0
