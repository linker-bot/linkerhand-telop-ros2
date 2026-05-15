import pytest
import numpy as np
from unittest.mock import Mock, MagicMock
from linkerhand_retarget.linkerhand.handcore import HandCore, KalmanFilter, MultiTargetKalman


class MockJoint:
    def __init__(self, joint_type="revolute", lower=-1.0, upper=1.0):
        self.type = joint_type
        self.limit = MagicMock()
        self.limit.lower = lower
        self.limit.upper = upper


class MockRobot:
    def __init__(self, joints):
        self.joint_map = joints


class TestHandCoreGetJointLimits:
    def test_revolute_joints(self):
        joints = {
            'joint1': MockJoint("revolute", -1.57, 1.57),
            'joint2': MockJoint("revolute", -0.5, 0.5),
        }
        robot = MockRobot(joints)
        lower, upper, ranges = HandCore.get_joint_limits(robot)
        
        assert len(lower) == 2

    def test_prismatic_joint(self):
        joints = {
            'prismatic_joint': MockJoint("prismatic", -0.5, 0.5),
        }
        robot = MockRobot(joints)
        lower, upper, ranges = HandCore.get_joint_limits(robot)
        
        assert lower[0] == -0.5
        assert upper[0] == 0.5
        assert ranges[0] == 1.0

    def test_fixed_joint_skipped(self):
        joints = {
            'fixed_joint': MockJoint("fixed", -1.0, 1.0),
            'revolute_joint': MockJoint("revolute", -1.0, 1.0),
        }
        robot = MockRobot(joints)
        lower, upper, ranges = HandCore.get_joint_limits(robot)
        
        assert len(lower) == 1

    def test_joint_without_limit(self):
        joints = {
            'revolute_no_limit': Mock(),
        }
        joints['revolute_no_limit'].type = "revolute"
        joints['revolute_no_limit'].limit = None
        
        robot = MockRobot(joints)
        lower, upper, ranges = HandCore.get_joint_limits(robot)
        
        assert lower[0] == -3.1415926535
        assert upper[0] == 3.1415926535


class TestHandCoreProjectionProcess:
    def test_projection_process_returns_30_values(self):
        hand_position = np.random.rand(25, 3)
        result = HandCore.projection_process(hand_position)
        
        assert len(result) == 30

    def test_projection_process_returns_list(self):
        hand_position = np.ones((25, 3)) * 0.1
        result = HandCore.projection_process(hand_position)
        
        assert len(result) == 30
        assert isinstance(result[0], float)


class TestKalmanFilter:
    def test_initialization(self):
        kf = KalmanFilter(process_variance=0.01, measurement_variance=0.1, estimated_error=1.0, initial_value=0.0)
        assert kf.process_variance == 0.01
        assert kf.measurement_variance == 0.1
        assert kf.estimated_error == 1.0
        assert kf.current_estimate == 0.0

    def test_update_first_measurement(self):
        kf = KalmanFilter(process_variance=0.01, measurement_variance=0.1, estimated_error=1.0, initial_value=0.0)
        result = kf.update(10.0)
        
        assert 0.0 < result < 10.0

    def test_update_convergence(self):
        kf = KalmanFilter(process_variance=0.001, measurement_variance=0.01, estimated_error=1.0, initial_value=0.0)
        
        results = []
        for _ in range(100):
            results.append(kf.update(10.0))
        
        assert abs(results[-1] - 10.0) < 0.5

    def test_update_with_known_measurement(self):
        kf = KalmanFilter(process_variance=0.01, measurement_variance=0.1, estimated_error=1.0, initial_value=5.0)
        result = kf.update(5.0)
        
        assert result == 5.0

    def test_estimated_error_decreases(self):
        kf = KalmanFilter(process_variance=0.01, measurement_variance=0.1, estimated_error=1.0, initial_value=0.0)
        
        initial_error = kf.estimated_error
        kf.update(10.0)
        
        assert kf.estimated_error < initial_error


class TestMultiTargetKalman:
    def test_initialization(self):
        mtkf = MultiTargetKalman(num_targets=5)
        
        assert mtkf.num_targets == 5
        assert len(mtkf.kalman_filters) == 5
        assert len(mtkf.smoothed_data) == 5

    def test_initialization_custom_params(self):
        mtkf = MultiTargetKalman(
            num_targets=3,
            process_variance=0.001,
            measurement_variance=0.01,
            estimated_error=0.5,
            initial_value=100.0
        )
        
        assert mtkf.num_targets == 3
        assert len(mtkf.kalman_filters) == 3

    def test_update_single_target(self):
        mtkf = MultiTargetKalman(num_targets=5)
        
        result = mtkf.update(10.0, index=2)
        
        assert isinstance(result, float)

    def test_update_all_targets(self):
        mtkf = MultiTargetKalman(num_targets=3)
        
        for i in range(3):
            result = mtkf.update(float(i * 10), index=i)
            assert isinstance(result, float)

    def test_smoothed_data_initialized(self):
        mtkf = MultiTargetKalman(num_targets=2)
        
        assert len(mtkf.smoothed_data) == 2
        assert isinstance(mtkf.smoothed_data, list)
