import pytest
import numpy as np
import tempfile
import os
import yaml
from linkerhand_retarget.linkerhand.utils import (
    DataSource,
    read_yaml,
    extract_dataset_folder_last_two_digits,
    translate_wrist_to_origin,
    apply_pose_matrix,
    inverse_transformation,
    trans_xyzwori_to_wxyzori,
    trans_wxyzori_to_xyzwori,
    scale_value,
    is_within_range,
    extend_line,
    poseture_to_matrix,
    cal_distance,
    change_list,
    quaternion_conjugate,
    quaternion_norm_squared,
    quaternion_inverse,
    quaternion_multiply,
    unitydata_to_worldspacedata,
    get_quaternion_relative,
    get_child_quaternion,
    rotate_matrix_x,
    rotate_matrix_y,
    rotate_matrix_z,
    rotate_quaternion,
    cubic_model,
)


class TestDataSource:
    def test_enum_values(self):
        assert DataSource.motion.value == 1
        assert DataSource.video.value == 2
        assert DataSource.vr.value == 3

    def test_enum_names(self):
        assert DataSource.motion.name == "motion"
        assert DataSource.video.name == "video"
        assert DataSource.vr.name == "vr"


class TestReadYaml:
    def test_read_valid_yaml(self):
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump({'key': 'value', 'number': 42}, f)
            f.flush()
            config = read_yaml(f.name)
            assert config['key'] == 'value'
            assert config['number'] == 42
            os.unlink(f.name)

    def test_read_nested_yaml(self):
        data = {'a': {'b': {'c': 1}}}
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(data, f)
            f.flush()
            config = read_yaml(f.name)
            assert config['a']['b']['c'] == 1
            os.unlink(f.name)


class TestExtractDatasetFolderLastTwoDigits:
    def test_valid_two_digits(self):
        assert extract_dataset_folder_last_two_digits("folder23") == 23
        assert extract_dataset_folder_last_two_digits("data99") == 99

    def test_single_digit(self):
        assert extract_dataset_folder_last_two_digits("folder05") == 5

    def test_no_digits(self):
        assert extract_dataset_folder_last_two_digits("folder") == -1
        assert extract_dataset_folder_last_two_digits("abc") == -1


class TestTranslateWristToOrigin:
    def test_basic_translation(self):
        joint_positions = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]])
        result = translate_wrist_to_origin(joint_positions)
        assert np.allclose(result[0], [0.0, 0.0, 0.0])
        assert np.allclose(result[1], [3.0, 3.0, 3.0])
        assert np.allclose(result[2], [6.0, 6.0, 6.0])

    def test_single_point(self):
        joint_positions = np.array([[1.0, 2.0, 3.0]])
        result = translate_wrist_to_origin(joint_positions)
        assert np.allclose(result, [[0.0, 0.0, 0.0]])


class TestApplyPoseMatrix:
    def test_identity_matrix(self):
        joint_positions = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        pose_matrix = np.eye(4)
        result = apply_pose_matrix(joint_positions, pose_matrix)
        assert np.allclose(result, joint_positions)

    def test_translation_matrix(self):
        joint_positions = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        pose_matrix = np.eye(4)
        pose_matrix[:3, 3] = [10.0, 20.0, 30.0]
        result = apply_pose_matrix(joint_positions, pose_matrix)
        assert np.allclose(result[0], [11.0, 20.0, 30.0])
        assert np.allclose(result[1], [10.0, 21.0, 30.0])


class TestInverseTransformation:
    def test_identity(self):
        matrix = np.eye(4)
        result = inverse_transformation(matrix)
        assert np.allclose(result, np.eye(4))

    def test_translation_only(self):
        matrix = np.eye(4)
        matrix[:3, 3] = [1.0, 2.0, 3.0]
        result = inverse_transformation(matrix)
        assert np.allclose(result[:3, 3], [-1.0, -2.0, -3.0])

    def test_rotation_only(self):
        matrix = np.eye(4)
        matrix[:3, :3] = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        result = inverse_transformation(matrix)
        assert np.allclose(result[:3, :3], matrix[:3, :3].T)


class TestQuaternionConversions:
    def test_trans_xyzwori_to_wxyzori(self):
        ori_xyzw = [0.1, 0.2, 0.3, 0.4]
        result = trans_xyzwori_to_wxyzori(ori_xyzw)
        assert result == (0.4, 0.1, 0.2, 0.3)

    def test_trans_wxyzori_to_xyzwori(self):
        ori_wxyz = [0.4, 0.1, 0.2, 0.3]
        result = trans_wxyzori_to_xyzwori(ori_wxyz)
        assert result == (0.1, 0.2, 0.3, 0.4)

    def test_quaternion_roundtrip(self):
        original = [0.1, 0.2, 0.3, 0.4]
        wxyz = trans_xyzwori_to_wxyzori(original)
        back = trans_wxyzori_to_xyzwori(wxyz)
        assert np.allclose(back, original)


class TestScaleValue:
    def test_identity_scale(self):
        result = scale_value(5.0, 0.0, 10.0, 0.0, 10.0)
        assert result == 5.0

    def test_range_conversion(self):
        result = scale_value(5.0, 0.0, 10.0, 0.0, 100.0)
        assert result == 50.0

    def test_negative_range(self):
        result = scale_value(5.0, 0.0, 10.0, -100.0, 0.0)
        assert result == -50.0

    def test_out_of_bounds(self):
        result = scale_value(15.0, 0.0, 10.0, 0.0, 100.0)
        assert result == 150.0


class TestIsWithinRange:
    def test_within_bounds(self):
        assert is_within_range(5.0, 0.0, 10.0) == 5.0

    def test_above_max(self):
        assert is_within_range(15.0, 0.0, 10.0) == 10.0

    def test_below_min(self):
        assert is_within_range(-5.0, 0.0, 10.0) == 0.0


class TestExtendLine:
    def test_extend_positive(self):
        point1 = [0.0, 0.0, 0.0]
        point2 = [1.0, 0.0, 0.0]
        result = extend_line(point1, point2, 1.0)
        assert np.allclose(result, [2.0, 0.0, 0.0])

    def test_extend_negative(self):
        point1 = [0.0, 0.0, 0.0]
        point2 = [1.0, 0.0, 0.0]
        result = extend_line(point1, point2, -0.5)
        assert np.allclose(result, [0.5, 0.0, 0.0])


class TestPosetureToMatrix:
    def test_identity_rotation(self):
        position = [1.0, 2.0, 3.0]
        ori = [0.0, 0.0, 0.0, 1.0]
        matrix = poseture_to_matrix(position, ori)
        assert np.allclose(matrix[:3, 3], position)

    def test_180_degree_rotation(self):
        position = [0.0, 0.0, 0.0]
        ori = [1.0, 0.0, 0.0, 0.0]
        matrix = poseture_to_matrix(position, ori)
        assert np.allclose(matrix[:3, 3], position)


class TestCalDistance:
    def test_same_point(self):
        assert cal_distance([1.0, 2.0, 3.0], [1.0, 2.0, 3.0]) == 0.0

    def test_unit_distance(self):
        assert cal_distance([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]) == 1.0

    def test_3d_distance(self):
        result = cal_distance([0.0, 0.0, 0.0], [1.0, 2.0, 2.0])
        assert np.isclose(result, 3.0)


class TestChangeList:
    def test_none_conversion(self):
        input_list = ['None', '1', '2']
        result = change_list(input_list)
        assert result == [None, '1', '2']

    def test_no_none(self):
        input_list = ['1', '2', '3']
        result = change_list(input_list)
        assert result == ['1', '2', '3']

    def test_all_none(self):
        input_list = ['None', 'None']
        result = change_list(input_list)
        assert result == [None, None]


class TestQuaternionOperations:
    def test_quaternion_conjugate(self):
        q = [1.0, 2.0, 3.0, 4.0]
        result = quaternion_conjugate(q)
        assert np.allclose(result, [-1.0, -2.0, -3.0, 4.0])

    def test_quaternion_norm_squared(self):
        q = [1.0, 2.0, 2.0, 2.0]
        result = quaternion_norm_squared(q)
        assert result == 13.0

    def test_quaternion_inverse(self):
        q = [0.0, 0.0, 0.0, 1.0]
        result = quaternion_inverse(q)
        assert np.allclose(result, [0.0, 0.0, 0.0, 1.0])

    def test_quaternion_multiply_identity(self):
        q = [0.0, 0.0, 0.0, 1.0]
        result = quaternion_multiply(q, q)
        assert np.allclose(result, q)

    def test_quaternion_multiply_rotation(self):
        q1 = [0.0, 0.0, 0.0, 1.0]
        q2 = [0.0, 0.0, 0.707, 0.707]
        result = quaternion_multiply(q1, q2)
        assert np.allclose(result, q2, atol=0.01)


class TestUnitydataToWorldspacedata:
    def test_basic_conversion(self):
        positions = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
        result = unitydata_to_worldspacedata(positions)
        assert result == [[1.0, 3.0, 2.0], [4.0, 6.0, 5.0]]


class TestQuaternionRelative:
    def test_identity_orientation(self):
        ori = [0.0, 0.0, 0.0, 1.0]
        targetori = [0.0, 0.0, 0.0, 1.0]
        result = get_quaternion_relative(ori, targetori)
        assert np.allclose(result, [0.0, 0.0, 0.0, 1.0], atol=0.01) or np.allclose(
            result, [0.0, 0.0, 0.0, -1.0], atol=0.01
        )


class TestGetChildQuaternion:
    def test_identity_combination(self):
        ori = [0.0, 0.0, 0.0, 1.0]
        ori_relative = [0.0, 0.0, 0.0, 1.0]
        result = get_child_quaternion(ori, ori_relative)
        assert np.allclose(result, [0.0, 0.0, 0.0, 1.0], atol=0.01)


class TestRotateMatrix:
    def test_rotate_matrix_x_90(self):
        result = rotate_matrix_x(np.pi / 2)
        expected = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
        assert np.allclose(result, expected)

    def test_rotate_matrix_y_90(self):
        result = rotate_matrix_y(np.pi / 2)
        expected = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
        assert np.allclose(result, expected)

    def test_rotate_matrix_z_90(self):
        result = rotate_matrix_z(np.pi / 2)
        expected = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        assert np.allclose(result, expected)

    def test_rotate_matrix_identity(self):
        assert np.allclose(rotate_matrix_x(0), np.eye(3))
        assert np.allclose(rotate_matrix_y(0), np.eye(3))
        assert np.allclose(rotate_matrix_z(0), np.eye(3))


class TestRotateQuaternion:
    def test_no_rotation(self):
        original_quat = [0.0, 0.0, 0.0, 1.0]
        result = rotate_quaternion(original_quat, 0, 0, 0)
        assert np.allclose(result, original_quat, atol=0.01)

    def test_180_degree_roll(self):
        original_quat = [0.0, 0.0, 0.0, 1.0]
        result = rotate_quaternion(original_quat, 180, 0, 0)
        assert np.allclose(result, [1.0, 0.0, 0.0, 0.0], atol=0.01)


class TestCubicModel:
    def test_basic_evaluation(self):
        result = cubic_model(1.0, 1.0, 1.0, 1.0, 1.0)
        assert result == 4.0

    def test_zero_coefficients(self):
        result = cubic_model(2.0, 0.0, 0.0, 0.0, 5.0)
        assert result == 5.0

    def test_array_input(self):
        x = np.array([0.0, 1.0, 2.0])
        result = cubic_model(x, 1.0, 0.0, 0.0, 0.0)
        assert np.allclose(result, [0.0, 1.0, 8.0])
