import pytest
import numpy as np
from linkerhand_retarget.linkerhand.handcoreex import MultiStateLinearMapper


FINGER_CONFIGS_TEST = {
    'thumb': {
        'name': 'thumb',
        'joints': [0, 1, 2],
        'weights': [0.2, 0.3, 0.5],
        'robot_idx': 0,
        'reverse_motion': False,
    },
    'index': {
        'name': 'index',
        'joints': [3, 4, 5],
        'weights': [0.3, 0.3, 0.4],
        'robot_idx': 1,
        'reverse_motion': False,
    },
}

MAPPING_ORDER_TEST = ['thumb', 'index']


class TestMultiStateLinearMapper:
    def test_initialization(self):
        mapper = MultiStateLinearMapper(FINGER_CONFIGS_TEST, MAPPING_ORDER_TEST)
        assert mapper.finger_configs == FINGER_CONFIGS_TEST
        assert mapper.mapping_order == MAPPING_ORDER_TEST
        assert len(mapper.glove_states) == 0
        assert len(mapper.robot_states) == 0

    def test_add_state(self):
        mapper = MultiStateLinearMapper(FINGER_CONFIGS_TEST, MAPPING_ORDER_TEST)
        glove_angles = [0.0] * 21
        robot_angles = [0.0] * 6
        
        mapper.add_state('original', glove_angles, robot_angles)
        
        assert 'original' in mapper.glove_states
        assert 'original' in mapper.robot_states
        assert np.array_equal(mapper.glove_states['original'], glove_angles)

    def test_add_state_with_list(self):
        mapper = MultiStateLinearMapper(FINGER_CONFIGS_TEST, MAPPING_ORDER_TEST)
        glove_angles = [1.0] * 21
        robot_angles = [0.5] * 6
        
        mapper.add_state('fist', glove_angles, robot_angles)
        
        assert 'fist' in mapper.glove_states

    def test_remove_state(self):
        mapper = MultiStateLinearMapper(FINGER_CONFIGS_TEST, MAPPING_ORDER_TEST)
        glove_angles = [0.0] * 21
        robot_angles = [0.0] * 6
        
        mapper.add_state('original', glove_angles, robot_angles)
        mapper.remove_state('original')
        
        assert 'original' not in mapper.glove_states

    def test_set_state_order(self):
        mapper = MultiStateLinearMapper(FINGER_CONFIGS_TEST, MAPPING_ORDER_TEST)
        mapper.add_state('original', [0.0] * 21, [0.0] * 6)
        mapper.add_state('fist', [1.0] * 21, [1.0] * 6)
        
        mapper.set_state_order(['original', 'fist'])
        
        assert mapper.state_order == ['original', 'fist']

    def test_set_state_order_invalid_state(self):
        mapper = MultiStateLinearMapper(FINGER_CONFIGS_TEST, MAPPING_ORDER_TEST)
        mapper.add_state('original', [0.0] * 21, [0.0] * 6)
        
        with pytest.raises(ValueError):
            mapper.set_state_order(['original', 'nonexistent'])

    def test_map_glove_to_robot_requires_two_states(self):
        mapper = MultiStateLinearMapper(FINGER_CONFIGS_TEST, MAPPING_ORDER_TEST)
        
        with pytest.raises(ValueError, match="请至少设置两个状态"):
            mapper.map_glove_to_robot([0.0] * 21)

    def test_map_glove_to_robot_with_original(self):
        mapper = MultiStateLinearMapper(FINGER_CONFIGS_TEST, MAPPING_ORDER_TEST)
        mapper.add_state('original', [0.0] * 21, [0.0] * 6)
        mapper.add_state('fist', [1.0] * 21, [1.0] * 6)
        mapper.set_state_order(['original', 'fist'])
        
        result = mapper.map_glove_to_robot([0.5] * 21)
        
        assert isinstance(result, np.ndarray)

    def test_map_glove_to_robot_returns_array(self):
        mapper = MultiStateLinearMapper(FINGER_CONFIGS_TEST, MAPPING_ORDER_TEST)
        mapper.add_state('original', [0.0] * 21, [0.0] * 6)
        mapper.add_state('fist', [1.0] * 21, [1.0] * 6)
        mapper.set_state_order(['original', 'fist'])
        
        result = mapper.map_glove_to_robot([0.5] * 21)
        
        assert isinstance(result, np.ndarray)
        assert len(result) == 6

    def test_map_glove_to_robot_with_numpy_array(self):
        mapper = MultiStateLinearMapper(FINGER_CONFIGS_TEST, MAPPING_ORDER_TEST)
        mapper.add_state('original', [0.0] * 21, [0.0] * 6)
        mapper.add_state('fist', [1.0] * 21, [1.0] * 6)
        mapper.set_state_order(['original', 'fist'])
        
        result = mapper.map_glove_to_robot(np.array([0.5] * 21))
        
        assert isinstance(result, np.ndarray)

    def test_map_glove_to_robot_full_extension(self):
        mapper = MultiStateLinearMapper(FINGER_CONFIGS_TEST, MAPPING_ORDER_TEST)
        mapper.add_state('original', [0.0] * 21, [0.0] * 6)
        mapper.add_state('fist', [1.0] * 21, [1.0] * 6)
        mapper.set_state_order(['original', 'fist'])
        
        result = mapper.map_glove_to_robot([0.0] * 21)
        
        assert isinstance(result, np.ndarray)

    def test_map_glove_to_robot_full_flexion(self):
        mapper = MultiStateLinearMapper(FINGER_CONFIGS_TEST, MAPPING_ORDER_TEST)
        mapper.add_state('original', [0.0] * 21, [0.0] * 6)
        mapper.add_state('fist', [1.0] * 21, [1.0] * 6)
        mapper.set_state_order(['original', 'fist'])
        
        result = mapper.map_glove_to_robot([1.0] * 21)
        
        assert isinstance(result, np.ndarray)


class TestMultiStateLinearMapperEdgeCases:
    def test_empty_glove_states(self):
        mapper = MultiStateLinearMapper(FINGER_CONFIGS_TEST, MAPPING_ORDER_TEST)
        assert len(mapper.glove_states) == 0

    def test_debug_value_initialized(self):
        mapper = MultiStateLinearMapper(FINGER_CONFIGS_TEST, MAPPING_ORDER_TEST)
        assert len(mapper.debug_value) == 20

    def test_history_initialized(self):
        mapper = MultiStateLinearMapper(FINGER_CONFIGS_TEST, MAPPING_ORDER_TEST)
        assert len(mapper.raw_history) == 0
        assert len(mapper.filtered_history) == 0

    def test_multiple_states(self):
        mapper = MultiStateLinearMapper(FINGER_CONFIGS_TEST, MAPPING_ORDER_TEST)
        mapper.add_state('original', [0.0] * 21, [0.0] * 6)
        mapper.add_state('opose', [0.5] * 21, [0.5] * 6)
        mapper.add_state('fist', [1.0] * 21, [1.0] * 6)
        
        assert len(mapper.glove_states) == 3

    def test_normalize_weights_accepts_python_lists(self):
        mapper = MultiStateLinearMapper(FINGER_CONFIGS_TEST, MAPPING_ORDER_TEST)

        assert mapper._normalize_weights([1.0, 0.0, 0.0]) == [1.0, 0.0, 0.0]
