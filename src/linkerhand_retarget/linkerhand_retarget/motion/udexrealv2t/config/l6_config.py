# 手指配置常量
FINGER_CONFIGS = {
    'thumb_abduction': {
        'name': '拇指侧摆',
        'joints': [0, 1, 2],
        'weights': [0.3, 0.4, 0.3],
        'robot_idx': 0,
        'type': 'thumb'
    },
    'thumb_flexion': {
        'name': '拇指弯曲',
        'joints': [2, 3, 4],
        'weights': [0.3, 0.0, 0.7],
        'robot_idx': 1,
        'type': 'thumb'
    },
    'index': {
        'name': '食指',
        'joints': [6, 7, 8],
        'weights': [0.5, 0.3, 0.2],
        'robot_idx': 3,
        'type': 'finger'
    },
    'middle': {
        'name': '中指',
        'joints': [10, 11, 12],
        'weights': [0.5, 0.1, 0.4],
        'robot_idx': 5,
        'type': 'finger'
    },
    'ring': {
        'name': '无名指',
        'joints': [14, 15, 16],
        'weights': [0.5, 0.1, 0.4],
        'robot_idx': 7,
        'type': 'finger'
    },
    'pinky': {
        'name': '小指',
        'joints': [18, 19, 20],
        'weights': [0.5, 0.1, 0.4],
        'robot_idx': 9,
        'type': 'finger'
    }
}

# 映射顺序
MAPPING_ORDER = [
    'thumb_abduction', 'thumb_flexion', 
    'index', 'middle', 'ring', 'pinky'
]

# 三态默认配置
MULTI_SEGMENT_CONFIG = {
    'states': ['original', 'opose', 'fist'],
    'state_names': {
        'original': '张手',
        'opose': 'O手势',
        'fist': '握拳'
    }
}