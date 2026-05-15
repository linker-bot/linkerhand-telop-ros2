import numpy as np
import copy
from linkerhand.handcore import HandCore
from ..config.o6_config import FINGER_CONFIGS, MAPPING_ORDER
from typing import List
from linkerhand.handcoreex import MultiStateLinearMapper


# 修正 RightHand 类
class RightHand:
    def __init__(self, handcore: HandCore, length=6):
        self.handcore = handcore
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.g_jointpositions_arc = [0] * length
        self.g_jointvelocity_arc = [0] * length
        self.handstate = [0] * length
        self.calibrationopose = None
        self.calibrationfistpose = None
        self.calibrationoriginal = None
        
        # 机械手预设姿势
        self.robot_original = handcore.hand_lower_limits_l
        self.robot_opose = [1.1, 0.37, 0.0, 0.82, 0.0, 0.82, 0.0, 0.82, 0.0, 0.82, 0.0]
        self.robot_fist = handcore.hand_upper_limits_l
        
        # 映射器
        self.multi_state_mapper = MultiStateLinearMapper(FINGER_CONFIGS, MAPPING_ORDER)
    
    def initialize_mapper(self) -> bool:
        """初始化映射器"""
        glove_original = self._to_list(self.calibrationoriginal)
        glove_fist = self._to_list(self.calibrationfistpose)
        glove_opose = self._to_list(self.calibrationopose)
        
        self.multi_state_mapper.add_state('original', glove_original, self.robot_original)
        self.multi_state_mapper.add_state('opose', glove_opose, self.robot_opose)
        self.multi_state_mapper.add_state('fist', glove_fist, self.robot_fist)

        # 设置状态顺序
        self.multi_state_mapper.set_state_order(['original', 'opose', 'fist'])

    
    def _to_list(self, data):
        """转换为列表"""
        if hasattr(data, 'tolist'):
            return data.tolist()
        elif isinstance(data, np.ndarray):
            return data.tolist()
        else:
            return list(data)
    
    def joint_arc_update(self, joint_arc: List[float]):
        """映射手套数据到机械手"""
        qpos = np.zeros(25)
        arc_value = self.multi_state_mapper.map_glove_to_robot(joint_arc)
        qpos[17] = self.g_jointpositions_arc[1] = arc_value[0]
        qpos[20] = self.g_jointpositions_arc[0] = arc_value[1]
        qpos[1] = self.g_jointpositions_arc[2] = arc_value[3]
        qpos[9] = self.g_jointpositions_arc[3] = arc_value[5]
        qpos[13] = self.g_jointpositions_arc[4] = arc_value[7]
        qpos[5] = self.g_jointpositions_arc[5] = arc_value[9]
        self.g_jointpositions = self.handcore.trans_to_motor_right(qpos)
    
    def joint_update(self, joint_arc):
        qpos = np.zeros(25)
        # 拇指处理
        qpos[16] = joint_arc[1] * 1.2
        qpos[17] = joint_arc[0] * -2 + joint_arc[1] * 1.2
        qpos[20] = joint_arc[4] * 0.5 + joint_arc[2] * 0.8
 
        qpos[1] = joint_arc[6] * 0.1 + joint_arc[8] * 0.7
        qpos[9] = joint_arc[10] * 0.1 + joint_arc[12] * 0.7
        qpos[13] = joint_arc[14] * 0.1 + joint_arc[16] * 0.7
        qpos[5] = joint_arc[18] * 0.1 + joint_arc[20] * 0.7
        self.g_jointpositions = self.handcore.trans_to_motor_right(qpos)

    def speed_update(self):
        for i in range(len(self.g_jointpositions)):
            lastpos = self.last_jointpositions[i]
            position_error = int(abs(self.g_jointpositions[i] - lastpos))
            position_derict = 1 if self.g_jointpositions[i] - lastpos > 0 else -1
            slow_limit = 4
            fast_limit = 10
            max_vel = int(self.last_jointvelocity[i] * 2)
            mid_vel = int(self.last_jointvelocity[i] * 0.7)
            min_vel = int(self.last_jointvelocity[i] * 0.5)
            target_vel = self.last_jointvelocity[i]
            if self.handstate[i] == 0:  # stop
                if 0 < position_error:
                    target_vel = position_error * 5 + 30
                    self.handstate[i] = 1
            elif self.handstate[i] == 1:  # slow
                if position_error >= fast_limit:
                    target_vel = position_error * 5 + 50
                    if target_vel > mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 2
                elif position_error == 0:
                    self.handstate[i] = 0
                    target_vel = position_error * 5 + 100
                else:
                    target_vel = position_error * 5 + 100
            else:  # fast
                if position_error >= fast_limit:
                    target_vel = position_error * 5 + 90
                    if target_vel > max_vel:
                        target_vel = max_vel
                elif slow_limit < position_error < fast_limit:
                    target_vel = position_error * 5 + 60
                    if target_vel < mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 3
                elif 0 < position_error <= slow_limit:
                    target_vel = position_error * 5 + 40
                    if target_vel < min_vel:
                        target_vel = min_vel
                    self.handstate[i] = 1
            self.g_jointvelocity[i] = int(target_vel * 1)
            if self.g_jointvelocity[i] > 255:
                self.g_jointvelocity[i] = 255
            self.g_jointvelocity[i] = 255
            self.last_jointvelocity[i] = self.g_jointvelocity[i]
            self.last_jointpositions[i] = self.g_jointpositions[i]

# LeftHand 类类似修正
class LeftHand:
    def __init__(self, handcore: HandCore, length=6):
        self.handcore = handcore
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.g_jointpositions_arc = [0] * length
        self.g_jointvelocity_arc = [0] * length
        self.handstate = [0] * length
        self.calibrationopose = None
        self.calibrationfistpose = None
        self.calibrationoriginal = None

        # 机械手预设姿势
        self.robot_original = handcore.hand_lower_limits_l
        self.robot_opose = [1.1, 0.37, 0.0, 0.82, 0.0, 0.82, 0.0, 0.82, 0.0, 0.82, 0.0]
        self.robot_fist = handcore.hand_upper_limits_l
        
        # 使用多态映射器（支持更多手势）
        self.multi_state_mapper = MultiStateLinearMapper(FINGER_CONFIGS, MAPPING_ORDER)

        
    def initialize_mapper(self) -> bool:
        """初始化映射器"""
        glove_original = self._to_list(self.calibrationoriginal)
        glove_fist = self._to_list(self.calibrationfistpose)
        glove_opose = self._to_list(self.calibrationopose)
        
        self.multi_state_mapper.add_state('original', glove_original, self.robot_original)
        self.multi_state_mapper.add_state('opose', glove_opose, self.robot_opose)
        self.multi_state_mapper.add_state('fist', glove_fist, self.robot_fist)

        glove_pinch = glove_original.copy()
        glove_pinch[2:5] = [1.5, 1.2, 1.0]  # 拇指弯曲明显
        glove_pinch[0:2] = glove_original[0:2]  # 拇指侧摆保持原始
        robot_pinch = self.robot_original.copy()
        robot_pinch[1] = 0.8  # 拇指弯曲加大
        robot_pinch[3] = 0.6  # 食指轻微弯曲

        self.multi_state_mapper.add_state('pinch', glove_pinch, robot_pinch)

        # 设置状态顺序
        self.multi_state_mapper.set_state_order(['original', 'opose', 'fist'])

    
    def _to_list(self, data):
        """转换为列表"""
        if hasattr(data, 'tolist'):
            return data.tolist()
        elif isinstance(data, np.ndarray):
            return data.tolist()
        else:
            return list(data)
    
    def joint_arc_update(self, joint_arc: List[float]):
        """映射手套数据到机械手"""
        qpos = np.zeros(25)
        arc_value = self.multi_state_mapper.map_glove_to_robot(joint_arc)
        qpos[17] = self.g_jointpositions_arc[1] = arc_value[0]
        qpos[20] = self.g_jointpositions_arc[0] = arc_value[1]
        qpos[1] = self.g_jointpositions_arc[2] = arc_value[3]
        qpos[9] = self.g_jointpositions_arc[3] = arc_value[5]
        qpos[13] = self.g_jointpositions_arc[4] = arc_value[7]
        qpos[5] = self.g_jointpositions_arc[5] = arc_value[9]
        self.g_jointpositions = self.handcore.trans_to_motor_right(qpos)
    
    def joint_update(self, joint_arc):
        qpos = np.zeros(25)
        # 拇指处理
        qpos[16] = joint_arc[1] * 1.2
        qpos[17] = joint_arc[0] * -2 + joint_arc[1] * 1.2
        qpos[20] = joint_arc[4] * 0.5 + joint_arc[2] * 0.8
 
        qpos[1] = joint_arc[6] * 0.1 + joint_arc[8] * 0.7
        qpos[9] = joint_arc[10] * 0.1 + joint_arc[12] * 0.7
        qpos[13] = joint_arc[14] * 0.1 + joint_arc[16] * 0.7
        qpos[5] = joint_arc[18] * 0.1 + joint_arc[20] * 0.7
        self.g_jointpositions = self.handcore.trans_to_motor_right(qpos)


    def speed_update(self):
        for i in range(len(self.g_jointpositions)):
            lastpos = self.last_jointpositions[i]
            position_error = int(abs(self.g_jointpositions[i] - lastpos))
            position_derict = 1 if self.g_jointpositions[i] - lastpos > 0 else -1
            slow_limit = 4
            fast_limit = 10
            max_vel = int(self.last_jointvelocity[i] * 2)
            mid_vel = int(self.last_jointvelocity[i] * 0.7)
            min_vel = int(self.last_jointvelocity[i] * 0.5)
            target_vel = self.last_jointvelocity[i]
            if self.handstate[i] == 0:  # stop
                if 0 < position_error:
                    target_vel = position_error * 5 + 30
                    self.handstate[i] = 1
            elif self.handstate[i] == 1:  # slow
                if position_error >= fast_limit:
                    target_vel = position_error * 5 + 50
                    if target_vel > mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 2
                elif position_error == 0:
                    self.handstate[i] = 0
                    target_vel = position_error * 5 + 100
                else:
                    target_vel = position_error * 5 + 100
            else:  # fast
                if position_error >= fast_limit:
                    target_vel = position_error * 5 + 90
                    if target_vel > max_vel:
                        target_vel = max_vel
                elif slow_limit < position_error < fast_limit:
                    target_vel = position_error * 5 + 60
                    if target_vel < mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 3
                elif 0 < position_error <= slow_limit:
                    target_vel = position_error * 5 + 40
                    if target_vel < min_vel:
                        target_vel = min_vel
                    self.handstate[i] = 1
            self.g_jointvelocity[i] = int(target_vel * 1)
            if self.g_jointvelocity[i] > 255:
                self.g_jointvelocity[i] = 255
            self.g_jointvelocity[i] = 255
            self.last_jointvelocity[i] = self.g_jointvelocity[i]
            self.last_jointpositions[i] = self.g_jointpositions[i]
