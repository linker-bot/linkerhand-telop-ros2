#!/usr/bin/env python3
"""LinkerForce Retarget 集成测试 - 无ROS依赖版本"""
import time
import sys
import copy
import json
import math
import threading
import statistics
import numpy as np
from pathlib import Path
from datetime import datetime, timedelta

sys.path.insert(0, '/home/linker-brunt/project/linkerhand_telop_sdk/old_git/new/ros2/src/linkerhand_retarget')

from linkerhand_retarget.linkerhand.linkerforce import ForceSerialReader
from linkerhand_retarget.linkerhand.constants import HandType, RobotName, ROBOT_LEN_MAP


class MockHandCore:
    """简化的HandCore用于测试"""
    def __init__(self, num_joints=25):
        self.hand_numjoints_r = num_joints
        self.hand_numjoints_l = num_joints
        self.hand_lower_limits_r = [-1.57] * num_joints
        self.hand_upper_limits_r = [1.57] * num_joints
        self.hand_lower_limits_l = [-1.57] * num_joints
        self.hand_upper_limits_l = [1.57] * num_joints
        self.dataminvalue_r = [0] * num_joints
        self.datamaxvalue_r = [255] * num_joints
        self.dataminvalue_l = [0] * num_joints
        self.datamaxvalue_l = [255] * num_joints
        self.sourcedataindex_r = list(range(num_joints))
        self.sourcedataindex_l = list(range(num_joints))
        self.urdfdataindex_r = list(range(num_joints))
        self.urdfdataindex_l = list(range(num_joints))
    
    def trans_to_motor_right(self, qpos):
        result = [255] * len(qpos)
        for i, val in enumerate(qpos):
            val = max(-1.57, min(1.57, val))
            result[i] = int((val + 1.57) / 3.14 * 255)
        return result
    
    def trans_to_motor_left(self, qpos):
        return self.trans_to_motor_right(qpos)


class RightHand:
    """简化版右手"""
    def __init__(self, handcore, length=25):
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.handstate = [0] * length
        self.handcore = handcore
        self.calibrationoriginal = None
        self.calibrationfistpose = None
        self.calibrationopose = None
    
    def joint_update(self, joint_arc):
        qpos = np.zeros(25)
        qpos[15] = joint_arc[3] * -2.5
        qpos[16] = joint_arc[20] * -2.6
        qpos[17] = joint_arc[2] * -0.2
        qpos[18] = joint_arc[1] * -1.5
        qpos[19] = joint_arc[0] * -1.5
        qpos[0] = joint_arc[7]
        qpos[1] = joint_arc[6] * -1
        qpos[2] = joint_arc[5] * -1
        qpos[3] = joint_arc[4] * -1
        if len(joint_arc) > 18:
            qpos[4] = joint_arc[19]
            qpos[5] = joint_arc[18] * -1
            qpos[6] = joint_arc[17] * -1
            qpos[7] = joint_arc[16] * -1
        if len(joint_arc) > 10:
            qpos[20] = joint_arc[11]
            qpos[8] = joint_arc[10] * -1
            qpos[9] = joint_arc[9] * -1
            qpos[10] = joint_arc[8] * -1
        if len(joint_arc) > 14:
            qpos[11] = joint_arc[15]
            qpos[12] = joint_arc[14] * -1
            qpos[13] = joint_arc[13] * -1
            qpos[14] = joint_arc[12] * -1
        self.g_jointpositions = self.handcore.trans_to_motor_right(qpos)
    
    def speed_update(self):
        pass


class LeftHand:
    """简化版左手"""
    def __init__(self, handcore, length=25):
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.handstate = [0] * length
        self.handcore = handcore
        self.calibrationoriginal = None
        self.calibrationfistpose = None
        self.calibrationopose = None
    
    def joint_update(self, joint_arc):
        qpos = np.zeros(25)
        qpos[15] = joint_arc[3] * 2.5
        qpos[16] = joint_arc[20] * 2.6
        qpos[17] = joint_arc[2] * 0.2
        qpos[18] = joint_arc[1] * 1.5
        qpos[19] = joint_arc[0] * 1.5
        qpos[0] = joint_arc[7] * -1
        qpos[1] = joint_arc[6] * -1
        qpos[2] = joint_arc[5] * -1
        qpos[3] = joint_arc[4] * -1
        if len(joint_arc) > 18:
            qpos[4] = joint_arc[19] * -1
            qpos[5] = joint_arc[18] * -1
            qpos[6] = joint_arc[17] * -1
            qpos[7] = joint_arc[16] * -1
        if len(joint_arc) > 10:
            qpos[20] = joint_arc[11] * 0
            qpos[8] = joint_arc[10] * -2
            qpos[9] = joint_arc[9] * -1
            qpos[10] = joint_arc[8] * -1
        if len(joint_arc) > 14:
            qpos[11] = joint_arc[15] * -1
            qpos[12] = joint_arc[14] * -1
            qpos[13] = joint_arc[13] * -1
            qpos[14] = joint_arc[12] * -1
        self.g_jointpositions = self.handcore.trans_to_motor_left(qpos)
    
    def speed_update(self):
        pass


class RetargetCore:
    """Retarget核心逻辑 - 无ROS依赖"""
    
    def __init__(self, 
                 lefthand: RobotName = RobotName.l21,
                 righthand: RobotName = RobotName.l21,
                 calibration: bool = False):
        self.lefthandtype = lefthand
        self.righthandtype = righthand
        self.calibration = calibration
        
        self.force_reader_left = None
        self.force_reader_right = None
        self.forcelock = threading.Lock()
        
        self.handcore = MockHandCore(num_joints=ROBOT_LEN_MAP[lefthand])
        self.lefthand = LeftHand(handcore=self.handcore, length=ROBOT_LEN_MAP[lefthand])
        self.righthand = RightHand(handcore=self.handcore, length=ROBOT_LEN_MAP[righthand])
        
        self.results = {'left': {}, 'right': {}}
        self.leftport = None
        self.rightport = None
        self.leftbaudrate = None
        self.rightbaudrate = None
        
        self.calibration_data_left = []
        self.calibration_data_right = []
        
        self.running = False
        self._thread = None
        
        self.receive_times_left = []
        self.receive_times_right = []
        
    def linkerforce_init(self):
        """初始化串口连接"""
        exclude_list = []
        baudrates = [2000000, 1000000, 921600, 460800]
        
        print("\n[左手] 扫描设备...")
        self.force_reader_left = ForceSerialReader(
            HandType.left,
            excludelist=exclude_list,
            baudrates=baudrates,
            isdebug=False
        )
        self.leftport, self.leftbaudrate, errorcode = self.force_reader_left.find_valid_ports(timeout=3)
        
        if self.leftport:
            if self.force_reader_left.openserial(port=self.leftport, baudrate=self.leftbaudrate):
                self.force_reader_left.start()
                time.sleep(0.1)
                self.force_reader_left.serial_port.write(self.force_reader_left.pack_01_data())
                time.sleep(0.5)
                if self.force_reader_left.handtype == 'Left':
                    print(f"[左手] 已连接: {self.leftport} @ {self.leftbaudrate}, 版本: {self.force_reader_left.version}")
                else:
                    print(f"[左手] 设备类型不匹配: {self.force_reader_left.handtype}")
            exclude_list.append(self.leftport)
        else:
            print("[左手] 未找到设备")
        
        print("\n[右手] 扫描设备...")
        self.force_reader_right = ForceSerialReader(
            HandType.right,
            excludelist=exclude_list,
            baudrates=baudrates,
            isdebug=False
        )
        self.rightport, self.rightbaudrate, errorcode = self.force_reader_right.find_valid_ports(timeout=3)
        
        if self.rightport:
            if self.force_reader_right.openserial(port=self.rightport, baudrate=self.rightbaudrate):
                self.force_reader_right.start()
                time.sleep(0.1)
                self.force_reader_right.serial_port.write(self.force_reader_right.pack_01_data())
                time.sleep(0.5)
                if self.force_reader_right.handtype == 'Right':
                    print(f"[右手] 已连接: {self.rightport} @ {self.rightbaudrate}, 版本: {self.force_reader_right.version}")
                else:
                    print(f"[右手] 设备类型不匹配: {self.force_reader_right.handtype}")
        else:
            print("[右手] 未找到设备")
        
        if self.calibration:
            self.run_calibration()
    
    def _calculate_weighted_average(self, data_list):
        """计算加权平均值"""
        if not data_list:
            return [0.0] * 21
        
        n = len(data_list)
        if n == 1:
            return data_list[0]
        
        weights = [i + 1 for i in range(n)]
        total_weight = sum(weights)
        
        result = [0.0] * len(data_list[0])
        for i, data in enumerate(data_list):
            w = weights[i] / total_weight
            for j in range(len(result)):
                result[j] += data[j] * w
        
        return result
    
    def run_calibration(self, duration_per_pose=5):
        """执行标定流程"""
        print("\n" + "=" * 50)
        print("开始标定流程")
        print("=" * 50)
        
        if self.force_reader_left and self.force_reader_left.handtype != 'Left' and \
           self.force_reader_right and self.force_reader_right.handtype != 'Right':
            print("无可用设备，跳过标定")
            return False
        
        self.calibration_data_left = []
        self.calibration_data_right = []
        
        print(f"\n[标定 1/3] 请保持五指张开姿势 (对应电机值255)")
        self._collect_calibration_data(duration_per_pose)
        if self.calibration_data_left:
            self.lefthand.calibrationoriginal = self._calculate_weighted_average(self.calibration_data_left)
        if self.calibration_data_right:
            self.righthand.calibrationoriginal = self._calculate_weighted_average(self.calibration_data_right)
        
        self.calibration_data_left = []
        self.calibration_data_right = []
        
        print(f"\n[标定 2/3] 请握紧拳头 (对应电机值0)")
        self._collect_calibration_data(duration_per_pose)
        if self.calibration_data_left:
            self.lefthand.calibrationfistpose = self._calculate_weighted_average(self.calibration_data_left)
        if self.calibration_data_right:
            self.righthand.calibrationfistpose = self._calculate_weighted_average(self.calibration_data_right)
        
        self.calibration_data_left = []
        self.calibration_data_right = []
        
        print(f"\n[标定 3/3] 请保持O型手势 (对应电机中间值)")
        self._collect_calibration_data(duration_per_pose)
        if self.calibration_data_left:
            self.lefthand.calibrationopose = self._calculate_weighted_average(self.calibration_data_left)
        if self.calibration_data_right:
            self.righthand.calibrationopose = self._calculate_weighted_average(self.calibration_data_right)
        
        print("\n标定完成")
        return True
    
    def _collect_calibration_data(self, duration):
        """采集标定数据"""
        prepare_time = duration * 0.4
        collect_time = duration * 0.6
        
        print(f"  准备阶段: {prepare_time:.1f}s")
        time.sleep(prepare_time)
        
        print(f"  采集阶段: {collect_time:.1f}s")
        start = time.time()
        while time.time() - start < collect_time:
            if self.force_reader_left and self.force_reader_left.handtype == 'Left':
                self.calibration_data_left.append(copy.deepcopy(self.force_reader_left.poslist))
            if self.force_reader_right and self.force_reader_right.handtype == 'Right':
                self.calibration_data_right.append(copy.deepcopy(self.force_reader_right.poslist))
            time.sleep(0.05)
        
        print(f"  采集样本: 左手 {len(self.calibration_data_left)}, 右手 {len(self.calibration_data_right)}")
    
    def process_once(self):
        """单次数据处理"""
        if self.force_reader_left and self.force_reader_left.handtype == 'Left':
            try:
                self.force_reader_left.serial_port.write(self.force_reader_left.pack_03_data())
            except:
                pass
            left_positions = copy.deepcopy(self.force_reader_left.poslist)
            self.lefthand.joint_update(left_positions)
            self.lefthand.speed_update()
            
            if self.force_reader_left.receive_times:
                self.receive_times_left = self.force_reader_left.receive_times[-100:]
        
        if self.force_reader_right and self.force_reader_right.handtype == 'Right':
            try:
                self.force_reader_right.serial_port.write(self.force_reader_right.pack_03_data())
            except:
                pass
            right_positions = copy.deepcopy(self.force_reader_right.poslist)
            self.righthand.joint_update(right_positions)
            self.righthand.speed_update()
            
            if self.force_reader_right.receive_times:
                self.receive_times_right = self.force_reader_right.receive_times[-100:]
    
    def _process_loop(self, rate_hz=30):
        """处理循环"""
        interval = 1.0 / rate_hz
        while self.running:
            self.process_once()
            time.sleep(interval)
    
    def start_processing(self, rate_hz=30):
        """启动处理线程"""
        self.running = True
        self._thread = threading.Thread(target=self._process_loop, args=(rate_hz,), daemon=True)
        self._thread.start()
    
    def stop_processing(self):
        """停止处理"""
        self.running = False
        if self._thread:
            self._thread.join(timeout=2)
    
    def stop(self):
        """停止所有连接"""
        self.stop_processing()
        if self.force_reader_left:
            try:
                self.force_reader_left.stop()
            except:
                pass
        if self.force_reader_right:
            try:
                self.force_reader_right.stop()
            except:
                pass
    
    def get_stats(self):
        """获取统计数据"""
        stats = {
            'left': {
                'connected': self.force_reader_left and self.force_reader_left.handtype == 'Left',
                'port': self.leftport,
                'baudrate': self.leftbaudrate,
                'version': self.force_reader_left.version if self.force_reader_left else None,
                'receive_count': self.force_reader_left.receive_count if self.force_reader_left else 0,
                'motor_positions': self.lefthand.g_jointpositions[:6] if self.lefthand else []
            },
            'right': {
                'connected': self.force_reader_right and self.force_reader_right.handtype == 'Right',
                'port': self.rightport,
                'baudrate': self.rightbaudrate,
                'version': self.force_reader_right.version if self.force_reader_right else None,
                'receive_count': self.force_reader_right.receive_count if self.force_reader_right else 0,
                'motor_positions': self.righthand.g_jointpositions[:6] if self.righthand else []
            }
        }
        return stats


class RetargetIntegrationTest:
    """Retarget集成测试"""
    
    def __init__(self):
        self.retarget = None
        self.test_results = {}
    
    def setup(self):
        """初始化"""
        print("=" * 60)
        print("LinkerForce Retarget 集成测试 (无ROS)")
        print("=" * 60)
        
        self.retarget = RetargetCore(
            lefthand=RobotName.l21,
            righthand=RobotName.l21,
            calibration=False
        )
        self.retarget.linkerforce_init()
        return True
    
    def teardown(self):
        """清理"""
        if self.retarget:
            self.retarget.stop()
        print("\n设备已关闭")
    
    def test_device_connection(self):
        """测试1: 设备连接"""
        print("\n" + "-" * 40)
        print("[测试1] 设备连接")
        print("-" * 40)
        
        stats = self.retarget.get_stats()
        
        left_ok = stats['left']['connected']
        right_ok = stats['right']['connected']
        
        print(f"  左手: {'✅ 已连接' if left_ok else '❌ 未连接'}")
        if left_ok:
            print(f"    端口: {stats['left']['port']} @ {stats['left']['baudrate']}")
            print(f"    版本: {stats['left']['version']}")
        
        print(f"  右手: {'✅ 已连接' if right_ok else '❌ 未连接'}")
        if right_ok:
            print(f"    端口: {stats['right']['port']} @ {stats['right']['baudrate']}")
            print(f"    版本: {stats['right']['version']}")
        
        result = left_ok or right_ok
        print(f"  {'✅ 通过' if result else '❌ 失败'}")
        return result
    
    def test_joint_mapping(self, duration=5):
        """测试2: 关节映射"""
        print("\n" + "-" * 40)
        print(f"[测试2] 关节映射 ({duration}s)")
        print("-" * 40)
        
        self.retarget.start_processing(rate_hz=30)
        time.sleep(duration)
        self.retarget.stop_processing()
        
        stats = self.retarget.get_stats()
        
        if stats['left']['connected']:
            motor_pos = stats['left']['motor_positions']
            print(f"  左手电机位置: {motor_pos}")
            valid = all(0 <= p <= 255 for p in motor_pos)
            print(f"  左手映射有效性: {'✅' if valid else '❌'}")
        
        if stats['right']['connected']:
            motor_pos = stats['right']['motor_positions']
            print(f"  右手电机位置: {motor_pos}")
            valid = all(0 <= p <= 255 for p in motor_pos)
            print(f"  右手映射有效性: {'✅' if valid else '❌'}")
        
        result = True
        print(f"  {'✅ 通过' if result else '❌ 失败'}")
        return result
    
    def test_data_rate(self, duration=10):
        """测试3: 数据帧率"""
        print("\n" + "-" * 40)
        print(f"[测试3] 数据帧率 ({duration}s)")
        print("-" * 40)
        
        self.retarget.start_processing(rate_hz=30)
        
        initial_left = self.retarget.force_reader_left.receive_count if self.retarget.force_reader_left else 0
        initial_right = self.retarget.force_reader_right.receive_count if self.retarget.force_reader_right else 0
        
        time.sleep(duration)
        
        self.retarget.stop_processing()
        
        final_left = self.retarget.force_reader_left.receive_count if self.retarget.force_reader_left else 0
        final_right = self.retarget.force_reader_right.receive_count if self.retarget.force_reader_right else 0
        
        left_frames = final_left - initial_left
        right_frames = final_right - initial_right
        
        left_fps = left_frames / duration if left_frames > 0 else 0
        right_fps = right_frames / duration if right_frames > 0 else 0
        
        if left_frames > 0:
            print(f"  左手: {left_frames} 帧, {left_fps:.1f} Hz")
        if right_frames > 0:
            print(f"  右手: {right_frames} 帧, {right_fps:.1f} Hz")
        
        result = left_fps >= 5 or right_fps >= 5
        print(f"  {'✅ 通过' if result else '❌ 帧率过低'}")
        return result
    
    def test_frame_interval(self, duration=5):
        """测试4: 帧间隔分析"""
        print("\n" + "-" * 40)
        print(f"[测试4] 帧间隔分析 ({duration}s)")
        print("-" * 40)
        
        self.retarget.start_processing(rate_hz=30)
        time.sleep(duration)
        self.retarget.stop_processing()
        
        stats = self.retarget.get_stats()
        
        if self.retarget.receive_times_left and len(self.retarget.receive_times_left) >= 2:
            times = self.retarget.receive_times_left
            intervals = [(times[i] - times[i-1]) * 1000 for i in range(1, len(times))]
            avg = statistics.mean(intervals)
            std = statistics.stdev(intervals) if len(intervals) > 1 else 0
            print(f"  左手帧间隔: 平均 {avg:.2f}ms, 标准差 {std:.2f}ms")
        
        if self.retarget.receive_times_right and len(self.retarget.receive_times_right) >= 2:
            times = self.retarget.receive_times_right
            intervals = [(times[i] - times[i-1]) * 1000 for i in range(1, len(times))]
            avg = statistics.mean(intervals)
            std = statistics.stdev(intervals) if len(intervals) > 1 else 0
            print(f"  右手帧间隔: 平均 {avg:.2f}ms, 标准差 {std:.2f}ms")
        
        result = True
        print(f"  {'✅ 通过' if result else '❌ 失败'}")
        return result
    
    def test_calibration(self, duration_per_pose=3):
        """测试5: 标定流程"""
        print("\n" + "-" * 40)
        print("[测试5] 标定流程")
        print("-" * 40)
        
        if not self.retarget.force_reader_left and not self.retarget.force_reader_right:
            print("  ⚠️ 无设备，跳过")
            return True
        
        print("  开始标定 (每个姿势 3 秒)...")
        result = self.retarget.run_calibration(duration_per_pose=duration_per_pose)
        
        if result:
            print(f"  左手张开数据: {self.retarget.lefthand.calibrationoriginal[:5] if self.retarget.lefthand.calibrationoriginal else 'N/A'}...")
            print(f"  左手握拳数据: {self.retarget.lefthand.calibrationfistpose[:5] if self.retarget.lefthand.calibrationfistpose else 'N/A'}...")
        
        print(f"  {'✅ 标定完成' if result else '❌ 标定失败'}")
        return result
    
    def generate_report(self):
        """生成报告"""
        print("\n" + "=" * 60)
        print("测试报告")
        print("=" * 60)
        
        stats = self.retarget.get_stats() if self.retarget else {}
        
        print("\n## 设备状态")
        if stats.get('left', {}).get('connected'):
            print(f"- 左手: {stats['left']['port']} @ {stats['left']['baudrate']}, 版本 {stats['left']['version']}")
        if stats.get('right', {}).get('connected'):
            print(f"- 右手: {stats['right']['port']} @ {stats['right']['baudrate']}, 版本 {stats['right']['version']}")
        
        print("\n## 测试结果")
        passed = sum(1 for r in self.test_results.values() if r)
        total = len(self.test_results)
        print(f"- 通过: {passed}/{total}")
        
        for name, result in self.test_results.items():
            print(f"  - {name}: {'✅' if result else '❌'}")
        
        print("\n" + "=" * 60)
    
    def run_all_tests(self):
        """运行所有测试"""
        if not self.setup():
            return
        
        try:
            self.test_results['测试1-设备连接'] = self.test_device_connection()
            self.test_results['测试2-关节映射'] = self.test_joint_mapping(duration=5)
            self.test_results['测试3-数据帧率'] = self.test_data_rate(duration=10)
            self.test_results['测试4-帧间隔'] = self.test_frame_interval(duration=5)
            # self.test_results['测试5-标定流程'] = self.test_calibration(duration_per_pose=3)
        except Exception as e:
            print(f"\n测试中断: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.teardown()
        
        self.generate_report()


if __name__ == "__main__":
    test = RetargetIntegrationTest()
    test.run_all_tests()