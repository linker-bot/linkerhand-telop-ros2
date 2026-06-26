import time
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int32MultiArray, Header, Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from pathlib import Path

# 将项目根目录放在最前面
# 强制使用项目本地的 linkerhand 模块
_project_root = Path(__file__).absolute().parent.parent.parent
_project_root_str = str(_project_root)

if _project_root_str in sys.path:
    sys.path.remove(_project_root_str)
sys.path.insert(0, _project_root_str)

from linkerhand.linkerforce import ForceSerialReader
from linkerhand.constants import RobotName, ROBOT_LEN_MAP, HandType
from linkerhand.handcore import HandCore
from tqdm import tqdm
from pathlib import Path
from colorama import Fore, init
from datetime import datetime, timedelta
import threading
import copy
import pickle
import os
import json
import sys
import serial.tools.list_ports
import numpy as np
import math
import yaml


TMP_FILE_PATH = Path(__file__).parent / "tmp" / "jointangle_data.tmp"
SAMPLE_FILE_PATH = Path(__file__).parent.parent.parent / "config" / "calibration_sample.yml"


class Retarget():
    def __init__(self, 
                node,
                righthand: RobotName, 
                lefthand: RobotName, 
                handcore: HandCore,
                lefthandpubprint: bool, 
                righthandpubprint: bool,
                calibration: bool = False,
                auto_detect: bool = True,
                isgetdebug: bool = True,
                baseconfig: dict = None,
                cmd_ports: list = None,
                cmd_baudrate: int = None,
                cmd_auto_scan: bool = None):
        """
        初始化 LinkerForce Retarget 模块 (ROS1 版本)
        
        Args:
            leftport: 左手串口路径（如 '/dev/ttyUSB0'），auto_detect=True 时可忽略
            leftbaudrate: 左手波特率
            rightport: 右手串口路径（如 '/dev/ttyUSB1'），auto_detect=True 时可忽略
            rightbaudrate: 右手波特率
            lefthand: 左手机器人类型
            righthand: 右手机器人类型
            handcore: HandCore 实例
            lefthandpubprint: 是否打印左手调试信息
            righthandpubprint: 是否打印右手调试信息
            calibration: True=强制标定, False=尝试加载缓存
            auto_detect: 是否自动检测串口（默认 True）
            isgetdebug: 是否发布debug测试数据话题（默认False）
            baseconfig: 基础配置字典
        """        

        self.node = node
        self.lefthandtype = lefthand
        self.righthandtype = righthand
        self.handcore = handcore
        self.runing = True
        self.lefthandpubprint = lefthandpubprint
        self.righthandpubprint = righthandpubprint
        self.isdebugpub = isgetdebug
        self.baseconfig = baseconfig or {}
        
        self.show_fist_calibration = self.baseconfig.get('calibration', {}).get('show_fist', True)
        self.fist_extend_ratio = self.baseconfig.get('calibration', {}).get('fist_extend_ratio', 0.5)
        
        # 命令行串口参数（候选列表，系统自动识别左右手）
        self.cmd_ports = cmd_ports
        self.cmd_baudrate = cmd_baudrate
        self.cmd_auto_scan = cmd_auto_scan

        # 根据右手类型初始化
        mapper_debug = self.baseconfig.get('debug', {}).get('mapper_debug', False)
        
        if self.righthandtype == RobotName.o7 \
            or self.righthandtype == RobotName.l7 \
            or self.righthandtype == RobotName.o7v1 \
            or self.righthandtype == RobotName.o7v3:
            from .hand.linkerforce_l7 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand], is_debug=mapper_debug)
            
        elif self.righthandtype == RobotName.o6:
            from .hand.linkerforce_o6 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand], is_debug=mapper_debug)
        elif self.righthandtype == RobotName.l6:
            from .hand.linkerforce_l6 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand], is_debug=mapper_debug)
        elif self.righthandtype == RobotName.o20:
            from .hand.linkerforce_o20 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand], is_debug=mapper_debug)
        elif self.righthandtype == RobotName.o30:
            from .hand.linkerforce_o30 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand], is_debug=mapper_debug)
        elif self.righthandtype == RobotName.o30i:
            from .hand.linkerforce_o30i import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand], is_debug=mapper_debug)
        elif self.righthandtype == RobotName.l25 \
            or self.righthandtype == RobotName.g20:
            from .hand.linkerforce_g20 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand], is_debug=mapper_debug)
        elif self.righthandtype == RobotName.l20:
            from .hand.linkerforce_l20 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand], is_debug=mapper_debug)
        elif self.righthandtype == RobotName.l10 \
            or self.righthandtype == RobotName.l10v7 \
            or self.righthandtype == RobotName.l20lite:
            from .hand.linkerforce_l10 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand], is_debug=mapper_debug)
        else:
            print("未正确定义机械左手对象，请检查支持清单列表!")

        # 根据左手类型初始化
        if self.lefthandtype == RobotName.o7 \
            or self.lefthandtype == RobotName.l7 \
            or self.lefthandtype == RobotName.o7v1 \
            or self.lefthandtype == RobotName.o7v3:
            from .hand.linkerforce_l7 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand], is_debug=mapper_debug)
        elif self.lefthandtype == RobotName.o6:
            from .hand.linkerforce_o6 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand], is_debug=mapper_debug)
        elif self.lefthandtype == RobotName.l6:
            from .hand.linkerforce_l6 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand], is_debug=mapper_debug)
        elif self.lefthandtype == RobotName.o20:
            from .hand.linkerforce_o20 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand], is_debug=mapper_debug)
        elif self.lefthandtype == RobotName.o30:
            from .hand.linkerforce_o30 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand], is_debug=mapper_debug)
        elif self.lefthandtype == RobotName.o30i:
            from .hand.linkerforce_o30i import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand], is_debug=mapper_debug)
        elif self.lefthandtype == RobotName.l25 \
            or self.lefthandtype == RobotName.g20:
            from .hand.linkerforce_g20 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand], is_debug=mapper_debug)
        elif self.lefthandtype == RobotName.l20:
            from .hand.linkerforce_l20 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand], is_debug=mapper_debug)
        elif self.lefthandtype == RobotName.l10 \
            or self.lefthandtype == RobotName.l10v7 \
            or self.lefthandtype == RobotName.l20lite:
            from .hand.linkerforce_l10 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand], is_debug=mapper_debug)
        else:
            print("未正确定义机械右手对象，请检查支持清单列表!")

        self.node.get_logger().info(f"[机械手] 左手型号: {self.lefthandtype.name}, 右手型号: {self.righthandtype.name}")

        # ROS2 发布器
        self.publisher_r = self.node.create_publisher(
            JointState,
            '/cb_right_hand_control_cmd',
            self.handcore.hand_numjoints_r)
            
        self.publisher_l = self.node.create_publisher(
            JointState,
            '/cb_left_hand_control_cmd',
            self.handcore.hand_numjoints_l)

        self.publisher_angle_r = self.node.create_publisher(
            JointState,
            '/cb_right_hand_control_angle_cmd',
            self.handcore.hand_numjoints_r)
            
        self.publisher_angle_l = self.node.create_publisher(
            JointState,
            '/cb_left_hand_control_angle_cmd',
            self.handcore.hand_numjoints_l)

        # 创建订阅者，订阅/cb_left_hand_matrix_touch话题
        self.left_touch_subscription = self.node.create_subscription(
            String,
            '/cb_left_hand_matrix_touch',
            self.touch_left_callback,
            10  # QoS 队列深度
        )
        self.right_touch_subscription = self.node.create_subscription(
            String,
            '/cb_right_hand_matrix_touch', 
            self.touch_right_callback,
            10  # QoS 队列深度
        )


        if self.isdebugpub:
            # # ROS1 发布器，触感矩阵转换相关
            # self.publisher_hand_matrix2int_r = self.node.create_publisher(
            #     '/cb_right_hand_matrix2int', 
            #     Int32MultiArray, 
            #     self.handcore.hand_numjoints_r)
            
            # self.publisher_hand_matrix2int_l = self.node.create_publisher(
            #     '/cb_left_hand_matrix2int', 
            #     Int32MultiArray, 
            #     self.handcore.hand_numjoints_l)


            self.publisher_hand_debugdata_r = self.node.create_publisher(
                Float32MultiArray, 
                '/cb_right_hand_debug', 
                self.handcore.hand_numjoints_r)


            self.publisher_hand_debugdata_l = self.node.create_publisher(
                Float32MultiArray, 
                '/cb_left_hand_debug', 
                self.handcore.hand_numjoints_l)

        # 初始化统计结果
        self.results = {
            'left':{},
            'right':{}
        } 
        self.leftforcesendcount = -1
        self.rightforcesendcount = -1

        # 状态变量
        self.pubprintcount = 0
        self.force_reader_left = None
        self.force_reader_right = None
        self.calibration = calibration
        self.leftport = None
        self.leftbaudrate = None
        self.rightport = None
        self.rightbaudrate = None
        
        # 强制手套数据源 (none/open/fist/opose)
        self.force_glove_pose = None
        self.calibration_cache = None  # 缓存标定数据

        # 力数据线程锁
        self.forcelock = threading.Lock()

        # 自动标定相关变量
        self.calibration_data_left = []
        self.calibration_data_right = []
        self.calibration_in_progress = False
        
        # ========== 调试：映射层跳变检测 ==========
        self.debug_enabled = True  # 设为 False 关闭调试
        self.debug_motor_jump_threshold = 20  # 电机值跳变阈值
        self.debug_raw_jump_threshold = 0.35  # 手套原始弧度跳变阈值
        self.debug_last_motor_l = [255] * 6
        self.debug_last_motor_r = [255] * 6
        self.debug_last_raw_l = [0.0] * 21
        self.debug_last_raw_r = [0.0] * 21
        self.debug_last_o6_pinky_raw_r = None



    def touch_left_callback(self, msg):
        self.process_touch_data(msg.data,'left')   
           
    def touch_right_callback(self, msg):
        self.process_touch_data(msg.data,'right')

    def process_touch_data(self, json_str, hand_type):
        with self.forcelock:
            try:
                data = json.loads(json_str)    
                self.results[hand_type] = {}
                # 处理每个手指的矩阵
                for finger in ['thumb_matrix', 'index_matrix', 'middle_matrix', 'ring_matrix', 'little_matrix']:
                    matrix = np.array(data[finger])
                    # 计算接触面积（非零元素数量）
                    contact_area = np.count_nonzero(matrix)         
                    # 计算总接触力
                    total_force = np.sum(matrix)         
                    # 计算平均接触力（避免除以零）
                    avg_force = total_force / contact_area if contact_area > 0 else 0
                    max_force = np.max(matrix) * 4 if contact_area > 0 else 0
                    if max_force > 500:
                        max_force = 500
                    self.results[hand_type][finger] = {
                        'contact_area': contact_area,
                        'total_force': total_force,
                        'avg_force': avg_force,
                        'max_force': max_force
                    }
                
            except Exception as e:
                self.node.get_logger().error("Error processing touch data: %s" % str(e))
                return None

    def _capture_o6_right_pinky_raw_jump(self, right_positions, left_valid=False, right_valid=True):
        if not getattr(self, "debug_enabled", False) or getattr(self.righthandtype, "name", self.righthandtype) != "o6":
            return None
        if not right_valid or right_positions is None or len(right_positions) <= 20:
            return None

        def fmt(values):
            return "[" + ", ".join(f"{value:.6f}" for value in values) + "]"

        indices = [18, 19, 20]
        current = [float(right_positions[index]) for index in indices]
        previous = self.debug_last_o6_pinky_raw_r
        self.debug_last_o6_pinky_raw_r = current

        if previous is None:
            deltas = [0.0] * len(current)
        else:
            deltas = [current_value - previous_value for current_value, previous_value in zip(current, previous)]
        max_local_index = max(range(len(deltas)), key=lambda index: abs(deltas[index]))
        max_delta = deltas[max_local_index]
        threshold = getattr(self, "debug_raw_jump_threshold", 0.35)
        is_jump = previous is not None and abs(max_delta) >= threshold

        glove_version = getattr(getattr(self, "righthand", None), "glove_version", None)
        trace = {
            "raw_indices": indices,
            "raw_previous": previous,
            "raw_current": current,
            "raw_delta": deltas,
            "max_index": indices[max_local_index],
            "max_delta": max_delta,
            "is_jump": is_jump,
            "threshold": threshold,
            "left_valid": left_valid,
            "right_valid": right_valid,
            "glove_version": glove_version,
        }
        self.node.get_logger().warn(
            "[O6右手小指根部原始数据跟踪] "
            f"is_jump={is_jump}, "
            f"raw_idx={indices}, "
            f"raw_prev={fmt(previous) if previous is not None else 'None'}, "
            f"raw_curr={fmt(current)}, "
            f"raw_delta={fmt(deltas)}, "
            f"max_idx={indices[max_local_index]}, "
            f"max_delta={max_delta:.6f}, "
            f"threshold={threshold}, "
            f"left_valid={left_valid}, right_valid={right_valid}, "
            f"glove_version={glove_version}"
        )
        return trace

    def _trace_o6_right_publish_before_topic(self, raw_jump_trace, msg_r):
        if not raw_jump_trace:
            return

        def fmt(values):
            return "[" + ", ".join(f"{float(value):.6f}" for value in values) + "]"

        position = list(getattr(msg_r, "position", []))
        velocity = list(getattr(msg_r, "velocity", []))
        pinky_motor = position[5] if len(position) > 5 else None
        pinky_velocity = velocity[5] if len(velocity) > 5 else None
        pinky_motor_text = "None" if pinky_motor is None else f"{float(pinky_motor):.6f}"
        pinky_velocity_text = "None" if pinky_velocity is None else f"{float(pinky_velocity):.6f}"

        self.node.get_logger().warn(
            "[O6右手小指根部发布前跟踪] "
            f"pub_count={getattr(self, 'pubprintcount', None)}, "
            f"is_jump={raw_jump_trace.get('is_jump')}, "
            f"raw_idx={raw_jump_trace['raw_indices']}, "
            f"raw_delta={fmt(raw_jump_trace['raw_delta'])}, "
            f"max_idx={raw_jump_trace['max_index']}, "
            f"max_delta={raw_jump_trace['max_delta']:.6f}, "
            f"publish_position={fmt(position)}, "
            f"publish_velocity={fmt(velocity)}, "
            f"pinky_motor={pinky_motor_text}, "
            f"pinky_velocity={pinky_velocity_text}"
        )

    def linkerforce_init(self):
        # 从配置读取串口参数
        serial_config = self.baseconfig.get('serial', {})
        baudrates = serial_config.get('baudrates', [2000000, 1000000, 921600, 460800])
        exclude_ports = serial_config.get('exclude_ports', [])
        serial_debug = serial_config.get('serial_debug', False)
        config_auto_scan = serial_config.get('auto_scan', False)
        
        # 自动扫描开关：命令行参数 > 配置文件 > 默认false
        auto_scan = self.cmd_auto_scan if self.cmd_auto_scan is not None else config_auto_scan
        
        saved_left = serial_config.get('left', {})
        saved_right = serial_config.get('right', {})
        
        # 确定候选端口列表
        if self.cmd_ports:
            candidate_ports = self.cmd_ports
            self.node.get_logger().info(f"使用命令行候选串口: {candidate_ports}")
        elif auto_scan:
            ports = serial.tools.list_ports.comports()
            candidate_ports = [
                port.device
                for port in ports
                if (
                    port.device not in exclude_ports
                    and (
                        port.device.startswith("/dev/ttyUSB")
                        or port.device.startswith("/dev/ttyACM")
                        or port.device.startswith("/dev/ttyXRUSB")
                        or port.device.startswith("/dev/ttyOBC")
                    )
                )
            ]
            self.node.get_logger().info(f"自动扫描候选串口: {candidate_ports}")
        else:
            candidate_ports = None
            self.node.get_logger().info(f"使用配置文件串口: 左手={saved_left.get('port')}, 右手={saved_right.get('port')}")
        
        # 确定波特率
        if self.cmd_baudrate:
            baudrates = [self.cmd_baudrate]
        
        self.node.get_logger().info(f"波特率组合: {baudrates}, 自动扫描={auto_scan}, 调试={serial_debug}")
        
        # 日志回调函数
        def serial_logger(level, msg):
            if level == 'error':
                self.node.get_logger().error(msg)
            elif level == 'warn':
                self.node.get_logger().warn(msg)
            elif level == 'debug':
                self.node.get_logger().debug(msg)
            else:
                self.node.get_logger().info(msg)
        
        # 如果提供了候选端口列表，从中自动检测左右手
        if candidate_ports:
            left_found, right_found = self._init_from_candidates(
                candidate_ports, baudrates, exclude_ports, serial_debug, serial_logger
            )
        else:
            # 使用配置文件的预设端口
            self.force_reader_left = ForceSerialReader(
                HandType.left,
                excludelist=exclude_ports,
                baudrates=baudrates,
                isdebug=serial_debug,
                logger=serial_logger
            )
            left_found = self._init_hand(
                self.force_reader_left, 
                'Left', 
                saved_left.get('port'), 
                saved_left.get('baudrate'),
                '左手',
                auto_scan
            )
            
            exclude_right = exclude_ports + ([self.leftport] if left_found else [])
            self.force_reader_right = ForceSerialReader(
                HandType.right,
                excludelist=exclude_right,
                baudrates=baudrates,
                isdebug=serial_debug,
                logger=serial_logger
            )
            right_found = self._init_hand(
                self.force_reader_right, 
                'Right', 
                saved_right.get('port') if saved_right.get('port') not in exclude_right else None,
                saved_right.get('baudrate'),
                '右手',
                auto_scan
            )
        
        # 保存检测到的串口配置
        if left_found or right_found:
            self._save_serial_to_config(left_found, right_found)
        
        time.sleep(1)

        # 标定流程
        if self.calibration is True:
            self.calibration = "auto_calibrate"
            self.node.get_logger().info("强制标定模式：将进行自动标定")
        else:
            if self._load_from_tmp() is True:
                self.node.get_logger().info("已加载缓存标定数据，跳过标定流程")
                self.calibration = -1
                self._initialize_ready_mappers()
            else:
                self.calibration = "auto_calibrate"
                self.node.get_logger().info("未找到有效缓存，将进行自动标定")
    
    def _init_from_candidates(self, candidate_ports, baudrates, exclude_ports, serial_debug, serial_logger):
        """从候选端口列表中自动检测并初始化左右手"""
        left_found = False
        right_found = False
        detected_ports = {}
        
        for port in candidate_ports:
            if port in exclude_ports:
                continue
            
            # 检查端口是否存在
            if not os.path.exists(port):
                self.node.get_logger().warn(f"端口不存在: {port}")
                continue
            
            self.node.get_logger().info(f"检测候选端口: {port}")
            
            temp_reader = ForceSerialReader(
                HandType.left,
                excludelist=[],
                baudrates=baudrates,
                isdebug=serial_debug,
                logger=serial_logger
            )
            
            detected = False
            for baudrate in baudrates:
                try:
                    if temp_reader.openserial(port=port, baudrate=baudrate):
                        temp_reader.start()
                        time.sleep(0.1)
                        temp_reader.serial_port.write(temp_reader.pack_01_data())
                        
                        for _ in range(10):
                            time.sleep(0.1)
                            if temp_reader.handtype:
                                detected_ports[port] = (temp_reader.handtype, baudrate, temp_reader.version)
                                self.node.get_logger().info(f"检测到 {port}: {temp_reader.handtype} @ {baudrate}")
                                detected = True
                                break
                        
                        if detected:
                            break
                except Exception as e:
                    self.node.get_logger().debug(f"端口 {port} @ {baudrate} 检测失败: {e}")
                finally:
                    temp_reader.stop()
            
            if not detected:
                self.node.get_logger().warn(f"端口 {port} 未能识别设备类型")
        
        # 根据检测到的手型初始化
        for port, (handtype, baudrate, version) in detected_ports.items():
            if handtype == 'Left' and not left_found:
                self.force_reader_left = ForceSerialReader(
                    HandType.left,
                    excludelist=exclude_ports,
                    baudrates=baudrates,
                    isdebug=serial_debug,
                    logger=serial_logger
                )
                if self.force_reader_left.openserial(port=port, baudrate=baudrate):
                    self.force_reader_left.handtype = handtype
                    self.force_reader_left.version = version
                    self.force_reader_left.start()
                    self.force_reader_left.serial_port.write(self.force_reader_left.pack_01_data())
                    self.leftport = port
                    self.leftbaudrate = baudrate
                    left_found = True
                    self.node.get_logger().info(f"左手已连接: {port} @ {baudrate}, 版本 {version}")
                    if version:
                        self.lefthand.set_glove_version(version)
                        self.node.get_logger().info(f"[手套版本] 左手: v{version.split('.')[0]} ({version})")
                    
            elif handtype == 'Right' and not right_found:
                self.force_reader_right = ForceSerialReader(
                    HandType.right,
                    excludelist=exclude_ports + ([self.leftport] if left_found else []),
                    baudrates=baudrates,
                    isdebug=serial_debug,
                    logger=serial_logger
                )
                if self.force_reader_right.openserial(port=port, baudrate=baudrate):
                    self.force_reader_right.handtype = handtype
                    self.force_reader_right.version = version
                    self.force_reader_right.start()
                    self.force_reader_right.serial_port.write(self.force_reader_right.pack_01_data())
                    self.rightport = port
                    self.rightbaudrate = baudrate
                    right_found = True
                    self.node.get_logger().info(f"右手已连接: {port} @ {baudrate}, 版本 {version}")
                    if version:
                        self.righthand.set_glove_version(version)
                        self.node.get_logger().info(f"[手套版本] 右手: v{version.split('.')[0]} ({version})")
        
        # 初始化未找到的 reader（占位）
        if not left_found:
            self.node.get_logger().error("未找到左手力反馈手套")
            self.force_reader_left = None
        if not right_found:
            self.node.get_logger().error("未找到右手力反馈手套")
            self.force_reader_right = None
        
        return left_found, right_found
    
    def _init_hand(self, reader, hand_type, saved_port, saved_baudrate, hand_name, auto_scan=False):
        """初始化单个手的串口连接"""
        found = False
        
        # 尝试预设串口
        if saved_port and saved_baudrate:
            self.node.get_logger().info(f"尝试预设{hand_name}串口: {saved_port}")
            try:
                if reader.openserial(port=saved_port, baudrate=int(saved_baudrate)):
                    time.sleep(0.3)
                    reader.start()
                    time.sleep(0.3)
                    reader.serial_port.write(reader.pack_01_data())
                    time.sleep(0.3)
                    if reader.handtype == hand_type:
                        self.node.get_logger().info(f"预设{hand_name}串口有效, 版本{reader.version}")
                        if hand_type == 'Left':
                            self.leftport = saved_port
                            self.leftbaudrate = int(saved_baudrate)
                            if reader.version:
                                self.lefthand.set_glove_version(reader.version)
                                self.node.get_logger().info(f"[手套版本] 左手: v{reader.version.split('.')[0]} ({reader.version})")
                        else:
                            self.rightport = saved_port
                            self.rightbaudrate = int(saved_baudrate)
                            if reader.version:
                                self.righthand.set_glove_version(reader.version)
                                self.node.get_logger().info(f"[手套版本] 右手: v{reader.version.split('.')[0]} ({reader.version})")
                        found = True
                    else:
                        reader.stop()
                        self.node.get_logger().warn(f"预设{hand_name}串口类型不匹配")
            except Exception as e:
                self.node.get_logger().warn(f"预设{hand_name}串口无效: {e}")
        
        # 预设无效，根据 auto_scan 决定是否搜索设备
        if not found:
            if auto_scan:
                self.node.get_logger().info(f"搜索{hand_name}力反馈手套...")
                port, baudrate, errorcode = reader.find_valid_ports(timeout=0.001)
                if port:
                    if reader.openserial(port=port, baudrate=baudrate):
                        time.sleep(0.3)
                        reader.start()
                        time.sleep(0.3)
                        reader.serial_port.write(reader.pack_01_data())
                        time.sleep(0.3)
                        if reader.handtype == hand_type:
                            self.node.get_logger().info(f"已搜索到{hand_name}力反馈手套, 版本{reader.version}")
                            if hand_type == 'Left':
                                self.leftport = port
                                self.leftbaudrate = baudrate
                                if reader.version:
                                    self.lefthand.set_glove_version(reader.version)
                                    self.node.get_logger().info(f"[手套版本] 左手: v{reader.version.split('.')[0]} ({reader.version})")
                            else:
                                self.rightport = port
                                self.rightbaudrate = baudrate
                                if reader.version:
                                    self.righthand.set_glove_version(reader.version)
                                    self.node.get_logger().info(f"[手套版本] 右手: v{reader.version.split('.')[0]} ({reader.version})")
                            found = True
                        else:
                            self.node.get_logger().warn(f"{hand_name}无法正常识别")
                else:
                    self.node.get_logger().warn(f"未搜索到{hand_name}力反馈手套")
            else:
                self.node.get_logger().warn(f"{hand_name}串口未连接（自动扫描已禁用）")
        
        return found
    
    def _save_serial_to_config(self, left_found, right_found):
        """保存检测到的串口配置到 base_config.yml"""
        config_path = Path(__file__).parent.parent.parent / "config" / "base_config.yml"
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            if 'serial' not in config:
                config['serial'] = {}
            if 'left' not in config['serial']:
                config['serial']['left'] = {}
            if 'right' not in config['serial']:
                config['serial']['right'] = {}
            
            if left_found:
                config['serial']['left']['port'] = self.leftport
                config['serial']['left']['baudrate'] = self.leftbaudrate
            if right_found:
                config['serial']['right']['port'] = self.rightport
                config['serial']['right']['baudrate'] = self.rightbaudrate
            
            with open(config_path, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)
            
            self.node.get_logger().info(f"串口配置已保存: 左手={self.leftport}, 右手={self.rightport}")
        except Exception as e:
            self.node.get_logger().error(f"保存串口配置失败: {e}")
          
    def process_callback(self):
        if not self.runing:
            return

        self.pubprintcount += 1
        
        left_valid = self.force_reader_left and self.force_reader_left.handtype == 'Left'
        right_valid = self.force_reader_right and self.force_reader_right.handtype == 'Right'
        
        warn_interval = 150
        
        if not left_valid and not right_valid and not self.force_glove_pose:
            if self.pubprintcount % warn_interval == 1:
                self.node.get_logger().warn("设备未连接: 左手套(未连接), 右手套(未连接)")
            return
        
        if left_valid and not right_valid and not self.force_glove_pose:
            if self.pubprintcount % warn_interval == 1:
                self.node.get_logger().warn("设备未连接: 右手套(未连接)")
        elif right_valid and not left_valid and not self.force_glove_pose:
            if self.pubprintcount % warn_interval == 1:
                self.node.get_logger().warn("设备未连接: 左手套(未连接)")
        

        if left_valid or self.force_glove_pose:
            if self.force_glove_pose:
                left_positions = self._get_forced_positions(self.force_glove_pose, 'left')
                if left_positions is None:
                    if left_valid:
                        left_positions = copy.deepcopy(self.force_reader_left.poslist)
                        self.node.get_logger().warn(f"强制姿态 {self.force_glove_pose} 数据不存在，使用实际数据")
                    else:
                        self.node.get_logger().warn(f"强制姿态 {self.force_glove_pose} 数据不存在且无设备")
                        return
            else:
                left_positions = copy.deepcopy(self.force_reader_left.poslist)
            
            self.lefthand.joint_update(left_positions)
            self.lefthand.speed_update()
            if left_valid:
                with self.forcelock:
                    if self.results['left']:
                        self.force_reader_left.forcelist = [
                                self.results['left']['thumb_matrix']['max_force'],
                                self.results['left']['index_matrix']['max_force'],
                                self.results['left']['middle_matrix']['max_force'],
                                self.results['left']['ring_matrix']['max_force'],
                                self.results['left']['little_matrix']['max_force']
                            ]
                        self.force_reader_left.serial_port.write(self.force_reader_left.pack_04_data())
            if self.lefthandpubprint and self.pubprintcount % 5 == 0:
                print(f"左手位置: {self.lefthand.g_jointpositions}")
            msg_l = JointState()
            msg_l.header.stamp = self.node.get_clock().now().to_msg()
            msg_l.name = [f'joint{i + 1}' for i in range(len(self.lefthand.g_jointpositions))]
            msg_l.position = [float(num) for num in self.lefthand.g_jointpositions]
            self.publisher_l.publish(msg_l)

            if self.isdebugpub:
                msg_debug_l = Float32MultiArray()
                msg_debug_l.data = [float(num) for num in self.lefthand.multi_state_mapper.debug_value]
                self.publisher_hand_debugdata_l.publish(msg_debug_l)
                
        if right_valid or self.force_glove_pose:
            if self.force_glove_pose:
                right_positions = self._get_forced_positions(self.force_glove_pose, 'right')
                if right_positions is None:
                    if right_valid:
                        right_positions = copy.deepcopy(self.force_reader_right.poslist)
                        self.node.get_logger().warn(f"强制姿态 {self.force_glove_pose} 数据不存在，使用实际数据")
                    else:
                        self.node.get_logger().warn(f"强制姿态 {self.force_glove_pose} 数据不存在且无设备")
                        return
            else:
                right_positions = copy.deepcopy(self.force_reader_right.poslist)

            o6_pinky_raw_jump_trace = self._capture_o6_right_pinky_raw_jump(
                right_positions,
                left_valid=left_valid,
                right_valid=right_valid,
            )
            self.righthand.joint_update(right_positions)
            self.righthand.speed_update()
            if right_valid:
                with self.forcelock:
                    if self.results['right']:
                        self.force_reader_right.forcelist = [
                                self.results['right']['thumb_matrix']['max_force'],
                                self.results['right']['index_matrix']['max_force'],
                                self.results['right']['middle_matrix']['max_force'],
                                self.results['right']['ring_matrix']['max_force'],
                                self.results['right']['little_matrix']['max_force']
                            ]
                        self.force_reader_right.serial_port.write(self.force_reader_right.pack_04_data())
            if self.righthandpubprint and self.pubprintcount % 5 == 0:
                print(f"右手位置: {self.righthand.g_jointpositions}")

            msg_r = JointState()
            msg_r.header.stamp = self.node.get_clock().now().to_msg()
            msg_r.name = [f'joint{i + 1}' for i in range(len(self.righthand.g_jointpositions))]
            msg_r.position = [float(num) for num in self.righthand.g_jointpositions]
            msg_r.velocity = [float(num) for num in self.righthand.g_jointvelocity]
            self._trace_o6_right_publish_before_topic(o6_pinky_raw_jump_trace, msg_r)
            self.publisher_r.publish(msg_r)

            if self.isdebugpub:
                msg_debug_r = Float32MultiArray()
                msg_debug_r.data = [float(num) for num in self.righthand.multi_state_mapper.debug_value]

                self.publisher_hand_debugdata_r.publish(msg_debug_r)
        
        self.pubprintcount += 1

    def _calculate_weighted_average(self, data_list):
        """
        计算加权平均值，后面的数据权重更高
        
        Args:
            data_list: 包含多帧数据的列表，每帧是21个关节值的列表
            
        Returns:
            加权平均后的21个关节值列表
        """
        if not data_list:
            return [0.0] * 21
        
        n = len(data_list)
        if n == 1:
            return data_list[0]
        
        # 生成权重：后面的数据权重更高
        weights = np.array([i + 1 for i in range(n)], dtype=float)
        weights = weights / weights.sum()
        
        # 转换为numpy数组进行计算
        data_array = np.array(data_list)
        
        # 加权平均
        weighted_avg = np.average(data_array, axis=0, weights=weights)
        
        return weighted_avg.tolist()

    def _check_stability(self, window_size=20, threshold=0.05):
        """
        检测手势稳定性
        
        Args:
            window_size: 检测窗口大小（帧数）
            threshold: 稳定性阈值（关节角度方差）
            
        Returns:
            (is_stable, variance): 是否稳定，当前方差
        """
        if len(self.calibration_data_left) < window_size:
            return False, 1.0
        
        recent_left = self.calibration_data_left[-window_size:]
        recent_right = self.calibration_data_right[-window_size:]
        
        var_left = np.var(recent_left, axis=0).mean()
        var_right = np.var(recent_right, axis=0).mean()
        
        is_stable = var_left < threshold and var_right < threshold
        return is_stable, max(var_left, var_right)

    def _is_reader_connected(self, hand):
        reader = getattr(self, f"force_reader_{hand}", None)
        expected_handtype = "Left" if hand == "left" else "Right"
        return reader is not None and getattr(reader, "handtype", None) == expected_handtype

    def _has_complete_calibration(self, hand):
        return (
            getattr(hand, "calibrationoriginal", None) is not None
            and getattr(hand, "calibrationopose", None) is not None
            and getattr(hand, "calibrationfistpose", None) is not None
        )

    def _initialize_ready_mappers(self):
        if self._has_complete_calibration(self.righthand):
            self.righthand.initialize_mapper()
        else:
            self.node.get_logger().warn("右手标定数据不完整，跳过右手映射器初始化")

        if self._has_complete_calibration(self.lefthand):
            self.lefthand.initialize_mapper()
        else:
            self.node.get_logger().warn("左手标定数据不完整，跳过左手映射器初始化")
    
    def _calibration_with_progress(self, stability_window=30, stability_threshold=0.03):
        """
        带稳定性检测的标定数据采集
        
        Args:
            stability_window: 稳定性检测窗口（帧数）
            stability_threshold: 稳定性阈值
        """
        self.calibration_data_left = []
        self.calibration_data_right = []

        left_valid = self._is_reader_connected("left")
        right_valid = self._is_reader_connected("right")

        if not left_valid and not right_valid:
            print(f"{Fore.RED}没有可用的手套数据，无法采集标定样本{Fore.RESET}")
            return

        left_reader = self.force_reader_left if left_valid else None
        right_reader = self.force_reader_right if right_valid else None

        temp_buffer_left = []
        temp_buffer_right = []
        
        stability_samples = []
        stable_start = None
        collected_duration = 0
        target_stable_duration = 5.0
        
        with tqdm(total=100, desc=f"{Fore.CYAN}标定进度{Fore.RESET}",
                  bar_format="{l_bar}{bar}| {n_fmt}/{total_fmt} [{postfix}]",
                  postfix="") as pbar:
            while collected_duration < target_stable_duration:
                left_pos = copy.deepcopy(left_reader.poslist) if left_valid else None
                right_pos = copy.deepcopy(right_reader.poslist) if right_valid else None

                if left_valid:
                    temp_buffer_left.append(left_pos)
                if right_valid:
                    temp_buffer_right.append(right_pos)

                if left_valid and len(temp_buffer_left) > stability_window:
                    temp_buffer_left.pop(0)
                if right_valid and len(temp_buffer_right) > stability_window:
                    temp_buffer_right.pop(0)

                ready_counts = []
                if left_valid:
                    ready_counts.append(len(temp_buffer_left))
                if right_valid:
                    ready_counts.append(len(temp_buffer_right))

                if min(ready_counts) < stability_window:
                    pbar.set_postfix_str(f"{Fore.YELLOW}等待数据 {min(ready_counts)}/{stability_window}{Fore.RESET}")
                    pbar.refresh()
                    time.sleep(1.0 / 30)
                    continue

                variances = []
                drifts = []
                if left_valid:
                    variances.append(np.var(temp_buffer_left, axis=0).mean())
                    drifts.append(np.abs(np.array(temp_buffer_left[-1]) - np.array(temp_buffer_left[0])).mean())
                if right_valid:
                    variances.append(np.var(temp_buffer_right, axis=0).mean())
                    drifts.append(np.abs(np.array(temp_buffer_right[-1]) - np.array(temp_buffer_right[0])).mean())

                variance = max(variances)
                drift = max(drifts)
                
                is_stable = (variance < stability_threshold and drift < stability_threshold)
                
                if is_stable:
                    if stable_start is None:
                        stable_start = time.time()
                    
                    stability_samples.append((left_pos, right_pos))
                    collected_duration = time.time() - stable_start
                    
                    progress = min(100, int(collected_duration / target_stable_duration * 100))
                    pbar.n = progress
                    pbar.last_print_n = progress
                    pbar.set_postfix_str(f"{Fore.GREEN}稳定 {collected_duration:.1f}s var={variance:.3f} drift={drift:.3f}{Fore.RESET}")
                else:
                    stable_start = None
                    stability_samples = []
                    collected_duration = 0
                    pbar.n = 0
                    pbar.last_print_n = 0
                    pbar.set_postfix_str(f"{Fore.YELLOW}等待稳定 var={variance:.3f} drift={drift:.3f}{Fore.RESET}")
                
                pbar.refresh()
                time.sleep(1.0 / 30)
        
        if len(stability_samples) > 0:
            self.calibration_data_left = [s[0] for s in stability_samples if s[0] is not None]
            self.calibration_data_right = [s[1] for s in stability_samples if s[1] is not None]
            print(f"{Fore.GREEN}采集完成，有效样本: {len(stability_samples)} 帧{Fore.RESET}")

    def run_calibration(self):
        """
        执行自动标定流程
        """
        # 检查手套连接状态
        left_valid = self._is_reader_connected("left")
        right_valid = self._is_reader_connected("right")
        
        if not left_valid and not right_valid:
            print(f"\n{Fore.RED}【标定失败】左右手套均未连接，请检查设备连接后重试{Fore.RESET}\n")
            self.calibration_in_progress = False
            return False
        
        self.calibration_in_progress = True
        
        # ===== 标定顺序 =====
        # show_fist_calibration=True : fist -> opose -> open
        # show_fist_calibration=False: opose -> open, fist 由延伸计算
        self.calibration_data_left = []
        self.calibration_data_right = []
        total_steps = 2 if not self.show_fist_calibration else 3
        if self.show_fist_calibration:
            # ===== 第一步：握拳标定 (对应0) =====
            self.calibration_data_left = []
            self.calibration_data_right = []
            print(f"\n{Fore.YELLOW}{'='*50}{Fore.RESET}")
            print(f"{Fore.YELLOW}【标定 1/{total_steps}】请握紧拳头 (对应电机值0){Fore.RESET}")
            print(f"{Fore.YELLOW}{'='*50}{Fore.RESET}\n")

            self._calibration_with_progress(10)

            fist_ok = True
            if left_valid:
                if len(self.calibration_data_left) > 0:
                    self.lefthand.calibrationfistpose = self._calculate_weighted_average(self.calibration_data_left)
                else:
                    fist_ok = False
            if right_valid:
                if len(self.calibration_data_right) > 0:
                    self.righthand.calibrationfistpose = self._calculate_weighted_average(self.calibration_data_right)
                else:
                    fist_ok = False

            if not fist_ok:
                print(f"\n{Fore.RED}【标定失败】握拳数据采集失败{Fore.RESET}\n")
                self.calibration_in_progress = False
                return False

            # ===== 第二步：O型标定 (对应中间值) =====
            self.calibration_data_left = []
            self.calibration_data_right = []
            print(f"\n{Fore.MAGENTA}{'='*50}{Fore.RESET}")
            print(f"{Fore.MAGENTA}【标定 2/{total_steps}】请保持O型手势 (对应电机中间值){Fore.RESET}")
            print(f"{Fore.MAGENTA}{'='*50}{Fore.RESET}\n")

            self._calibration_with_progress(10)

            opose_ok = True
            if left_valid:
                if len(self.calibration_data_left) > 0:
                    self.lefthand.calibrationopose = self._calculate_weighted_average(self.calibration_data_left)
                else:
                    opose_ok = False
            if right_valid:
                if len(self.calibration_data_right) > 0:
                    self.righthand.calibrationopose = self._calculate_weighted_average(self.calibration_data_right)
                else:
                    opose_ok = False

            if not opose_ok:
                print(f"\n{Fore.RED}【标定失败】O型手势数据采集失败{Fore.RESET}\n")
                self.calibration_in_progress = False
                return False

            # ===== 第三步：五指张开标定 (对应255) =====
            self.calibration_data_left = []
            self.calibration_data_right = []
            print(f"\n{Fore.GREEN}{'='*50}{Fore.RESET}")
            print(f"{Fore.GREEN}【标定 3/{total_steps}】请保持五指张开姿势 (对应电机值255){Fore.RESET}")
            print(f"{Fore.GREEN}{'='*50}{Fore.RESET}\n")

            self._calibration_with_progress(10)

            open_ok = True
            if left_valid:
                if len(self.calibration_data_left) > 0:
                    self.lefthand.calibrationoriginal = self._calculate_weighted_average(self.calibration_data_left)
                else:
                    open_ok = False
            if right_valid:
                if len(self.calibration_data_right) > 0:
                    self.righthand.calibrationoriginal = self._calculate_weighted_average(self.calibration_data_right)
                else:
                    open_ok = False

            if not open_ok:
                print(f"\n{Fore.RED}【标定失败】五指张开数据采集失败{Fore.RESET}\n")
                self.calibration_in_progress = False
                return False
        else:
            # ===== 第一步：O型标定 (对应中间值) =====
            self.calibration_data_left = []
            self.calibration_data_right = []
            print(f"\n{Fore.MAGENTA}{'='*50}{Fore.RESET}")
            print(f"{Fore.MAGENTA}【标定 1/{total_steps}】请保持O型手势 (对应电机中间值){Fore.RESET}")
            print(f"{Fore.MAGENTA}{'='*50}{Fore.RESET}\n")

            self._calibration_with_progress(10)

            opose_ok = True
            if left_valid:
                if len(self.calibration_data_left) > 0:
                    self.lefthand.calibrationopose = self._calculate_weighted_average(self.calibration_data_left)
                else:
                    opose_ok = False
            if right_valid:
                if len(self.calibration_data_right) > 0:
                    self.righthand.calibrationopose = self._calculate_weighted_average(self.calibration_data_right)
                else:
                    opose_ok = False

            if not opose_ok:
                print(f"\n{Fore.RED}【标定失败】O型手势数据采集失败{Fore.RESET}\n")
                self.calibration_in_progress = False
                return False

            # ===== 第二步：五指张开标定 (对应255) =====
            self.calibration_data_left = []
            self.calibration_data_right = []
            print(f"\n{Fore.GREEN}{'='*50}{Fore.RESET}")
            print(f"{Fore.GREEN}【标定 2/{total_steps}】请保持五指张开姿势 (对应电机值255){Fore.RESET}")
            print(f"{Fore.GREEN}{'='*50}{Fore.RESET}\n")

            self._calibration_with_progress(10)

            open_ok = True
            if left_valid:
                if len(self.calibration_data_left) > 0:
                    self.lefthand.calibrationoriginal = self._calculate_weighted_average(self.calibration_data_left)
                else:
                    open_ok = False
            if right_valid:
                if len(self.calibration_data_right) > 0:
                    self.righthand.calibrationoriginal = self._calculate_weighted_average(self.calibration_data_right)
                else:
                    open_ok = False

            if not open_ok:
                print(f"\n{Fore.RED}【标定失败】五指张开数据采集失败{Fore.RESET}\n")
                self.calibration_in_progress = False
                return False

            self._calculate_fist_from_extension()
        # ===== 保存标定数据 =====
        if self._save_to_tmp():
            print(f"\n{Fore.GREEN}{'='*50}{Fore.RESET}")
            print(f"{Fore.GREEN}【标定完成】{'三个' if self.show_fist_calibration else '两个'}姿势数据已保存{Fore.RESET}")
            print(f"{Fore.GREEN}{'='*50}{Fore.RESET}\n")
        
        self.calibration_in_progress = False
        self._initialize_ready_mappers()
        return True

    def _calculate_fist_from_extension(self):
        """
        从 original 和 opose 延伸计算 fist 值
        fist = opose + (opose - original) * extend_ratio
        """
        ratio = self.fist_extend_ratio

        calculated = []
        for hand_label, hand in (("左手", self.lefthand), ("右手", self.righthand)):
            original = hand.calibrationoriginal
            opose = hand.calibrationopose
            if original is None or opose is None:
                continue

            length = min(len(original), len(opose))
            hand.calibrationfistpose = [
                opose[i] + (opose[i] - original[i]) * ratio
                for i in range(length)
            ]
            calculated.append(hand_label)

        if calculated:
            self.node.get_logger().info(
                f"[自动计算] {','.join(calculated)}握拳值已从 O型延伸 {ratio*100:.0f}% 生成"
            )

    def _save_to_tmp(self):
        """
        保存标定数据到临时文件 (JSON格式，与ROS2一致)
        - jointangleoriginal: 五指张开 (对应电机255)
        - jointanglefist: 握拳 (对应电机0)
        """
        # v2.8.6 版本添加标定差异检测
        def calculate_vector_difference(vec1, vec2):
                """计算两个向量之间的差异"""
                if not vec1 or not vec2:
                    return 0
                
                # 确保向量长度一致
                min_len = min(len(vec1), len(vec2))
                if min_len == 0:
                    return 0
                
                # 计算欧几里得距离作为差异度量
                squared_diff = 0
                for i in range(min_len):
                    squared_diff += (vec1[i] - vec2[i]) ** 2
                return math.sqrt(squared_diff)
            
        # 检查右手标定数据的差异
        right_diff_original_fist = calculate_vector_difference(
            self.righthand.calibrationoriginal, 
            self.righthand.calibrationfistpose
        )
        
        # 检查左手标定数据的差异
        left_diff_original_fist = calculate_vector_difference(
            self.lefthand.calibrationoriginal,
            self.lefthand.calibrationfistpose
        )   
        
        # 设置阈值，根据实际情况调整
        # 这个阈值表示两个向量之间的最小可接受差异
        MIN_DIFFERENCE_THRESHOLD = 3.0
        
        right_connected = self._is_reader_connected("right")
        left_connected = self._is_reader_connected("left")

        # 检查右手数据是否有效
        right_valid = False
        if right_connected:
            # 检查original和fistpose的差异
            if right_diff_original_fist > MIN_DIFFERENCE_THRESHOLD:
                right_valid = True
            else:
                self.node.get_logger().error("右手张手和握拳标定数据差异过小，可能未正确标定")
        
        # 检查左手数据是否有效
        left_valid = False
        if left_connected:
            # 检查original和fistpose的差异
            if left_diff_original_fist > MIN_DIFFERENCE_THRESHOLD:
                left_valid = True
            else:
                self.node.get_logger().error("左手张手和握拳标定数据差异过小，可能未正确标定")
        
        # 如果没有有效的新数据，直接返回不保存
        if not right_valid and not left_valid:
            self.node.get_logger().error("没有有效的标定数据差异，取消保存")
            return False

        robot_name_r = getattr(self, "robot_name_r", None)
        robot_name_l = getattr(self, "robot_name_l", None)
        data = {
            "timestamp": datetime.now().isoformat(),
            "robotname_r": getattr(robot_name_r, "name", str(robot_name_r)),
            "robotname_l": getattr(robot_name_l, "name", str(robot_name_l)),
            "jointangleoriginal_r": self.righthand.calibrationoriginal,
            "jointangleoriginal_l": self.lefthand.calibrationoriginal,
            "jointanglefist_r": self.righthand.calibrationfistpose,
            "jointanglefist_l": self.lefthand.calibrationfistpose,
            "jointangleopose_r": self.righthand.calibrationopose,
            "jointangleopose_l": self.lefthand.calibrationopose
        }

        try:
            TMP_FILE_PATH.parent.mkdir(parents=True, exist_ok=True)
            if TMP_FILE_PATH.exists():
                content = TMP_FILE_PATH.read_text()
                historydata = json.loads(content)
                # 根据有效性更新数据
                if not right_valid:
                    data["jointangleoriginal_r"] = historydata["jointangleoriginal_r"]
                    data["jointanglefist_r"] = historydata["jointanglefist_r"]
                    data["jointangleopose_r"] = historydata["jointangleopose_r"]
                    # 同时更新 righthand 的标定属性
                    self.righthand.calibrationoriginal = historydata["jointangleoriginal_r"]
                    self.righthand.calibrationfistpose = historydata["jointanglefist_r"]
                    self.righthand.calibrationopose = historydata["jointangleopose_r"]
                    self.node.get_logger().warning("右手未能正确标定,采用上一次的标定内容。")
                if not left_valid:
                    data["jointangleoriginal_l"] = historydata["jointangleoriginal_l"]
                    data["jointanglefist_l"] = historydata["jointanglefist_l"]
                    data["jointangleopose_l"] = historydata["jointangleopose_l"]
                    # 同时更新 lefthand 的标定属性
                    self.lefthand.calibrationoriginal = historydata["jointangleoriginal_l"]
                    self.lefthand.calibrationfistpose = historydata["jointanglefist_l"]
                    self.lefthand.calibrationopose = historydata["jointangleopose_l"]
                    self.node.get_logger().warning("左手未能正确标定,采用上一次的标定内容。")
            json_str = json.dumps(data, indent=2)
            TMP_FILE_PATH.write_text(json_str)
            self.node.get_logger().info("标定数据保存成功")
            return True
        except Exception as e:
            self.node.get_logger().error(f"保存失败: {e}")
            return False

    def _load_from_tmp(self):
        """
        从临时文件读取数据 (JSON格式)
        如果文件不存在，自动从样本数据(YAML)加载并保存
        """
        data = None
        from_sample = False
        
        if TMP_FILE_PATH.exists():
            try:
                content = TMP_FILE_PATH.read_text()
                data = json.loads(content)
                self.node.get_logger().info("加载用户标定数据")
            except Exception as e:
                self.node.get_logger().error(f"读取标定数据失败: {e}")
        
        if data is None and SAMPLE_FILE_PATH.exists():
            try:
                with open(SAMPLE_FILE_PATH, 'r') as f:
                    data = yaml.safe_load(f)
                from_sample = True
                self.node.get_logger().info("首次使用，加载样本标定数据")
            except Exception as e:
                self.node.get_logger().error(f"读取样本数据失败: {e}")
        
        if data is None:
            self.node.get_logger().warn("标定数据不存在")
            return False

        if 'timestamp' not in data or not data['timestamp']:
            self.node.get_logger().warn("无效的时间戳...")
            return False

        try:
            saved_time = datetime.fromisoformat(str(data['timestamp']))
            current_time = datetime.now()
            time_diff = current_time - saved_time
            if time_diff > timedelta(days=30):
                self.node.get_logger().warn("标定数据已超过30天有效期，建议重新标定...")
        except:
            pass

        robot_name_r = getattr(self, "robot_name_r", None)
        robot_name_l = getattr(self, "robot_name_l", None)
        current_robot_r = getattr(robot_name_r, "name", str(robot_name_r)).lower()
        current_robot_l = getattr(robot_name_l, "name", str(robot_name_l)).lower()
        saved_robot_r = str(data.get("robotname_r", "") or "").strip().lower()
        saved_robot_l = str(data.get("robotname_l", "") or "").strip().lower()

        if not from_sample:
            if not saved_robot_r or not saved_robot_l:
                self.node.get_logger().warning("标定缓存缺少机器人型号信息，忽略旧缓存并重新标定")
                return False
            if saved_robot_r != current_robot_r or saved_robot_l != current_robot_l:
                self.node.get_logger().warning(
                    "标定缓存型号不匹配，忽略旧缓存并重新标定: "
                    f"tmp=({saved_robot_r}, {saved_robot_l}), "
                    f"current=({current_robot_r}, {current_robot_l})"
                )
                return False
        else:
            data["robotname_r"] = getattr(robot_name_r, "name", str(robot_name_r))
            data["robotname_l"] = getattr(robot_name_l, "name", str(robot_name_l))

        self.righthand.calibrationoriginal = data.get('jointangleoriginal_r')
        self.lefthand.calibrationoriginal = data.get('jointangleoriginal_l')
        
        if data.get('jointanglefist_r'):
            self.righthand.calibrationfistpose = data['jointanglefist_r']
        if data.get('jointanglefist_l'):
            self.lefthand.calibrationfistpose = data['jointanglefist_l']
        
        if data.get('jointangleopose_r'):
            self.righthand.calibrationopose = data['jointangleopose_r']
        if data.get('jointangleopose_l'):
            self.lefthand.calibrationopose = data['jointangleopose_l']
        
        if self.lefthand.calibrationfistpose is None or self.righthand.calibrationfistpose is None:
            if self.lefthand.calibrationoriginal and self.lefthand.calibrationopose:
                self._calculate_fist_from_extension()
        
        if from_sample:
            TMP_FILE_PATH.parent.mkdir(parents=True, exist_ok=True)
            with open(TMP_FILE_PATH, 'w') as f:
                json.dump(data, f, indent=2)
            self.node.get_logger().info("样本标定数据已保存到用户标定文件")
        
        self.node.get_logger().info("标定数据加载成功")
        return True

    def _get_forced_positions(self, pose_type: str, hand_type: str):
        """
        获取强制姿态的标定数据
        
        Args:
            pose_type: 姿态类型 (open/fist/opose)
            hand_type: 手类型 (left/right)
            
        Returns:
            位置数据列表，如果不存在返回 None
        """
        if not TMP_FILE_PATH.exists():
            return None
        
        if self.calibration_cache is None:
            try:
                content = TMP_FILE_PATH.read_text()
                self.calibration_cache = json.loads(content)
            except:
                return None
        
        hand = hand_type.lower()
        pose_map = {
            'open': f'jointangleoriginal_{hand[0]}',
            'fist': f'jointanglefist_{hand[0]}',
            'opose': f'jointangleopose_{hand[0]}'
        }
        
        key = pose_map.get(pose_type)
        if key and self.calibration_cache:
            return self.calibration_cache.get(key)
        return None

    def process(self):
        """主处理函数"""
        # 初始化串口连接
        self.linkerforce_init()
        # 执行标定（如果需要）
        if self.calibration == "auto_calibrate":
            if not self.run_calibration():
                self.node.get_logger().error("标定失败，退出程序")
                return False
            self.calibration = -1
        self.node.create_timer(1.0/30, self.process_callback)  # 30Hz
        return True

    def stop_serial_threads(self):
        """停止串口线程，在 destroy_node 时调用"""
        self.runing = False
        if self.force_reader_left:
            self.force_reader_left.stop()
        if self.force_reader_right:
            self.force_reader_right.stop()
        self.node.get_logger().info("串口线程已停止")

    def set_mode(self, mode, param=None):
        """
        设置遥操作模式
        
        Args:
            mode: 运行模式
                - 'glove': 使用手套数据
                - 'fixed_opose': 使用固定O型姿态
                - 'fixed_fist': 使用固定握拳姿态
            param: 额外参数 (dict)
                - serial_debug: bool, 开启串口调试
                - mapper_debug: bool, 开启映射器调试
        """
        if param is None:
            param = {}
        
        # 处理串口调试开关
        if 'serial_debug' in param:
            debug_enabled = param['serial_debug']
            if hasattr(self.force_reader_left, 'isdebug'):
                self.force_reader_left.isdebug = debug_enabled
            if hasattr(self.force_reader_right, 'isdebug'):
                self.force_reader_right.isdebug = debug_enabled
            self.node.get_logger().info(f"串口调试: {'开启' if debug_enabled else '关闭'}")
        
        # 处理映射器调试开关
        if 'mapper_debug' in param:
            mapper_debug_enabled = param['mapper_debug']
            self.baseconfig.setdefault('debug', {})['mapper_debug'] = mapper_debug_enabled
            if hasattr(self.righthand, 'multi_state_mapper') and hasattr(self.righthand.multi_state_mapper, 'set_debug'):
                self.righthand.multi_state_mapper.set_debug(mapper_debug_enabled)
            if hasattr(self.lefthand, 'multi_state_mapper') and hasattr(self.lefthand.multi_state_mapper, 'set_debug'):
                self.lefthand.multi_state_mapper.set_debug(mapper_debug_enabled)
            if isinstance(mapper_debug_enabled, list):
                fingers_str = ', '.join(mapper_debug_enabled) if mapper_debug_enabled else '全部'
                self.node.get_logger().info(f"映射器调试: 开启 (手指: {fingers_str})")
            else:
                self.node.get_logger().info(f"映射器调试: {'开启' if mapper_debug_enabled else '关闭'}")
        
        # 处理强制手套数据源
        if 'force_glove_pose' in param:
            pose = param['force_glove_pose']
            if pose in ['open', 'fist', 'opose', 'none', None]:
                self.force_glove_pose = pose if pose != 'none' else None
                if self.force_glove_pose:
                    self.node.get_logger().info(f"强制手套数据源: {self.force_glove_pose}")
                else:
                    self.node.get_logger().info("强制手套数据源: 关闭，使用实际数据")
            else:
                self.node.get_logger().warn(f"无效的强制姿态: {pose}，可选: open/fist/opose/none")
        
        # 处理延伸指数因子
        if 'mapper_exp_factor' in param:
            exp_factor = param['mapper_exp_factor']
            if isinstance(exp_factor, dict):
                # 指定手指设置
                for finger, value in exp_factor.items():
                    if hasattr(self.righthand, 'multi_state_mapper') and hasattr(self.righthand.multi_state_mapper, 'exp_factors'):
                        if finger in self.righthand.multi_state_mapper.exp_factors:
                            self.righthand.multi_state_mapper.exp_factors[finger] = value
                    if hasattr(self.lefthand, 'multi_state_mapper') and hasattr(self.lefthand.multi_state_mapper, 'exp_factors'):
                        if finger in self.lefthand.multi_state_mapper.exp_factors:
                            self.lefthand.multi_state_mapper.exp_factors[finger] = value
                fingers_str = ', '.join([f"{k}:{v}" for k, v in exp_factor.items()])
                self.node.get_logger().info(f"延伸指数因子(指定): {fingers_str}")
            else:
                # 全部手指设置
                if hasattr(self.righthand, 'multi_state_mapper') and hasattr(self.righthand.multi_state_mapper, 'exp_factors'):
                    for finger in self.righthand.multi_state_mapper.exp_factors:
                        self.righthand.multi_state_mapper.exp_factors[finger] = exp_factor
                if hasattr(self.lefthand, 'multi_state_mapper') and hasattr(self.lefthand.multi_state_mapper, 'exp_factors'):
                    for finger in self.lefthand.multi_state_mapper.exp_factors:
                        self.lefthand.multi_state_mapper.exp_factors[finger] = exp_factor
                self.node.get_logger().info(f"延伸指数因子(全部): {exp_factor}")
        
        # 处理缩放因子
        if 'mapper_scale_factor' in param:
            scale_factor = param['mapper_scale_factor']
            if isinstance(scale_factor, dict):
                # 指定手指设置
                for finger, value in scale_factor.items():
                    if hasattr(self.righthand, 'multi_state_mapper') and hasattr(self.righthand.multi_state_mapper, 'scale_factors'):
                        if finger in self.righthand.multi_state_mapper.scale_factors:
                            self.righthand.multi_state_mapper.scale_factors[finger] = value
                    if hasattr(self.lefthand, 'multi_state_mapper') and hasattr(self.lefthand.multi_state_mapper, 'scale_factors'):
                        if finger in self.lefthand.multi_state_mapper.scale_factors:
                            self.lefthand.multi_state_mapper.scale_factors[finger] = value
                fingers_str = ', '.join([f"{k}:{v}" for k, v in scale_factor.items()])
                self.node.get_logger().info(f"缩放因子(指定): {fingers_str}")
            else:
                # 全部手指设置
                if hasattr(self.righthand, 'multi_state_mapper') and hasattr(self.righthand.multi_state_mapper, 'scale_factors'):
                    for finger in self.righthand.multi_state_mapper.scale_factors:
                        self.righthand.multi_state_mapper.scale_factors[finger] = scale_factor
                if hasattr(self.lefthand, 'multi_state_mapper') and hasattr(self.lefthand.multi_state_mapper, 'scale_factors'):
                    for finger in self.lefthand.multi_state_mapper.scale_factors:
                        self.lefthand.multi_state_mapper.scale_factors[finger] = scale_factor
                self.node.get_logger().info(f"缩放因子(全部): {scale_factor}")
        
        # 模式切换（仅当 mode 有明确值时）
        if mode == 'glove':
            if hasattr(self.righthand, 'use_fixed_pose'):
                self.righthand.use_fixed_pose = False
            if hasattr(self.lefthand, 'use_fixed_pose'):
                self.lefthand.use_fixed_pose = False
            self.node.get_logger().info("模式切换: 手套数据")
            
        elif mode == 'fixed_opose':
            if hasattr(self.righthand, 'use_fixed_pose'):
                self.righthand.use_fixed_pose = True
                self.righthand.fixed_pose = self.righthand.robot_opose
            if hasattr(self.lefthand, 'use_fixed_pose'):
                self.lefthand.use_fixed_pose = True
                self.lefthand.fixed_pose = self.lefthand.robot_opose
            self.node.get_logger().info("模式切换: 固定O型姿态")
            
        elif mode == 'fixed_fist':
            if hasattr(self.righthand, 'use_fixed_pose'):
                self.righthand.use_fixed_pose = True
                self.righthand.fixed_pose = self.righthand.robot_fist
            if hasattr(self.lefthand, 'use_fixed_pose'):
                self.lefthand.use_fixed_pose = True
                self.lefthand.fixed_pose = self.lefthand.robot_fist
            self.node.get_logger().info("模式切换: 固定握拳姿态")
            
        elif mode is not None:
            self.node.get_logger().warn(f"未知模式: {mode}")
