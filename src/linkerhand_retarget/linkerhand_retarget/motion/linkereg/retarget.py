#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LinkerEG 遥操作手套 Retarget 模块

功能:
- 通过串口连接 LinkerEG 手套
- 自动初始化 (frame_enable -> 读版本 -> 启用映射数据)
- 接收左右手数据并通过 ROS2 话题发布
- 支持传感器原始数据推送
- 支持通过话题动态启用/禁用原始数据
- 完全不依赖 HandCore

话题列表:
  控制数据 (驱动机械手):
    /cb_right_hand_control_cmd  (sensor_msgs/JointState)
    /cb_left_hand_control_cmd   (sensor_msgs/JointState)
  
  传感器原始数据:
    /cb_right_hand_raw_data     (sensor_msgs/JointState) - 15个int32关节值
    /cb_left_hand_raw_data      (sensor_msgs/JointState) - 15个int32关节值
  
  控制命令:
    /cb_hand_setting_cmd        (std_msgs/String) - 发送 "on" 或 "off"
"""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .linkeregcore import LinkerEGSerial, PROTOCOL_MAP


class Retarget:
    """LinkerEG Retarget 类"""
    
    def __init__(self, 
                 node: Node,
                 port: str = None,
                 baudrate: int = 115200,
                 password: str = 'i',
                 isdebug: bool = False,
                 mode: str = 'sdk'):
        """
        初始化 LinkerEG Retarget
        
        Args:
            node: ROS2 节点
            port: 串口路径，为 None 时自动扫描 (左右手共用同一个串口)
            baudrate: 波特率 (默认 115200)
            password: sudo 密码，用于自动修复串口权限
            isdebug: 是否打印调试信息
            mode: 控制模式 'sdk'=SDK控制模式, 'receiver'=接收器控制模式
        """
        self.node = node
        self.port = port
        self.baudrate = baudrate
        self.password = password
        self.isdebug = isdebug
        self.mode = mode
        self.raw_data_enabled = False  # 原始数据推送状态 (通过话题控制)
        self.running = True
        self.pubprintcount = 0
        self.raw_count = 0
        
        # 创建 LinkerEG 串口通讯对象
        self.linkereg = LinkerEGSerial(
            port=port, 
            baudrate=baudrate, 
            password=password,
            isdebug=isdebug,
            mode=mode
        )
        
        # 设置数据回调
        self.linkereg.on_right_hand_data = self._on_right_hand_data
        self.linkereg.on_left_hand_data = self._on_left_hand_data
        
        # ROS2 发布器 - 控制数据
        self.publisher_r = self.node.create_publisher(
            JointState,
            '/cb_right_hand_control_cmd',
            10
        )
        self.publisher_l = self.node.create_publisher(
            JointState,
            '/cb_left_hand_control_cmd',
            10
        )
        
        # ROS2 发布器 - 传感器原始数据 (默认创建，通过话题控制启用/禁用)
        self.publisher_r_raw = self.node.create_publisher(
            JointState,
            '/cb_right_hand_raw_data',
            10
        )
        self.publisher_l_raw = self.node.create_publisher(
            JointState,
            '/cb_left_hand_raw_data',
            10
        )
        
        # ROS2 订阅器 - 设置命令 (用于动态启用/禁用原始数据)
        self.setting_sub = self.node.create_subscription(
            String,
            '/cb_hand_setting_cmd',
            self._on_setting_cmd,
            10
        )
        
        # 定时器 - 检查连接状态
        self.status_timer = self.node.create_timer(1.0, self._status_callback)
        
        self.node.get_logger().info("LinkerEG Retarget 模块已创建")
    
    def _on_right_hand_data(self, control_data: list, protocol: int):
        """右手数据回调 - 直接发布数据"""
        if not self.running:
            return
        
        now = self.node.get_clock().now().to_msg()
        protocol_info = PROTOCOL_MAP.get(protocol, {'name': 'Unknown', 'joints': len(control_data)})
        
        # 调试打印
        if self.isdebug and self.pubprintcount % 50 == 0:
            self.node.get_logger().info(
                f"[LinkerEG] 右手 ({protocol_info['name']}): ctrl={control_data}"
            )
        
        msg_ctrl = JointState()
        msg_ctrl.header.stamp = now
        msg_ctrl.name = [f'joint{i + 1}' for i in range(len(control_data))]
        msg_ctrl.position = [float(v) for v in control_data]
        msg_ctrl.velocity = [255.0] * len(control_data)
        self.publisher_r.publish(msg_ctrl)
        self.pubprintcount += 1
    
    def _on_left_hand_data(self, control_data: list, protocol: int):
        """左手数据回调 - 直接发布数据"""
        if not self.running:
            return
        
        now = self.node.get_clock().now().to_msg()
        protocol_info = PROTOCOL_MAP.get(protocol, {'name': 'Unknown', 'joints': len(control_data)})
        
        # 调试打印
        if self.isdebug and self.pubprintcount % 50 == 0:
            self.node.get_logger().info(
                f"[LinkerEG] 左手 ({protocol_info['name']}): ctrl={control_data}"
            )
        
        msg_ctrl = JointState()
        msg_ctrl.header.stamp = now
        msg_ctrl.name = [f'joint{i + 1}' for i in range(len(control_data))]
        msg_ctrl.position = [float(v) for v in control_data]
        msg_ctrl.velocity = [255.0] * len(control_data)
        self.publisher_l.publish(msg_ctrl)
    
    def _on_right_hand_raw_data(self, raw_data: list):
        """右手传感器原始数据回调 - 直接发布数据"""
        if not self.running or not self.raw_data_enabled:
            return
        
        now = self.node.get_clock().now().to_msg()
        joint_names = [
            'thumb_spread', 'thumb_bend', 'thumb_tip',
            'index_spread', 'index_bend', 'index_tip',
            'middle_spread', 'middle_bend', 'middle_tip',
            'ring_spread', 'ring_bend', 'ring_tip',
            'pinky_spread', 'pinky_bend', 'pinky_tip'
        ]
        msg = JointState()
        msg.header.stamp = now
        msg.name = joint_names
        msg.position = [float(v) for v in raw_data]
        self.publisher_r_raw.publish(msg)
        self.raw_count += 1
    
    def _on_left_hand_raw_data(self, raw_data: list):
        """左手传感器原始数据回调 - 直接发布数据"""
        if not self.running or not self.raw_data_enabled:
            return
        
        now = self.node.get_clock().now().to_msg()
        joint_names = [
            'thumb_spread', 'thumb_bend', 'thumb_tip',
            'index_spread', 'index_bend', 'index_tip',
            'middle_spread', 'middle_bend', 'middle_tip',
            'ring_spread', 'ring_bend', 'ring_tip',
            'pinky_spread', 'pinky_bend', 'pinky_tip'
        ]
        msg = JointState()
        msg.header.stamp = now
        msg.name = joint_names
        msg.position = [float(v) for v in raw_data]
        self.publisher_l_raw.publish(msg)
    
    def _on_setting_cmd(self, msg: String):
        """
        处理设置命令
        
        支持的命令:
            on  - 启用传感器原始数据推送
            off - 禁用传感器原始数据推送
        """
        cmd = msg.data.strip().lower()
        
        if cmd == 'on':
            if self.raw_data_enabled:
                return  # 已启用，静默忽略
            
            # 设置回调
            self.linkereg.on_right_hand_raw_data = self._on_right_hand_raw_data
            self.linkereg.on_left_hand_raw_data = self._on_left_hand_raw_data
            
            # 发送启用命令
            self.linkereg.enable_raw_data()
            self.raw_data_enabled = True
            self.node.get_logger().info("[LinkerEG] 已启用传感器原始数据推送")
            self.node.get_logger().info("[LinkerEG] 原始数据话题: /cb_right_hand_raw_data, /cb_left_hand_raw_data")
        
        elif cmd == 'off':
            if not self.raw_data_enabled:
                self.node.get_logger().info("[LinkerEG] 原始数据已经禁用")
                return
            
            # 发送禁用命令
            self.linkereg.disable_raw_data()
            self.raw_data_enabled = False
            
            # 清除回调
            self.linkereg.on_right_hand_raw_data = None
            self.linkereg.on_left_hand_raw_data = None
            
            self.node.get_logger().info("[LinkerEG] 已禁用传感器原始数据推送")
        
        else:
            self.node.get_logger().warn(f"[LinkerEG] 未知命令: {cmd}")
            self.node.get_logger().info("[LinkerEG] 支持的命令: on, off")
    
    def _status_callback(self):
        """定时状态检查回调"""
        if not self.running:
            return
        
        if self.linkereg.version:
            # 版本号只打印一次
            pass
    
    def initialize(self) -> bool:
        """初始化串口连接和手套"""
        if self.port:
            self.node.get_logger().info(f"[LinkerEG] 正在连接指定串口 {self.port}...")
        else:
            self.node.get_logger().info("[LinkerEG] 未指定串口，将自动扫描...")
        
        # 打开串口（如果 port 为 None，会自动扫描）
        if not self.linkereg.open():
            self.node.get_logger().error("[LinkerEG] 无法打开串口")
            return False
        
        # 更新实际使用的串口
        self.port = self.linkereg.port
        self.node.get_logger().info(f"[LinkerEG] 已连接串口: {self.port}")
        
        # 启动接收线程
        self.linkereg.start()
        
        # 执行初始化流程
        if not self.linkereg.initialize():
            self.node.get_logger().error("[LinkerEG] 初始化失败")
            return False
        


        # 等待版本号
        timeout = 2.0
        start_time = time.time()
        while self.linkereg.version is None and (time.time() - start_time) < timeout:
            time.sleep(0.1)
        
        if self.linkereg.version:
            self.node.get_logger().info(f"[LinkerEG] 手套版本: {self.linkereg.version}")
        else:
            self.node.get_logger().warn("[LinkerEG] 未能获取版本号，但继续运行")
        
        self.node.get_logger().info("[LinkerEG] 初始化成功，等待数据...")
        self.node.get_logger().info("[LinkerEG] 控制数据话题: /cb_right_hand_control_cmd, /cb_left_hand_control_cmd")
        self.node.get_logger().info("[LinkerEG] 原始数据话题: /cb_right_hand_raw_data, /cb_left_hand_raw_data (默认禁用)")
        self.node.get_logger().info("[LinkerEG] 设置命令话题: /cb_hand_setting_cmd (on/off)")
        return True
    
    def process(self):
        """主处理函数 (阻塞)"""
        if not self.initialize():
            self.node.get_logger().error("[LinkerEG] 初始化失败，无法启动")
            return
        
        try:
            # 保持节点运行
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            self.node.get_logger().info("[LinkerEG] 收到退出信号")
        finally:
            self.shutdown()
    
    def shutdown(self):
        """关闭模块"""
        self.running = False
        # 禁用原始数据推送 (如果启用了)
        if self.raw_data_enabled:
            self.linkereg.disable_raw_data()
        self.linkereg.close()
        self.node.get_logger().info("[LinkerEG] 模块已关闭")
