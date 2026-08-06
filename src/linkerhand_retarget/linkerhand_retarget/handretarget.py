#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import os
from pathlib import Path

# 强制使用src目录的路径
def setup_src_paths():
    """确保使用src目录而不是build目录"""
    # 获取工作空间的绝对路径
    current_file = Path(__file__).absolute()
    workspace_dir = current_file.parent.parent.parent.parent
    
    # 添加src目录到Python路径
    src_package_dir = workspace_dir / "src" / "linkerhand_retarget" / "linkerhand_retarget"
    if src_package_dir.exists():
        paths_to_add = [
            src_package_dir,
            src_package_dir / "linkerhand",
        ]
        
        for path in paths_to_add:
            if path.exists() and str(path) not in sys.path:
                sys.path.insert(0, str(path))
    
    return src_package_dir

workspace_dir = setup_src_paths()


import time
from threading import Thread, Event
from pathlib import Path
from queue import Empty
from typing import Optional
import numpy as np
import enum
import signal, sys


_script_dir = str(Path(__file__).parent)
# 使用本地 linkerhand
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)

from linkerhand.utils import *
from linkerhand.vtrdyncore import *
from linkerhand.handcore import HandCore
from linkerhand.config import HandConfig
from linkerhand.constants import RetargetingType, DataSource, MotionSource, RobotName
from linkerhand_retarget.mujoco_display import (
    MujocoDisplay,
    MujocoDisplayProcess,
    build_mujoco_display_plans,
    detect_loaded_hands,
    extract_mujoco_joint_positions,
)
from linkerhand_retarget.version import get_version

from ament_index_python.packages import get_package_share_directory
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor

import json


vr_pose_cache_r = []
vr_pose_cache_l = []
video_pose_cache_r = []
video_pose_cache_l = []
reangle_r = []
reangle_l = []
right_hand_pose_end = []
left_hand_pose_end = []


def shutdown_rclpy_if_running():
    try:
        ok = getattr(rclpy, "ok", None)
        if ok is None or ok():
            rclpy.shutdown()
    except RuntimeError:
        pass


def signal_handler(sig, frame):
    shutdown_rclpy_if_running()


class HandRetargetNode(Node):
    def __init__(self):
        super().__init__('handretarget_node')
        self.get_logger().info(f"LinkerHand Retarget SDK 版本: {get_version()}")
        print("Ready Create HandRetargetNode!")

        package_share_dir = workspace_dir

        self.robot_dir = package_share_dir  / "assets" / "robots" / "hands"
        self.base_config = package_share_dir 

        self.handconfig = HandConfig(str(self.robot_dir), str(self.base_config))
        self.handcore = HandCore(self.handconfig)

        self.baseconfig = self.handconfig.baseconfig
        self.retagetconfig = self.handconfig.retagetconfig

        # 声明参数并提供默认值
        # 兼容 Foxy (无 dynamic_typing) 和 Jazzy (有 dynamic_typing)
        try:
            auto_scan_desc = ParameterDescriptor(dynamic_typing=True)
        except (TypeError, AttributeError, AssertionError):
            auto_scan_desc = ParameterDescriptor()

        self.declare_parameters(
            namespace='',
            parameters=[
                ('calibration', False),
                ('ports', ['']),
                ('baudrate', 0),
                ('auto_scan', None, auto_scan_desc),
            ]
        )
        #
        self.scene, self.retargeting_r, self.retargeting_l, self.config_r, self.config_l = None, None, None, None, None
        self.robot_name_r, self.robot_name_l = None, None
        self.retargeting_type = None
        self.datasource_type = None
        self.motion_type = None
        self.udp_ip, self.udp_port, self.use_can, self.motion_device = None, None, None, None

        self.calibration = self.get_parameter('calibration').value
        print(f"是否启用标定: {self.calibration} ")
        
        # 读取命令行串口参数（候选列表）
        cmd_ports = self.get_parameter('ports').value
        self.cmd_ports = [p for p in cmd_ports if p] if cmd_ports else None
        self.cmd_baudrate = self.get_parameter('baudrate').value or None
        self.cmd_auto_scan = self.get_parameter('auto_scan').value
        
        if self.cmd_ports:
            print(f"命令行指定候选串口: {self.cmd_ports} @ {self.cmd_baudrate}")
        if self.cmd_auto_scan:
            print(f"命令行启用自动扫描")
        
        self.calibrationopen_r, self.calibrationopen_l, self.calibrationclose_r, self.calibrationclose_l = None, None, None, None
        self.retarget = None
        self.mujoco_displays = []
        self.mujoco_timer = None
        self._mujoco_debug_counter = 0
        self.datasource_type = DataSource[self.baseconfig["system"]["datasource_type"]]
        self.retargeting_type = RetargetingType[self.baseconfig["system"]["retargeting_type"]]
        self.motion_type = MotionSource[self.baseconfig["system"]["motion_type"]]
        self.robot_name_r = RobotName[self.baseconfig["system"]["robotname_r"]]
        self.robot_name_l = RobotName[self.baseconfig["system"]["robotname_l"]]

        self.udp_ip = self.baseconfig["udp"]["ip"]
        self.udp_port = int(self.baseconfig["udp"]["port"])
        self.use_can = bool(self.baseconfig["system"]["usecan"])
        self.motion_device = self.baseconfig["system"]["motion_device"]

        # LinkerEG 配置
        self.linkereg_port = self.baseconfig.get("linkereg", {}).get("port", None)
        self.linkereg_password = self.baseconfig.get("linkereg", {}).get("password", "i")

        self.righthandprint = bool(self.baseconfig["debug"]["joint_motor_debug_r"])
        self.lefthandprint = bool(self.baseconfig["debug"]["joint_motor_debug_l"])

        # if self.datasource_type == DataSource.vr:
        #     self.vr_right_sub = self.create_subscription(
        #         JointState,
        #         '/vr_right_hand_pose',
        #         self.vr_right_pose_callback,
        #         10)
        #     self.vr_left_sub = self.create_subscription(
        #         JointState,
        #         '/vr_left_hand_pose',
        #         self.vr_left_pose_callback,
        #         10)
        # elif self.datasource_type == DataSource.video:
        #     self.video_right_sub = self.create_subscription(
        #         JointState,
        #         '/video_right_hand_pose',
        #         self.video_right_pose_callback,
        #         10)
        #     self.video_left_sub = self.create_subscription(
        #         JointState,
        #         '/video_left_hand_pose',
        #         self.video_left_pose_callback,
        #         10)
        
        self.pubprintcount = 0

        # 订阅遥操作参数话题
        self.teleop_param_sub = self.create_subscription(
            String,
            '/hand_teleop_param',
            self.teleop_param_callback,
            10
        )

        # 发布遥操作状态话题
        self.teleop_state_pub = self.create_publisher(
            String,
            '/hand_teleop_state',
            10
        )

        # 当前模式
        self.current_mode = 'glove'

    def _start_mujoco_display_if_enabled(self):
        loaded_hands = detect_loaded_hands(self.retarget)
        left_reader = getattr(self.retarget, "force_reader_left", None)
        right_reader = getattr(self.retarget, "force_reader_right", None)
        plans = build_mujoco_display_plans(
            self.baseconfig,
            package_dir=self.base_config,
            robot_name_r=self.robot_name_r,
            robot_name_l=self.robot_name_l,
            loaded_hands=loaded_hands,
            urdf_paths={
                "right": self.handcore.righturdfpath,
                "left": self.handcore.lefturdfpath,
            },
        )

        if not any(plan.enabled for plan in plans):
            self.get_logger().info("MuJoCo display disabled")
            return

        self.get_logger().info(
            "MuJoCo display auto detection: "
            f"left_handtype={getattr(left_reader, 'handtype', None)}, "
            f"right_handtype={getattr(right_reader, 'handtype', None)}, "
            f"loaded_hands={loaded_hands}, "
            f"plans={[(plan.hand, str(plan.model_path)) for plan in plans]}"
        )

        startable_plan_count = sum(1 for plan in plans if plan.should_start)
        display_class = MujocoDisplayProcess if startable_plan_count > 1 else MujocoDisplay
        if display_class is MujocoDisplayProcess:
            self.get_logger().warn(
                "MuJoCo display will use isolated processes for multiple hands "
                "to avoid native viewer crashes."
            )

        for plan in plans:
            for warning in plan.warnings:
                self.get_logger().warn(warning)

            if not plan.should_start:
                self.get_logger().warn(
                    f"MuJoCo display for {plan.hand} hand will not start; SDK startup continues."
                )
                continue

            try:
                display = display_class(
                    plan.model_path,
                    fps=plan.fps,
                    hand=plan.hand,
                    model_scale=plan.model_scale,
                    model_rotate_rpy=plan.model_rotate_rpy,
                    model_translate_xyz=plan.model_translate_xyz,
                ).start()
                self.mujoco_displays.append(display)
                self.get_logger().info(
                    "MuJoCo display started for "
                    f"{plan.hand} hand: {plan.model_path}, "
                    f"scale={plan.model_scale}, rotate_rpy={plan.model_rotate_rpy}, "
                    f"translate_xyz={plan.model_translate_xyz}"
                )
            except Exception as exc:
                self.get_logger().warn(
                    "MuJoCo display failed to start; SDK startup continues. "
                    f"hand={plan.hand}, model={plan.model_path}, error={exc}"
                )

        if self.mujoco_displays and self.mujoco_timer is None:
            fps = max(display.fps for display in self.mujoco_displays)
            self.mujoco_timer = self.create_timer(1.0 / fps, self._sync_mujoco_displays)

    def _sync_mujoco_displays(self):
        for display in self.mujoco_displays:
            try:
                joint_positions = extract_mujoco_joint_positions(
                    self.handcore,
                    hand=display.hand,
                    movable_joint_names=display.movable_joint_names,
                    hand_model=(
                        self.retarget.lefthand
                        if display.hand == "left"
                        else self.retarget.righthand
                    ),
                )
                if joint_positions:
                    self._log_mujoco_thumb_roll_debug(display.hand, joint_positions)
                    display.update_joint_positions(joint_positions)
            except Exception as exc:
                self.get_logger().warn(
                    f"MuJoCo display update failed; hand={display.hand}, error={exc}"
                )

    def _log_mujoco_thumb_roll_debug(self, hand, joint_positions):
        debug_setting = self.baseconfig.get("debug", {}).get("mapper_debug", False)
        if isinstance(debug_setting, list):
            enabled = "thumb_rotate" in debug_setting
        else:
            enabled = bool(debug_setting)
        if not enabled or hand != "right":
            return
        if self.robot_name_r != RobotName.o20:
            return
        self._mujoco_debug_counter += 1
        if self._mujoco_debug_counter % 30 != 1:
            return
        hand_model = getattr(getattr(self, "retarget", None), "righthand", None)
        arc_values = getattr(hand_model, "g_jointpositions_arc", None)
        arc0 = arc_values[0] if arc_values and len(arc_values) > 0 else None
        self.get_logger().warn(
            "MuJoCo O20 thumb_cmc_roll sync: "
            f"sent={joint_positions.get('thumb_cmc_roll')}, "
            f"g_arc0={arc0}, "
            f"joint_count={len(joint_positions)}"
        )

    def teleop_param_callback(self, msg):
        """处理遥操作参数话题回调"""
        try:
            param = json.loads(msg.data)
            mode = param.get('mode')  # 可能为 None
            
            if self.retarget is not None and hasattr(self.retarget, 'set_mode'):
                self.retarget.set_mode(mode, param)
                if mode:
                    self.current_mode = mode
                # 发布状态反馈
                state_msg = String()
                state_msg.data = json.dumps({
                    'mode': mode or self.current_mode,
                    'status': 'success'
                })
                self.teleop_state_pub.publish(state_msg)
            else:
                self.get_logger().warn("retarget 未初始化或不支持 set_mode")
                state_msg = String()
                state_msg.data = json.dumps({
                    'mode': mode or 'unknown',
                    'status': 'failed',
                    'error': 'retarget not initialized'
                })
                self.teleop_state_pub.publish(state_msg)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON 解析错误: {e}")
            state_msg = String()
            state_msg.data = json.dumps({
                'mode': 'unknown',
                'status': 'failed',
                'error': str(e)
            })
            self.teleop_state_pub.publish(state_msg)
        except Exception as e:
            self.get_logger().error(f"参数处理错误: {e}")

    def retargetrun(self):
        if self.motion_type == MotionSource.udexreal:
            from linkerhand_retarget.motion.udexreal.retarget import Retarget
            self.retarget = Retarget(
                self,
                ip=self.udp_ip,
                port=self.udp_port,
                deviceid=self.motion_device,
                righthand=self.robot_name_r,
                lefthand=self.robot_name_l,
                handcore=self.handcore,
                lefthandpubprint=self.lefthandprint,
                righthandpubprint=self.righthandprint
            )
        elif self.motion_type == MotionSource.udexrealv2t:
            from linkerhand_retarget.motion.udexrealv2t.retarget import Retarget
            self.retarget = Retarget(
                self,
                ip=self.udp_ip,
                port=self.udp_port,
                deviceid=self.motion_device,
                righthand=self.robot_name_r,
                lefthand=self.robot_name_l,
                handcore=self.handcore,
                lefthandpubprint=self.lefthandprint,
                righthandpubprint=self.righthandprint,
                calibration = self.calibration
            )
        elif self.motion_type == MotionSource.linkerforce:
            from linkerhand_retarget.motion.linkerforce.retarget import Retarget
            self.retarget = Retarget(
                self,
                righthand=self.robot_name_r,
                lefthand=self.robot_name_l,
                handcore=self.handcore,
                lefthandpubprint=self.lefthandprint,
                righthandpubprint=self.righthandprint,
                calibration = self.calibration,
                baseconfig = self.baseconfig,
                cmd_ports=self.cmd_ports,
                cmd_baudrate=self.cmd_baudrate,
                cmd_auto_scan=self.cmd_auto_scan
            )
        elif self.motion_type == MotionSource.vtrdyn:
            from linkerhand_retarget.motion.vtrdyn.retarget import Retarget
            self.retarget = Retarget(
                self,
                ip=self.udp_ip,
                port=self.udp_port,
                righthand=self.robot_name_r,
                lefthand=self.robot_name_l,
                handcore=self.handcore,
                lefthandpubprint=self.lefthandprint,
                righthandpubprint=self.righthandprint,
                calibration = self.calibration
            )     
        elif self.motion_type == MotionSource.linkermcg:
            from linkerhand_retarget.motion.linkermcg.retarget import Retarget
            self.retarget = Retarget(
                self,
                ip=self.udp_ip,
                port=self.udp_port,
                righthand=self.robot_name_r,
                lefthand=self.robot_name_l,
                handcore=self.handcore,
                lefthandpubprint=self.lefthandprint,
                righthandpubprint=self.righthandprint
            )    
        elif self.motion_type == MotionSource.linkermcg_m7:
            from linkerhand_retarget.motion.linkermcg_m7.retarget import Retarget
            self.retarget = Retarget(
                self,
                ip=self.udp_ip,
                port=self.udp_port,
                righthand=self.robot_name_r,
                lefthand=self.robot_name_l,
                handcore=self.handcore,
                lefthandpubprint=self.lefthandprint,
                righthandpubprint=self.righthandprint
            )
        elif self.motion_type == MotionSource.linkereg2:
            from linkerhand_retarget.motion.linkereg.retarget import Retarget
            self.retarget = Retarget(
                self,
                port=self.linkereg_port,
                baudrate=921600,
                password=self.linkereg_password,
                isdebug=bool(self.baseconfig["debug"]["joint_pub_debug"]),
                mode='sdk'  # SDK控制模式
            )
        elif self.motion_type == MotionSource.linkereg1:
            from linkerhand_retarget.motion.linkereg.retarget import Retarget
            self.retarget = Retarget(
                self,
                port=self.linkereg_port,
                baudrate=921600,
                password=self.linkereg_password,
                isdebug=bool(self.baseconfig["debug"]["joint_pub_debug"]),
                mode='receiver'  # 接收器控制模式 (需要连接灵巧手)
            )
        if self.retarget is None:
            self.get_logger().error("未正确创建应用实例")
            return False
        else:
            print("启动应用实例")
            started = self.retarget.process()
            if not started:
                return False
            self._start_mujoco_display_if_enabled()
            return True

def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        signal.signal(signal.SIGINT, signal_handler)
        node = HandRetargetNode()
        executor = MultiThreadedExecutor()
        started = node.retargetrun()
        if started:
            # Keep the node alive
            rclpy.spin(node, executor)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("收到终止信号")
    finally:
        if node is not None:
            # 停止串口线程
            if hasattr(node, 'retarget') and node.retarget and hasattr(node.retarget, 'stop_serial_threads'):
                node.retarget.stop_serial_threads()
            if hasattr(node, 'mujoco_displays'):
                for display in node.mujoco_displays:
                    try:
                        display.close()
                    except Exception as exc:
                        node.get_logger().warn(f"关闭 MuJoCo display 失败: {exc}")
            node.destroy_node()
        shutdown_rclpy_if_running()


if __name__ == '__main__':
    main()
