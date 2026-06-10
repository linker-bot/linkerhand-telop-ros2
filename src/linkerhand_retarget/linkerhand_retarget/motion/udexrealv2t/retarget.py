
import time
import json
import copy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from datetime import datetime, timedelta
from tqdm import tqdm
from pathlib import Path
from colorama import Fore, init
from typing import Optional

from ...linkerhand.udexrealcore import UdexRealScoketUdp, TimeoutStatus, UdexRealData
from ...linkerhand.handcore import HandCore

from ...linkerhand.constants import RobotName, ROBOT_LEN_MAP


tmp_file_path = Path(__file__).parent / "tmp" / "jointangle_data.tmp"



class CalibrationProgress:
    """校准进度管理器"""
    def __init__(self, duration: float = 3.0):
        self.step_started = False
        self.message = ""
        self.start_time = 0
        self.duration = duration
        self.last_progress = -1
        
        # 进度条配置
        self.bar_length = 60  # 增加到60，占更多屏幕
        self.filled_char = '█'
        self.empty_char = '░'
        
        # 状态跟踪
        self.message_printed = False
    
    def start_step(self, message: str, duration: Optional[float] = None):
        """开始新的校准步骤"""
        # 重置状态
        self.step_started = True
        self.message = message
        self.start_time = time.time()
        self.last_progress = -1
        self.message_printed = False
        
        if duration is not None:
            self.duration = duration
        
        # 只在开始时打印一次消息
        if not self.message_printed:
            self.message_printed = True
    
    def update_progress(self) -> tuple[bool, int]:
        """
        更新并显示进度
        
        Returns:
            tuple[bool, int]: (是否完成, 当前进度百分比)
        """
        if not self.step_started:
            return False, 0
            
        elapsed = time.time() - self.start_time
        progress = min(100, int((elapsed / self.duration) * 100))
        
        # 只在进度有变化时更新显示
        if progress != self.last_progress:
            self.last_progress = progress
            
            # 计算填充长度
            filled = int(self.bar_length * progress // 100)
            
            # 构建进度条
            if progress < 100:
                bar = self.filled_char * filled + self.empty_char * (self.bar_length - filled)
            else:
                # 100%时显示完整条
                bar = self.filled_char * self.bar_length
            
            # 使用 \r 和 \033[K 确保完全覆盖
            print(f'\r\033[K{self.message} |{bar}| {progress:3d}%', end='', flush=True)
        
        # 检查是否完成
        is_completed = elapsed >= self.duration
        if is_completed:
            self.step_started = False
            # 完成后不在这里换行，让调用者控制
            
        return is_completed, progress
    
    def get_progress_info(self) -> dict:
        """获取进度信息"""
        if not self.step_started:
            return {"elapsed": 0, "progress": 0, "remaining": 0}
        
        elapsed = time.time() - self.start_time
        progress = min(1.0, elapsed / self.duration)
        
        return {
            "elapsed": elapsed,
            "progress": progress,
            "remaining": max(0, self.duration - elapsed),
            "progress_percent": int(progress * 100)
        }
    
    def reset(self):
        """重置状态"""
        self.step_started = False
        self.message = ""
        self.start_time = 0
        self.last_progress = -1
        self.message_printed = False
    
    def is_active(self) -> bool:
        """检查是否正在运行"""
        return self.step_started
    
    def stop(self, success: bool = True, final_message: Optional[str] = None):
        """停止进度条"""
        if not self.step_started:
            return
        
        self.step_started = False
        
        if success:
            if final_message:
                print(f"\r\033[K✅ {final_message}")
            else:
                print(f"\r\033[K✅ {self.message} - 完成")
        else:
            if final_message:
                print(f"\r\033[K❌ {final_message}")
            else:
                print(f"\r\033[K❌ {self.message} - 失败")


class Retarget():
    def __init__(self,node, ip, port, deviceid, lefthand: RobotName, righthand: RobotName, handcore: HandCore,
                lefthandpubprint: bool, righthandpubprint: bool,calibration :bool = False):
        self.node = node
        self.udp_ip = ip
        self.udp_port = port
        self.motion_device = deviceid
        self.lefthandtype = lefthand
        self.righthandtype = righthand
        self.handcore = handcore
        self.runing = True
        self.lefthandpubprint = lefthandpubprint
        self.righthandpubprint = righthandpubprint
        self.calibration = calibration

        # 根据右手类型初始化
        if self.righthandtype == RobotName.o7 \
            or self.righthandtype == RobotName.l7 \
            or self.righthandtype == RobotName.o7v1 \
            or self.righthandtype == RobotName.o7v3:
            from .hand.udexreal_l7 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.o6:
            from .hand.udexreal_o6 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l6:
            from .hand.udexreal_l6 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.o20:
            from .hand.udexreal_o20 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.o30:
            from .hand.udexreal_o30 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        # elif self.righthandtype == RobotName.l25:
        #     from .hand.udexreal_l25 import RightHand
        #     self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        # elif self.righthandtype == RobotName.t25:
        #     from .hand.udexreal_t25 import RightHand
        #     self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l20:
            from .hand.udexreal_l20 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l10 \
            or self.righthandtype == RobotName.l10v7 :
            from .hand.udexreal_l10v7 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l21:
            from .hand.udexreal_l21 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])

        # 根据LEFT手类型初始化
        if self.lefthandtype == RobotName.o7 \
            or self.lefthandtype == RobotName.l7 \
            or self.lefthandtype == RobotName.o7v1 \
            or self.lefthandtype == RobotName.o7v3:
            from .hand.udexreal_l7 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.o6:
            from .hand.udexreal_o6 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l6:
            from .hand.udexreal_l6 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.o20:
            from .hand.udexreal_o20 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.o30:
            from .hand.udexreal_o30 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l25:
            from .hand.udexreal_l25 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        # elif self.lefthandtype == RobotName.t25:
        #     from .hand.udexreal_t25 import LeftHand
        #     self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l20:
            from .hand.udexreal_l20 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l10 \
            or self.lefthandtype == RobotName.l10v7 :
            from .hand.udexreal_l10v7 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l21:
            from .hand.udexreal_l21 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])

        # ROS2 发布器
        self.publisher_r = self.node.create_publisher(
            JointState,
            '/cb_right_hand_control_cmd',
            self.handcore.hand_numjoints_r)
            
        self.publisher_l = self.node.create_publisher(
            JointState,
            '/cb_left_hand_control_cmd',
            self.handcore.hand_numjoints_l)
            
        self.timer = self.node.create_timer(1.0/120, self.process_callback)  # 120Hz
        self.pubprintcount = 0
        self.udp_datacapture = None

        self.calibration_progress = CalibrationProgress(duration=3.0)

    def initialize_udp(self):
        """初始化UDP连接"""
        self.udp_datacapture = UdexRealScoketUdp(
            host=self.udp_ip,
            port=self.udp_port,
            device_id=self.motion_device)
        self.udp_datacapture.set_timeout_callback(self.on_timeout_callback)
        self.udp_datacapture.set_data_recovered_callback(self.on_data_recovered_callback)
        return self.udp_datacapture.udp_initial()
    
    def on_timeout_callback(self, status: TimeoutStatus):
        """超时回调函数"""
        # if status.consecutive_timeout_checks == 1:  # 第一次超时
        #     print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] "
        #           f"警告: 数据接收超时，{status.time_since_last_data:.1f}秒未收到数据")
        if status.consecutive_timeout_checks % 1 == 0:  # 每10次检查打印一次
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] "
                f"警告: 数据接收超时，{status.time_since_last_data:.1f}秒未收到数据")

    def on_data_recovered_callback(self):
        """数据恢复回调函数"""
        print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 数据连接已恢复")


    def process_callback(self):
        if not self.runing:
            return
        
        mocapdata = self.udp_datacapture.realmocapdata

        # 检查数据是否更新
        if not mocapdata.is_update:
            return
        
        if self.calibration != -1:
            step_info = {
                None: ("请并拢五指...", 0),
                0: ("请做握拳姿势...", 1),
                1: ("请做O型姿势...", -1)
            }.get(self.calibration)
            
            if not step_info:
                return
                
            message, next_step = step_info
            
            # 启动或更新进度条
            if not self.calibration_progress.is_active():
                self.calibration_progress.start_step(message)
            
            # 更新显示
            is_completed, _ = self.calibration_progress.update_progress()
            
            if is_completed:
                print()  # 换行
                if self.calibration is None:
                    # 第一次校准：并拢五指姿势
                    self.righthand.calibrationoriginal = mocapdata.jointangle_rHand.copy()
                    self.lefthand.calibrationoriginal = mocapdata.jointangle_lHand.copy()
                    self.calibration = 0
                    
                elif self.calibration == 0:
                    # 第二次校准：握拳姿势
                    self.righthand.calibrationfistpose = mocapdata.jointangle_rHand.copy()
                    self.lefthand.calibrationfistpose = mocapdata.jointangle_lHand.copy()
                    self.calibration = 1
                    
                elif self.calibration == 1:
                    # 第三次校准：O型姿势
                    self.righthand.calibrationopose = mocapdata.jointangle_rHand.copy()
                    self.lefthand.calibrationopose = mocapdata.jointangle_lHand.copy()
                    self.calibration = -1
                    
                    # 完成所有校准步骤
                    self._save_to_tmp()
                    self.righthand.initialize_mapper()
                    self.lefthand.initialize_mapper()
                    print("✅ 校准完成！")
                
                # 重置进度条，准备下一步或结束
                self.calibration_progress.reset()
            
            return
        
        # 处理左右手原始数据
        self.lefthand.joint_arc_update(mocapdata.jointangle_lHand)
        self.righthand.joint_arc_update(mocapdata.jointangle_rHand)

        # 速度环节处理
        self.lefthand.speed_update()
        self.righthand.speed_update()

        # 调试打印
        if self.lefthandpubprint and self.pubprintcount % 1 == 0:
            self.node.get_logger().info(f"左手位置: {self.lefthand.g_jointpositions}")
        if self.righthandpubprint and self.pubprintcount % 1 == 0:
            self.node.get_logger().info(f"右手位置: {self.righthand.g_jointpositions}")

        # 发布右手数据
        msg_r = JointState()
        msg_r.header.stamp = self.node.get_clock().now().to_msg()
        msg_r.name = [f'joint{i + 1}' for i in range(len(self.righthand.g_jointpositions))]
        msg_r.position = [float(num) for num in self.righthand.g_jointpositions]
        msg_r.velocity = [float(num) for num in self.righthand.g_jointvelocity]
        self.publisher_r.publish(msg_r)

        # 发布左手数据
        msg_l = JointState()
        msg_l.header.stamp = self.node.get_clock().now().to_msg()
        msg_l.name = [f'joint{i + 1}' for i in range(len(self.lefthand.g_jointpositions))]
        msg_l.position = [float(num) for num in self.lefthand.g_jointpositions]
        msg_l.velocity = [float(num) for num in self.lefthand.g_jointvelocity]
        self.publisher_l.publish(msg_l)

        self.pubprintcount += 1

    def _save_to_tmp(self):
        """
        保存 jointangle_r 数据和时间戳到临时文件
        """
        newdata = {
            "timestamp": datetime.now().isoformat(),  # 当前时间
            "jointangleoriginal_r": self.righthand.calibrationoriginal,
            "jointangleoriginal_l": self.lefthand.calibrationoriginal,
            "jointanglefist_r": self.righthand.calibrationfistpose,
            "jointanglefist_l": self.lefthand.calibrationfistpose,
            "jointangleopose_r": self.righthand.calibrationopose,
            "jointangleopose_l": self.lefthand.calibrationopose
        }
        try:
            tmp_file_path.parent.mkdir(parents=True, exist_ok=True)
            if tmp_file_path.exists():
                content = tmp_file_path.read_text()
                historydata = json.loads(content)
                # if self.force_reader_left.handtype != 'Left':
                #     # 左手数据不存在
                #     newdata["jointangleoriginal_l"] = historydata["jointangleoriginal_l"]
                #     newdata["jointanglefist_l"] = historydata["jointanglefist_l"]
                #     newdata["jointangleopose_l"] = historydata["jointangleopose_l"]
                # if self.force_reader_right.handtype != 'Right':
                #     newdata["jointangleoriginal_r"] = historydata["jointangleoriginal_r"]
                #     newdata["jointanglefist_r"] = historydata["jointanglefist_r"]
                #     newdata["jointangleopose_r"] = historydata["jointangleopose_r"]
            json_str = json.dumps(newdata, indent=2)
            tmp_file_path.write_text(json_str)
            print("保存成功")
            return True
        except Exception as e:
            print(f"保存失败: {e}")
            return False

    def _load_from_tmp(self):
        """
        从临时文件读取数据，检查时间戳有效性
        - 如果文件不存在、时间戳为空或超过1小时，返回 False
        - 否则返回 jointangle_r 数据
        """
        if not tmp_file_path.exists():
            print("文件不存在")
            return False
        try:
            content = tmp_file_path.read_text()
            data = json.loads(content)
        except Exception as e:
            print(f"读取失败: {e}")
            return False

        # 检查时间戳
        if 'timestamp' not in data or not data['timestamp']:
            print("无效的时间戳...")
            return False

        # 检查是否超过8小时                     
        saved_time = datetime.fromisoformat(data['timestamp'])
        current_time = datetime.now()
        
        # 计算时间差
        time_diff = current_time - saved_time
        if time_diff > timedelta(hours=8):
            print("标定数据有效期不足8小时,暂不进行标定流程...")
        try:
            self.righthand.calibrationoriginal = data['jointangleoriginal_r']
            self.lefthand.calibrationoriginal = data['jointangleoriginal_l']
            self.righthand.calibrationfistpose = data['jointanglefist_r']
            self.lefthand.calibrationfistpose = data['jointanglefist_l']                                                                           
            self.righthand.calibrationopose = data['jointangleopose_r']
            self.lefthand.calibrationopose = data['jointangleopose_l']
        except Exception as e:
            print(f"标定数据异常: {e}")
            return False
        return True


    def process(self):
        """主处理函数"""
        if not self.initialize_udp():
            self.node.get_logger().error("初始化配置网络失败")
            return
        
        if self.calibration is True:
            self.calibration = None
        else:
            if self._load_from_tmp() is True:
                print("跳过标定流程")
                self.calibration = -1
                self.righthand.initialize_mapper()
                self.lefthand.initialize_mapper()
            else:
                self.calibration = None
