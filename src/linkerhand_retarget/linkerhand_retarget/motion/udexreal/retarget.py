
import time
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from datetime import datetime
from pathlib import Path

# 将项目根目录放在最前面
# 强制使用项目本地的 linkerhand 模块
_project_root = Path(__file__).absolute().parent.parent.parent
_project_root_str = str(_project_root)

if _project_root_str in sys.path:
    sys.path.remove(_project_root_str)
sys.path.insert(0, _project_root_str)
from linkerhand.udexrealcore import UdexRealScoketUdp, TimeoutStatus, UdexRealData
from linkerhand.handcore import HandCore

from linkerhand.constants import RobotName, ROBOT_LEN_MAP


LOG_FILE_PATH = "/tmp/b.log"

class Retarget():
    def __init__(self,node, ip, port, deviceid, lefthand: RobotName, righthand: RobotName, handcore: HandCore,
                lefthandpubprint: bool, righthandpubprint: bool):
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
        elif self.righthandtype == RobotName.l25 \
            or self.righthandtype == RobotName.g20:
            from .hand.udexreal_l25 import RightHand
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

        else:
            print("未正确定义机械左手对象，请检查支持清单列表!")

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
        elif self.lefthandtype == RobotName.l25 \
            or self.lefthandtype == RobotName.g20:
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
        else:
            print("未正确定义机械右手对象，请检查支持清单列表!")

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

        # 处理左右手原始数据
        self.lefthand.joint_update(mocapdata.jointangle_lHand)
        self.righthand.joint_update(mocapdata.jointangle_rHand)

        # 速度环节处理
        self.lefthand.speed_update()
        self.righthand.speed_update()

        # self.quick_log(self.lefthand.g_jointpositions[1],LOG_FILE_PATH,"SEND")

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

    def process(self):
        """主处理函数"""
        if not self.initialize_udp():
            self.node.get_logger().error("初始化配置网络失败")
            return False
        return True
