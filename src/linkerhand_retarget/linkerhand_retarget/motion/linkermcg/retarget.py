import time
import rclpy
import sys
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pathlib import Path

_project_root = Path(__file__).absolute().parent.parent.parent
_project_root_str = str(_project_root)

if _project_root_str in sys.path:
    sys.path.remove(_project_root_str)
sys.path.insert(0, _project_root_str)


from linkerhand.linkermcgcore import HaoCunScoketUdp
from linkerhand.handcore import HandCore
from linkerhand.constants import RobotName, ROBOT_LEN_MAP



class Retarget():
    def __init__(self,node, ip, port, lefthand: RobotName, righthand: RobotName, handcore: HandCore,
                lefthandpubprint: bool, righthandpubprint: bool):
        self.node = node
        self.udp_ip = ip
        self.udp_port = port
        self.lefthandtype = lefthand
        self.righthandtype = righthand
        self.handcore = handcore
        self.running = True
        self.lefthandpubprint = lefthandpubprint
        self.righthandpubprint = righthandpubprint
        
        # 根据右手类型初始化
        if self.righthandtype == RobotName.o7 \
            or self.righthandtype == RobotName.l7 \
            or self.righthandtype == RobotName.o7v1 \
            or self.righthandtype == RobotName.o7v3:
            from .hand.linkermcg_l7 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.o6:
            from .hand.linkermcg_o6 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l6:
            from .hand.linkermcg_l6 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l25 \
            or self.righthandtype == RobotName.g20:
            from .hand.linkermcg_l25 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        # elif self.righthandtype == RobotName.t25:
        #     from .hand.linkermcg_t25 import RightHand
        #     self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l20:
            from .hand.linkermcg_l20 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l10 \
            or self.righthandtype == RobotName.l10v7 :
            from .hand.linkermcg_l10v7 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l21:
            from .hand.linkermcg_l21 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])

        # 根据LEFT手类型初始化
        if self.lefthandtype == RobotName.o7 \
            or self.lefthandtype == RobotName.l7 \
            or self.lefthandtype == RobotName.o7v1 \
            or self.lefthandtype == RobotName.o7v3:
            from .hand.linkermcg_l7 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.o6:
            from .hand.linkermcg_o6 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l6:
            from .hand.linkermcg_l6 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l25 \
            or self.lefthandtype == RobotName.g20:
            from .hand.linkermcg_l25 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        # elif self.lefthandtype == RobotName.t25:
        #     from .hand.linkermcg_t25 import LeftHand
        #     self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l20:
            from .hand.linkermcg_l20 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l10 \
            or self.lefthandtype == RobotName.l10v7 :
            from .hand.linkermcg_l10v7 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l21:
            from .hand.linkermcg_l21 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])

        self.publisher_r = self.node.create_publisher(
            JointState,
            '/cb_right_hand_control_cmd',
            self.handcore.hand_numjoints_r)
            
        self.publisher_l = self.node.create_publisher(
            JointState,
            '/cb_left_hand_control_cmd',
            self.handcore.hand_numjoints_l)
        
        # 创建ROS定时器，以固定频率处理数据
        # 参数1: period 周期(秒)
        # 参数2: callback 回调函数
        # 参数3: oneshot 是否只执行一次
        self.timer = self.node.create_timer(1.0/120, self.process_callback)  # 120Hz
        self.pubprintcount = 0
        self.pubprintcount = 0

        self.pubprintcount = 0
        self.udp_datacapture = None

    def initialize_udp(self):
        """初始化UDP连接"""
        self.udp_datacapture = HaoCunScoketUdp(
            host=self.udp_ip,
            port=self.udp_port)
        if self.udp_datacapture.udp_initial():
            self.node.get_logger().info("UDP连接初始化成功")
            self.running = True
        else:
            self.node.get_logger().error("UDP连接初始化失败")
    
    def process_callback(self):
        if not self.running:
            return
    
        mocapdata = self.udp_datacapture.realmocapdata
        if not mocapdata.is_update:
            return
                    
        # 处理左右手原始数据
        self.lefthand.joint_update(mocapdata.jointangle_lHand)
        self.righthand.joint_update(mocapdata.jointangle_rHand)

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

    def process(self):
        """主处理函数"""
        self.initialize_udp()
        try:
            while rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except rclpy.ROSInterruptException:
            pass