
import time
import sapien
import tyro
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from datetime import datetime
from pathlib import Path
from loguru import logger
from sapien.asset import create_dome_envmap
from sapien.utils import Viewer

from ...linkerhand.handcore import HandCore
from ...linkerhand.constants import (
    RetargetingType, 
    DataSource, 
    ROBOT_LEN_MAP,
    MotionSource, 
    RobotName, 
    HandType, 
    get_default_config_path,
)
from ...linkerhand.retargeting_config import RetargetingConfig

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
            from .hand.simulator_l7 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.o6:
            from .hand.simulator_o6 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l6:
            from .hand.simulator_l6 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.o20:
            from .hand.simulator_o20 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.o30:
            from .hand.simulator_o30 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        # elif self.righthandtype == RobotName.l25:
        #     from .hand.simulator_l25 import RightHand
        #     self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        # elif self.righthandtype == RobotName.t25:
        #     from .hand.simulator_t25 import RightHand
        #     self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l20:
            from .hand.simulator_l20 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l10 \
            or self.righthandtype == RobotName.l10v7 :
            from .hand.simulator_l10v7 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l21:
            from .hand.simulator_l21 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])

        # 根据LEFT手类型初始化
        if self.lefthandtype == RobotName.o7 \
            or self.lefthandtype == RobotName.l7 \
            or self.lefthandtype == RobotName.o7v1 \
            or self.lefthandtype == RobotName.o7v3:
            from .hand.simulator_l7 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.o6:
            from .hand.simulator_o6 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l6:
            from .hand.simulator_l6 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.o20:
            from .hand.simulator_o20 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.o30:
            from .hand.simulator_o30 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l25:
            from .hand.simulator_l25 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        # elif self.lefthandtype == RobotName.t25:
        #     from .hand.simulator_t25 import LeftHand
        #     self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l20:
            from .hand.simulator_l20 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l10 \
            or self.lefthandtype == RobotName.l10v7 :
            from .hand.simulator_l10v7 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l21:
            from .hand.simulator_l21 import LeftHand
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

    def sapien_init(self):
        sapien.render.set_viewer_shader_dir("default")
        sapien.render.set_camera_shader_dir("default")

        config = RetargetingConfig.load_from_file(self.config_path)

        # Setup
        scene = sapien.Scene()
        render_mat = sapien.render.RenderMaterial()
        render_mat.base_color = [0.06, 0.08, 0.12, 1]
        render_mat.metallic = 0.0
        render_mat.roughness = 0.9
        render_mat.specular = 0.8
        scene.add_ground(-0.2, render_material=render_mat, render_half_size=[1000, 1000])

        # Lighting
        scene.add_directional_light(np.array([1, 1, -1]), np.array([3, 3, 3]))
        scene.add_point_light(np.array([2, 2, 2]), np.array([2, 2, 2]), shadow=False)
        scene.add_point_light(np.array([2, -2, 2]), np.array([2, 2, 2]), shadow=False)
        scene.set_environment_map(
            create_dome_envmap(sky_color=[0.2, 0.2, 0.2], ground_color=[0.2, 0.2, 0.2])
        )
        scene.add_area_light_for_ray_tracing(
            sapien.Pose([2, 1, 2], [0.707, 0, 0.707, 0]), np.array([1, 1, 1]), 5, 5
        )

        # Camera
        cam = scene.add_camera(
            name="Cheese!", width=600, height=600, fovy=1, near=0.1, far=10
        )
        cam.set_local_pose(sapien.Pose([0.50, 0, 0.0], [0, 0, 0, -1]))

        self.viewer = Viewer()
        self.viewer.set_scene(scene)
        self.viewer.control_window.show_origin_frame = False
        self.viewer.control_window.move_speed = 0.01
        self.viewer.control_window.toggle_camera_lines(False)
        self.viewer.set_camera_pose(cam.get_local_pose())

        # Load robot and set it to a good pose to take picture
        loader = scene.create_urdf_loader()
        filepath = Path(config.urdf_path)
        robot_name = filepath.stem
        loader.load_multiple_collisions_from_file = True
        loader.scale = 1.5

        filepath = str(filepath)
        self.robot = loader.load(filepath)
        self.robot.set_pose(sapien.Pose([0, 0, -0.13]))

        # Different robot loader may have different orders for joints
        self.sapien_joint_names = [joint.get_name() for joint in self.robot.get_active_joints()]
        retargeting_joint_names = self.retargeting.joint_names
        retargeting_to_sapien = np.array(
            [retargeting_joint_names.index(name) for name in self.sapien_joint_names]
        ).astype(int)

        self.qpos = [0] * len(self.sapien_joint_names)

    def process_callback(self):
        self.viewer.render()
        
        """定时器回调函数，处理数据并发布"""
        if not self.udp_datacapture or not self.udp_datacapture.udp_is_onnect():
            self.node.get_logger().warning("侦测到UDP断开状态，正在重连！")
            if not self.initialize_udp():
                self.node.get_logger().error("UDP重连失败")
                return
            time.sleep(2)
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
            return
