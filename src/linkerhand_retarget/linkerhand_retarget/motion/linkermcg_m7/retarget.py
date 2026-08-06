import rclpy
from sensor_msgs.msg import JointState

from linkerhand.constants import RobotName
from linkerhand.handcore import HandCore

from .hand.direct_hand import LeftHand, RightHand, expected_dof_for_robot
from .protocol import LinkerMcgM7UdpClient


class Retarget:
    def __init__(
        self,
        node,
        ip,
        port,
        lefthand: RobotName,
        righthand: RobotName,
        handcore: HandCore,
        lefthandpubprint: bool,
        righthandpubprint: bool,
    ):
        self.node = node
        self.udp_ip = ip
        self.udp_port = int(port)
        self.lefthandtype = lefthand
        self.righthandtype = righthand
        self.handcore = handcore
        self.running = True
        self.lefthandpubprint = lefthandpubprint
        self.righthandpubprint = righthandpubprint
        self.pubprintcount = 0
        self.udp_datacapture = None
        self._last_frame_index = 0
        self._last_mismatch_log = {}
        self.loaded_hands = ()

        self.expected_dof_r = expected_dof_for_robot(self.righthandtype)
        self.expected_dof_l = expected_dof_for_robot(self.lefthandtype)
        self.righthand = RightHand(handcore, length=self.expected_dof_r)
        self.lefthand = LeftHand(handcore, length=self.expected_dof_l)

        self.publisher_r = self.node.create_publisher(
            JointState,
            "/cb_right_hand_control_cmd",
            self.handcore.hand_numjoints_r,
        )
        self.publisher_l = self.node.create_publisher(
            JointState,
            "/cb_left_hand_control_cmd",
            self.handcore.hand_numjoints_l,
        )
        self.timer = self.node.create_timer(1.0 / 120, self.process_callback)

    def initialize_udp(self) -> bool:
        self.udp_datacapture = LinkerMcgM7UdpClient(
            host=self.udp_ip,
            port=self.udp_port,
            logger=self.node.get_logger(),
        )
        if self.udp_datacapture.udp_initial():
            self.node.get_logger().info(
                f"LinkerMCG M7 UDP 初始化成功: {self.udp_ip}:{self.udp_port}"
            )
            self.running = True
            return True
        self.node.get_logger().error("LinkerMCG M7 UDP 初始化失败")
        return False

    def _hand_data_compatible(self, hand: str, values, dof: int) -> bool:
        expected = self.expected_dof_l if hand == "left" else self.expected_dof_r
        if int(dof or 0) != expected:
            self._log_mismatch_once_per_second(hand, f"dof={dof}, expected={expected}")
            return False
        if len(values) < expected:
            self._log_mismatch_once_per_second(hand, f"len={len(values)}, expected={expected}")
            return False
        return True

    def _log_mismatch_once_per_second(self, hand: str, detail: str):
        now = self.node.get_clock().now().nanoseconds / 1e9
        last = self._last_mismatch_log.get(hand, 0.0)
        if now - last >= 1.0:
            self.node.get_logger().warn(f"LinkerMCG M7 {hand} 数据与配置不匹配: {detail}")
            self._last_mismatch_log[hand] = now

    def _publish_hand(self, publisher, hand_model, values):
        hand_model.joint_update(values)
        hand_model.speed_update()
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = [f"joint{i + 1}" for i in range(len(hand_model.g_jointpositions))]
        msg.position = [float(num) for num in hand_model.g_jointpositions]
        msg.velocity = [float(num) for num in hand_model.g_jointvelocity]
        publisher.publish(msg)

    def process_callback(self):
        if not self.running or self.udp_datacapture is None:
            return

        mocapdata = self.udp_datacapture.realmocapdata
        if not mocapdata.is_update or mocapdata.frame_index == self._last_frame_index:
            return
        self._last_frame_index = mocapdata.frame_index

        loaded_hands = []
        if self._hand_data_compatible("left", mocapdata.jointangle_lHand, mocapdata.dof):
            self._publish_hand(self.publisher_l, self.lefthand, mocapdata.jointangle_lHand)
            loaded_hands.append("left")
            if self.lefthandpubprint and self.pubprintcount % 30 == 0:
                self.node.get_logger().info(
                    f"[LinkerMCG M7] 左手 {mocapdata.hand_type}: {self.lefthand.g_jointpositions}"
                )

        if self._hand_data_compatible("right", mocapdata.jointangle_rHand, mocapdata.dof):
            self._publish_hand(self.publisher_r, self.righthand, mocapdata.jointangle_rHand)
            loaded_hands.append("right")
            if self.righthandpubprint and self.pubprintcount % 30 == 0:
                self.node.get_logger().info(
                    f"[LinkerMCG M7] 右手 {mocapdata.hand_type}: {self.righthand.g_jointpositions}"
                )

        self.loaded_hands = tuple(loaded_hands)
        self.pubprintcount += 1

    def process(self):
        return self.initialize_udp()

    def stop_serial_threads(self):
        self.running = False
        if self.udp_datacapture is not None:
            self.udp_datacapture.udp_close()

    def set_mode(self, mode, param=None):
        self.node.get_logger().info(f"LinkerMCG M7 当前仅支持默认 glove 模式: {mode}")
