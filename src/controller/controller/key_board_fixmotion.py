"""
ControlPublisher Node for Keyboard–Controlled Mecanum Drive
Publishes keyboard-derived drive commands [x_speed, y_speed, rotation_speed, timestamp] to '/controller/key_board'.
Publishes current control mode ('j' = joystick fallback, 'k' = keyboard, 'l' = algorithmic) to '/controller/mode'.
Subscribes to '/odom' (Odometry) and '/imu' (Imu) to detect when movement exceeds thresholds (1 cm or 1°) and auto-stop.
Periodically (0.2 s) reads single-key presses:
  • w/a/s/d: move forward/left/backward/right
  • q/e: rotate left/right
  • other keys: apply gradual decay to velocities
  • j/k/l: switch modes
"""

import time
import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math


class ControlPublisher(Node):

    def __init__(self) -> None:
        super().__init__("key_board")
        self.get_logger().info("Key board mecanum controller node has been started.")

        qos_reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.control_pub = self.create_publisher(Float64MultiArray, "/controller/key_board", 10)
        self.mode_pub = self.create_publisher(String, "/controller/mode", 1)
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, qos_reliable
        )
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_callback, 10)

        self.timer = self.create_timer(0.2, self.timer_callback)
        self.mode = "j"
        self.last_mode = None  # 用于检测模式变化
        self.control_val = [0, 0, 0, time.time()]  # x, y, r, t
        self.spd_limit = 0.03
        self.rot_limit = 0.01  # 旋转速度限制

        self.start_position = None  # 初始位置 (x, y)
        self.start_yaw = None  # 初始航向角度
        self.movement_active = False

    def get_key_press(self):
        fd = sys.stdin.fileno()
        og_attr = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, og_attr)
        return key

    def odom_callback(self, msg):
        if self.start_position is None:
            self.start_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        else:
            x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
            dx, dy = x - self.start_position[0], y - self.start_position[1]
            distance = math.sqrt(dx**2 + dy**2)
            if distance >= 0.01 and self.movement_active:  # 1cm
                self.stop_movement()

    def imu_callback(self, msg):
        if self.start_yaw is None:
            _, _, self.start_yaw = self.quaternion_to_euler(msg.orientation)
        else:
            _, _, current_yaw = self.quaternion_to_euler(msg.orientation)
            if abs(current_yaw - self.start_yaw) >= math.radians(1) and self.movement_active:  # 1°
                self.stop_movement()

    def quaternion_to_euler(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return (0, 0, yaw)

    def stop_movement(self):
        self.control_val = [0, 0, 0, time.time()]
        self.movement_active = False
        self.start_position = None
        self.start_yaw = None
        self.get_logger().info("Target reached: Stopping movement.")

    def timer_callback(self):
        con_msg = Float64MultiArray()
        mode_msg = String()
        control_value = self.get_key_press()
        mode_value = control_value

        if not self.movement_active:
            self.start_position = None  # 重置初始位置
            self.start_yaw = None  # 重置初始角度
            self.movement_active = True

        match control_value:
            case "w":
                self.control_val[1] = self.spd_limit
            case "a":
                self.control_val[0] = -self.spd_limit
            case "s":
                self.control_val[1] = -self.spd_limit
            case "d":
                self.control_val[0] = self.spd_limit
            case "q":
                self.control_val[2] = -self.rot_limit
            case "e":
                self.control_val[2] = self.rot_limit
            case _:
                self.control_val[0] *= 0.9
                self.control_val[1] *= 0.9
                self.control_val[2] *= 0.9

        match mode_value:
            case "j":
                self.mode = "j"
            case "k":
                self.mode = "k"
            case "l":
                self.mode = "l"

        if self.mode != self.last_mode:
            self.get_logger().info(f"Current mode: {self.mode}")
            self.last_mode = self.mode

        self.control_val[3] = time.time()
        con_msg.data = self.control_val
        mode_msg.data = self.mode
        self.control_pub.publish(con_msg)
        self.mode_pub.publish(mode_msg)


def main(args=None):
    rclpy.init(args=args)
    print("Publish Keyboard Control Value")
    pub = ControlPublisher()
    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
