"""
ControlPublisher Node for Gamepad-Controlled Mecanum Car
Provides manual and autonomous driving modes using an Xbox joystick.
Publishes mecanum drive commands to '/controller/joy_stick' and mode switches to '/controller/mode'.
Implements automatic stop when distance or heading change exceeds configurable limits, with a wait-time before resuming.
"""

import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64, String
from controller.xbox import Joystick
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class ControlPublisher(Node):
    """
    Node:
        name: “game_pad_fixmotion”
    Subscriptions:
        - /odom (Odometry): monitor traveled distance to trigger stop when ≥ dist_limit
        - /imu (Imu): monitor yaw change to trigger stop when ≥ angle_limit
    Publications:
        - /controller/joy_stick (Float64MultiArray):
            publishes [x_speed, y_speed, rotation_speed, timestamp]
        - /controller/mode (String):
            publishes current control mode:
              'j' = manual joystick,
              'l' = autonomous navigation,
              'k' = keyboard mode
    Controls:
        Buttons:
          - B: reset autonomous mode
          - A: enter autonomous mode (clears stop_flag, resets start points)
          - X: switch to manual joystick mode ('j')
          - Y: switch to autonomous mode ('l')
          - Left bumper: switch to keyboard mode ('k')
        Joysticks:
          - Left stick: forward/lateral movement (clamped by spd_rate)
          - Right stick: rotation (clamped by rot_limit)
        D‑pad:
          - Up/Down: increase/decrease dist_limit by spd_rate
          - Left/Right: increase/decrease angle_limit by rot_limit
    Behavior:
        - Manual mode: send joystick commands unless a stop has been triggered
        - Autonomous mode:
            • automatically stop when odometry or yaw change exceeds limit,
            • wait for wait_time seconds at zero speed,
            • then reset start points and resume last commanded velocity
    """

    def __init__(self) -> None:
        super().__init__("game_pad_fixmotion")        # Declare parameters and set default values
        self.declare_parameter("spd_rate", 0.005)  # Speed limit
        self.declare_parameter("rot_limit", 0.002)  # Rotation speed limit
        self.declare_parameter("mode", "j")  # Default mode
        self.declare_parameter("dist_limit", 0.005)  # Distance limit
        self.declare_parameter("angle_limit", 0.002)  # Angle limit
        self.declare_parameter("wait_time", 0.1)  # Wait time between actions

        # Get parameter values
        self.spd_rate = self.get_parameter("spd_rate").value
        self.rot_limit = self.get_parameter("rot_limit").value
        self.mode = self.get_parameter("mode").value
        self.dist_limit = self.get_parameter("dist_limit").value
        self.angle_limit = self.get_parameter("angle_limit").value
        self.wait_time = self.get_parameter("wait_time").value

        self.get_logger().info("Game pad mecanum controller node has been started.")
        self.get_logger().info(
            f"Current parameters: spd_rate={self.spd_rate}, rot_limit={self.rot_limit}, "
            f"mode='{self.mode}', dist_limit={self.dist_limit}, angle_limit={self.angle_limit}, "
            f"wait_time={self.wait_time}"
        )

        self.publisher = self.create_publisher(
            Float64MultiArray, "/controller/joy_stick", 10
        )
        self.mode_pub = self.create_publisher(String, "/controller/mode", 10)
        self.pub_time_period = 0.02
        self.timer = self.create_timer(self.pub_time_period, self.timer_callback)
        self.joy = Joystick()
        qos_reliable = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, qos_reliable
        )
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_callback, 10)

        self.stop_start_time = (
            None  # Used to record the timestamp when stop is triggered
        )
        # -- Initialize starting position and yaw for subsequent comparisons --
        self.start_position = None
        self.start_yaw = None

        # Flags for stop event and automatic mode
        self.stop_flag = False
        self.auto_flag = False
        self.last_mode = None
        self.last_msg = None

    def odom_callback(self, msg: Odometry) -> None:
        """检测里程变化，超过 dist_limit 就停车"""
        # if not (self.auto_flag and not self.stop_flag):
        #     return

        pos = msg.pose.pose.position
        if self.start_position is None:
            self.start_position = pos
            return

        dx = pos.x - self.start_position.x
        dy = pos.y - self.start_position.y
        dist = math.hypot(dx, dy)
        if dist >= self.dist_limit:
            self.get_logger().info(
                f"Distance limit reached: {dist:.4f} >= {self.dist_limit:.4f}"
            )
            self.stop_flag = True
            # Reset starting reference for next auto cycle
            self.start_position = None
            self.start_yaw = None

    def imu_callback(self, msg: Imu) -> None:
        """检测航向角变化，超过 angle_limit 就停车"""
        # if not (self.auto_flag and not self.stop_flag):
        #     return

        q = msg.orientation
        _, _, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        if self.start_yaw is None:
            self.start_yaw = yaw
            return

        # Normalize angle difference to [-π, π]
        delta = math.atan2(
            math.sin(yaw - self.start_yaw), math.cos(yaw - self.start_yaw)
        )
        if abs(delta) >= self.angle_limit:
            self.get_logger().info(
                f"Angle limit reached: {abs(delta):.4f} >= {self.angle_limit:.4f}"
            )
            self.stop_flag = True
            # Reset starting reference for next auto cycle
            self.start_position = None
            self.start_yaw = None

    def timer_callback(self) -> None:
        # -- Use D-pad to adjust dist_limit and angle_limit dynamically --
        if self.joy.dpadUp():
            self.dist_limit += self.spd_rate
            self.get_logger().info(f"dist_limit increased to {self.dist_limit:.4f}")
        if self.joy.dpadDown():
            self.dist_limit = max(0.0, self.dist_limit - self.spd_rate)
            self.get_logger().info(f"dist_limit decreased to {self.dist_limit:.4f}")
        if self.joy.dpadLeft():
            self.angle_limit += self.rot_limit
            self.get_logger().info(f"angle_limit increased to {self.angle_limit:.4f}")
        if self.joy.dpadRight():
            self.angle_limit = max(0.0, self.angle_limit - self.rot_limit)
            self.get_logger().info(f"angle_limit decreased to {self.angle_limit:.4f}")

        # -- Buttons toggle auto_flag and control mode --
        if self.joy.B():
            self.auto_flag = False
        if self.joy.A():
            self.auto_flag = True
            self.start_position = None
            self.start_yaw = None
            self.stop_flag = False
        if self.joy.X():
            self.mode = "j"
        if self.joy.Y():
            self.mode = "l"
        if self.joy.leftBumper():
            self.mode = "k"

        # Publish mode when it changes
        if self.mode != self.last_mode:
            self.get_logger().info(f"Current mode: {self.mode}")
            self.last_mode = self.mode
            mode_msg = String()
            mode_msg.data = self.mode
            self.mode_pub.publish(mode_msg)

        now = self.get_clock().now()
        time_in_sec = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] * 1e-9

        msg = Float64MultiArray()

        # -- Unified stop logic --
        if self.stop_flag:
            if self.stop_start_time is None:
                self.stop_start_time = time_in_sec
            elapsed = time_in_sec - self.stop_start_time
            if elapsed < self.wait_time:
                msg.data = [0.0, 0.0, 0.0, time_in_sec]
                self.get_logger().info(
                    f"Limit reached, waiting {elapsed:.2f}/{self.wait_time:.2f}s"
                )
                self.publisher.publish(msg)
                return
            # After wait, reset flags and references
            self.stop_flag = False
            self.stop_start_time = None
            self.start_position = None
            self.start_yaw = None
            self.last_msg = None

        if not self.auto_flag:
            # -- Manual joystick control branch --
            y_l, x_l = self.joy.leftStick()
            y_l = -y_l
            x_r, _ = self.joy.rightStick()
            x_r = -max(-self.rot_limit, min(self.rot_limit, x_r))
            x_l = max(-self.spd_rate, min(self.spd_rate, x_l))
            y_l = max(-self.spd_rate, min(self.spd_rate, y_l))
            msg.data = [x_l, y_l, x_r, time_in_sec]

            # Stop flags are set in odom/imu callbacks
        else:
            # -- Automatic mode branch --
            prev_x, prev_y, prev_rot = (
                self.last_msg[:3] if self.last_msg else (0.0, 0.0, 0.0)
            )
            msg.data = [prev_x, prev_y, prev_rot, time_in_sec]
            self.get_logger().debug(
                f"Auto mode: continuing velocities [{prev_x}, {prev_y}, {prev_rot}]"
            )
            self.publisher.publish(msg)
            self.last_msg = msg.data
            return

        # -- Publish (manual mode) if values changed --
        if self.last_msg is None or msg.data[:3] != self.last_msg[:3]:
            self.get_logger().info(
                f"Current mode: {self.mode}, Joystick Publishing: {msg.data}"
            )
            self.publisher.publish(msg)
            self.last_msg = msg.data


def main(args=None):
    rclpy.init(args=args)
    print("Publish Joystick Control Value")
    pub = ControlPublisher()

    rclpy.spin(pub)
    # pub.publishment()

    pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
