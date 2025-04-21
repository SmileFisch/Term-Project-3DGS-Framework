"""
ControlPublisher Node for Xbox Gamepad–Controlled Mecanum Drive
Provides manual joystick and algorithmic control modes.
Publishes drive commands [x_speed, y_speed, rotation_speed, timestamp] to '/controller/joy_stick'.
Publishes current mode ('j' = joystick, 'l' = algorithm, 'k' = keyboard) to '/controller/mode'.
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64, String
from controller.xbox import Joystick


class ControlPublisher(Node):
    """
    Node:
        - name: Game pad controller for mechanum control
        - function:
            - B button: stop car
            - A button: restart car
            - X button: manual control mode
            - Y button: autonomous driving mode
            - Left stick: driving signal
            - Right stick: turning in mecanum
        - Subscription: None
        - Publisher:
            - /controller/joy_stick
                - type: Float64MultiArray
                - description: publish control value and time
            - /controller/mode
                - type: String
                - description: publish mode, j for joy-con, l for self navigation
    """

    def __init__(self) -> None:
        super().__init__("game_pad_mecanum")
        # Declare parameters and set default values
        self.declare_parameter("spd_rate", 0.025)  # Speed limit
        self.declare_parameter("rot_limit", 0.01)  # Rotation speed limit
        self.declare_parameter("mode", "j")  # Default mode

        # Get parameter values
        self.spd_rate = self.get_parameter("spd_rate").value
        self.rot_limit = self.get_parameter("rot_limit").value
        self.mode = self.get_parameter("mode").value

        self.get_logger().info("Game pad mecanum controller node has been started.")
        self.publisher = self.create_publisher(Float64MultiArray, "/controller/joy_stick", 10)
        self.mode_pub = self.create_publisher(String, "/controller/mode", 10)
        self.pub_time_period = 0.02
        self.timer = self.create_timer(self.pub_time_period, self.timer_callback)
        self.joy = Joystick()
        self.stop_flag = False
        self.last_mode = None  # 用于检测模式变化
        self.last_msg = None  # 保存上一次的消息

    """
    def publishment(self) -> None:

        # To publish topic as fast as possible (rate: 4000Hz)

        while rclpy.ok():

            if self.joy.B():
                self.stop_flag = True
            if self.joy.A():
                self.stop_flag = False
            msg = Float64MultiArray()
            x, y = self.joy.leftStick()
            t = time.time()
            if self.stop_flag:
                msg.data = [0.0, 0.0, t]
            else:
                msg.data = [self.spd_rate*x, self.spd_rate*y, t]
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0)
            # time.sleep(0.02)
    """

    def timer_callback(self) -> None:
        # publish control topic with a fixed rate
        if self.joy.B():
            self.stop_flag = True
        if self.joy.A():
            self.stop_flag = False
        if self.joy.X():
            self.mode = "j"  # joy stick mode
        if self.joy.Y():
            self.mode = "l"  # algorithm
        if self.joy.leftBumper():
            self.mode = "k"  # keyboard

        # 如果模式发生变化，打印当前模式
        if self.mode != self.last_mode:
            self.get_logger().info(f"Current mode: {self.mode}")
            self.last_mode = self.mode
            mode_msg = String()
            mode_msg.data = self.mode
            self.mode_pub.publish(mode_msg)

        msg = Float64MultiArray()

        y_l, x_l = self.joy.leftStick()
        y_l = -1 * y_l
        x_r, _ = self.joy.rightStick()
        x_r = max(-self.rot_limit, min(self.rot_limit, x_r))  # 限制旋转速度范围
        x_r = -1 * x_r
        # 限制 x_l 和 y_l 的范围在正负 self.spd_rate 之间
        x_l = max(-self.spd_rate, min(self.spd_rate, x_l))
        y_l = max(-self.spd_rate, min(self.spd_rate, y_l))

        # 使用 ROS 时间系统
        current_time = self.get_clock().now()
        time_in_sec = (
            current_time.seconds_nanoseconds()[0] + current_time.seconds_nanoseconds()[1] * 1e-9
        )

        if self.stop_flag:
            msg.data = [0.0, 0.0, 0.0, time_in_sec]
            self.get_logger().info("Stop Now")
        else:
            # Check for directional pad inputs for fine adjustments
            if self.joy.dpadUp():
                x_l = min(x_l + 0.05, self.spd_rate)  # 向前移动（纵向速度增加）
            if self.joy.dpadDown():
                x_l = max(x_l - 0.05, -self.spd_rate)  # 向后移动（纵向速度减少）
            if self.joy.dpadLeft():
                y_l = min(y_l + 0.05, self.spd_rate)  # 向左移动（横向速度增加，V_y > 0）
            if self.joy.dpadRight():
                y_l = max(y_l - 0.05, -self.spd_rate)  # 向右移动（横向速度减少，V_y < 0）

            msg.data = [x_l, y_l, x_r, time_in_sec]

        # Log the values being published
        # 比较当前消息和上次发布的消息
        if self.last_msg is None or msg.data != self.last_msg:
            # Log the values being published
            self.get_logger().info(f"Current mode: {self.mode}, Joystick Publishing: {msg.data}")
            self.publisher.publish(msg)
            self.last_msg = msg.data  # 更新上次消息


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
