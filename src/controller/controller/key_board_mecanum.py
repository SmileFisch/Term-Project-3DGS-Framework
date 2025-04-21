"""
ControlPublisher Node for Keyboard–Controlled Mecanum Drive
Publishes keyboard-derived drive commands [x_speed, y_speed, rotation_speed, timestamp] to '/controller/key_board'.
Publishes current control mode ('j' = joystick fallback, 'k' = keyboard, 'l' = algorithmic) to '/controller/mode'.
Every 0.2 s reads one key:
  • w/a/s/d: incrementally adjust forward/left/backward/right velocity within spd_limit
  • q/e: incrementally adjust rotation speed within rot_limit (counter-/clockwise)
  • other keys: apply exponential decay to all velocity components
  • j/k/l: switch control mode
"""

import time
import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray


class ControlPublisher(Node):

    def __init__(self) -> None:
        super().__init__("key_board")
        self.get_logger().info("Key board mecanum controller node has been started.")
        self.control_pub = self.create_publisher(Float64MultiArray, "/controller/key_board", 10)
        self.mode_pub = self.create_publisher(String, "/controller/mode", 1)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.mode = "j"
        self.last_mode = None  # 用于检测模式变化
        self.t = time.time()
        self.control_val = [0, 0, 0, self.t]  # x, y, r, t
        self.stop_flag = False
        self.spd_limit = 0.03
        self.rot_limit = 0.01  # 设置旋转速度限制

    def get_key_press(self):
        # get key input
        fd = sys.stdin.fileno()
        og_attr = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, og_attr)
        return key

    def timer_callback(self):
        # publish
        con_msg = Float64MultiArray()
        mode_msg = String()
        control_value = self.get_key_press()
        mode_value = control_value

        # 更新速度逻辑
        match control_value:
            case "w":  # 向前，增加 y 方向速度
                self.control_val[1] = min(
                    self.control_val[1] + 0.05, self.spd_limit
                )  # 增加速度但不超过上限
            case "a":  # 向左，减少 x 方向速度
                self.control_val[0] = max(
                    self.control_val[0] - 0.05, -self.spd_limit
                )  # 减速但不低于下限
            case "s":  # 向后，减少 y 方向速度
                self.control_val[1] = max(
                    self.control_val[1] - 0.05, -self.spd_limit
                )  # 减速但不低于下限
            case "d":  # 向右，增加 x 方向速度
                self.control_val[0] = min(
                    self.control_val[0] + 0.05, self.spd_limit
                )  # 增加速度但不超过上限
            case "q":  # 逆时针旋转
                self.control_val[2] = max(
                    self.control_val[2] - 0.02, -self.rot_limit
                )  # 减少旋转速度但不低于下限
            case "e":  # 顺时针旋转
                self.control_val[2] = min(
                    self.control_val[2] + 0.02, self.rot_limit
                )  # 增加旋转速度但不超过上限
            case _:
                # 没有按下方向键或旋转键，速度和旋转速度逐步归零
                self.control_val[0] *= 0.9  # x 方向速度缓慢衰减
                self.control_val[1] *= 0.9  # y 方向速度缓慢衰减
                self.control_val[2] *= 0.9  # 旋转速度缓慢衰减

        # 更新模式逻辑
        match mode_value:
            case "j":  # 摇杆模式
                self.mode = "j"
            case "k":  # 键盘模式
                self.mode = "k"
            case "l":  # algorithm模式
                self.mode = "l"

        # 如果模式发生变化，打印当前模式
        if self.mode != self.last_mode:
            self.get_logger().info(f"Current mode: {self.mode}")
            self.last_mode = self.mode

        # 更新时间戳
        self.control_val[3] = time.time()

        # 发布消息
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
