"""
CarDriver Node for ROS 2 Mecanum Robot
Subscribes to multiplexed control commands and mode topics,
drives four motors via CarController, and publishes vehicle velocity
computed from encoder feedback. Supports dynamic mode switching
and runs all nodes on a MultiThreadedExecutor.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from driver_base.driver_base import Motor, CarController
from rclpy.executors import MultiThreadedExecutor


class CarDriver(Node):
    """
    ROS2 小车驱动节点
    """

    def __init__(self):
        """
        初始化小车控制器，包含电机和编码器
        :param wheel_diameter 轮子的直径，单位是 米 (m)
        :param ticks_per_revolution 编码器每转一圈产生的脉冲数
        :param gear_ratio 减速比
        :param max_rpm 最大电机转速(RPM)
        :param wheel_base 左右轮间距 (米)
        :param wheel_track 前后轮间距 (米)
        """
        super().__init__("driver_node")

        # 初始化默认运动模式为j模式
        self.current_mode = "j"

        # 订阅 /controller/mux
        self.sub = self.create_subscription(
            Float64MultiArray, "/controller/mux", self.control_callback, 10
        )

        self.mode_sub = self.create_subscription(String, "controller/mode", self.mode_callback, 10)

        # 声明参数
        self.declare_parameter("wheel_diameter", 0.055)
        self.declare_parameter("ticks_per_revolution", 4920)
        self.declare_parameter("gear_ratio", 20.0)
        self.declare_parameter("max_rpm", 400)
        self.declare_parameter("wheel_base", 0.22)  # 左右轮间距 (米)
        self.declare_parameter("wheel_track", 0.18)  # 前后轮间距 (米)

        # 获取参数
        wheel_diameter = self.get_parameter("wheel_diameter").value
        ticks_per_revolution = self.get_parameter("ticks_per_revolution").value
        gear_ratio = self.get_parameter("gear_ratio").value
        max_rpm = self.get_parameter("max_rpm").value
        self.wheel_base = self.get_parameter("wheel_base").value
        self.wheel_track = self.get_parameter("wheel_track").value

        self.get_logger().info("DriverNode has been started.")

        # 初始化小车控制器
        self.car_controller = self.init_car_controller(
            wheel_diameter, ticks_per_revolution, gear_ratio, max_rpm
        )

        # 创建速度发布器
        self.speed_publisher = self.create_publisher(Float64MultiArray, "/car/velocity", 10)

        # 创建定时器以发布速度
        self.create_timer(0.2, self.publish_velocity)

    def init_car_controller(self, wheel_diameter, ticks_per_revolution, gear_ratio, max_rpm):
        # 初始化四个电机（带编码器）
        motors = [
            Motor(
                pwm_pin=12,
                dir1_pin=1,
                dir2_pin=27,
                enc_a_pin=25,
                enc_b_pin=17,
                # compensate_rate=0.99446,
                compensate_rate=0.9599,
            ),
            # 左前轮
            Motor(
                pwm_pin=19,
                dir1_pin=20,
                dir2_pin=16,
                enc_a_pin=26,
                enc_b_pin=21,
                # compensate_rate=1.00000,
                compensate_rate=1.00000,
            ),
            # 左后轮
            Motor(
                pwm_pin=18,
                dir1_pin=14,
                dir2_pin=15,
                enc_a_pin=23,
                enc_b_pin=24,
                # compensate_rate=1.01992,
                compensate_rate=1.0088,
            ),
            # 右前轮
            Motor(
                pwm_pin=13,
                dir1_pin=6,
                dir2_pin=5,
                enc_a_pin=0,
                enc_b_pin=11,
                # compensate_rate=0.93280,
                compensate_rate=0.9287,
            ),
            # 右后轮
        ]

        return CarController(motors, wheel_diameter, ticks_per_revolution, gear_ratio, max_rpm)

    def mode_callback(self, msg):
        mode = msg.data.strip().lower()  # 清除空格并转为小写
        if mode in ["j", "k", "l"]:
            self.current_mode = mode
            self.get_logger().info(f"Mode updated to: {self.current_mode}")
        else:
            self.get_logger().warn(f"Unknown mode received: {mode}")

    def control_callback(self, msg):
        try:
            # x: 纵向速度 (m/s), y: 横向速度 (m/s), z: 旋转速度 (rad/s)
            x, y, z, _ = msg.data
        except ValueError:
            self.get_logger().error("Invalid control message format.")
            return

        # 根据当前模式执行对应的运动逻辑
        # if self.current_mode == 'mecanum':
        #     self.car_controller.mecanum_drive(x, y, z, self.wheel_base, self.wheel_track)
        # elif self.current_mode == 'differential':
        #     self.car_controller.differential_turn(x, y)
        # elif self.current_mode == 'axial':
        #     self.car_controller.axial_turn(x, y)
        # else:
        #     self.get_logger().warn(f"Unsupported mode: {self.current_mode}")

        if self.current_mode == "j":
            self.car_controller.mecanum_drive(x, y, z, self.wheel_base, self.wheel_track)
        elif self.current_mode == "k":
            self.car_controller.mecanum_drive(x, y, z, self.wheel_base, self.wheel_track)
        elif self.current_mode == "l":
            self.car_controller.mecanum_drive(x, y, z, self.wheel_base, self.wheel_track)
        else:
            self.get_logger().warn(f"Unsupported mode: {self.current_mode}")

        # self.get_logger().info(f"Control Command: x={x}, y={y}, z={z}, mode={self.current_mode}")

    def publish_velocity(self):
        """
        根据编码器读取每个轮子的速度，并计算小车的线速度和角速度
        """
        # 获取每个轮子的线速度
        wheel_speeds = [
            motor.get_speed(
                self.car_controller.wheel_diameter,
                self.car_controller.ticks_per_revolution,
                self.car_controller.gear_ratio,
            )
            for motor in self.car_controller.motors
        ]
        # 格式化输出列表
        # self.get_logger().info(f"Wheel Speeds: {', '.join(f'{speed:.2f}' for speed in wheel_speeds)}")

        # 根据运动模式计算线速度和角速度
        v_x, v_y, omega_z = 0.0, 0.0, 0.0

        # # 差速模式计算
        # if hasattr(self, 'current_mode') and self.current_mode == 'differential':
        #     v_x = (wheel_speeds[0] + wheel_speeds[2]) / 2  # 左前和右前轮平均速度
        # omega_z = (wheel_speeds[2] - wheel_speeds[0]) / self.wheel_track  #
        # 左右差速计算角速度

        # # 麦克纳姆模式计算
        # elif hasattr(self, 'current_mode') and self.current_mode == 'mecanum':
        # x方向速度（前后移动速度）
        v_x = (wheel_speeds[0] + wheel_speeds[1] + wheel_speeds[2] + wheel_speeds[3]) / 4
        # y方向速度（左右平移速度）
        v_y = (-wheel_speeds[0] + wheel_speeds[1] + wheel_speeds[2] - wheel_speeds[3]) / 4
        omega_z = (-wheel_speeds[0] + wheel_speeds[2] - wheel_speeds[1] + wheel_speeds[3]) / (
            2 * (self.wheel_base + self.wheel_track)
        )

        # # 轴向转向模式计算
        # elif hasattr(self, 'current_mode') and self.current_mode == 'axial':
        #     omega_z = (wheel_speeds[2] + wheel_speeds[3] - wheel_speeds[0] - wheel_speeds[1]) / (2 * self.wheel_track)

        # 构造消息并发布
        velocity = Float64MultiArray()
        velocity.data = [v_x, v_y, omega_z]  # x方向线速度, y方向线速度, 角速度
        self.speed_publisher.publish(velocity)

        # 输出日志
        self.get_logger().info(
            f"Published velocity: v_x={v_x:.6f}, v_y={v_y:.6f}, omega_z={omega_z:.6f}"
        )

    def stop(self):
        """停止所有电机"""
        self.car_controller.stop()


def main(args=None):
    """
    主函数：初始化并运行 ROS2 节点
    """
    rclpy.init(args=args)

    # 创建主节点
    car_driver = CarDriver()

    # 创建多线程执行器
    executor = MultiThreadedExecutor()

    # 将主节点加入执行器
    executor.add_node(car_driver)

    # 将每个电机的节点加入执行器
    for motor in car_driver.car_controller.motors:
        executor.add_node(motor.node)

    try:
        # 运行执行器
        executor.spin()
    except KeyboardInterrupt:
        car_driver.get_logger().info("Shutting down...")
    finally:
        # 销毁所有节点
        car_driver.destroy_node()
        for motor in car_driver.car_controller.motors:
            motor.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
