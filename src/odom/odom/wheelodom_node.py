"""
MecanumOdometryNode for ROS 2
- Subscribes to IMU and four wheel encoder pulse counts
- Computes robot pose (x, y, θ) and velocities (vx, vy, ω)
- Fuses encoder‐based heading with IMU orientation
- Publishes nav_msgs/Odometry on '/odom' and broadcasts TF odom→base_link
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from geometry_msgs.msg import Quaternion
import tf_transformations
import numpy as np
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration


class MecanumOdometryNode(Node):
    def __init__(self):
        super().__init__("mecanum_odometry_node")

        # Parameters
        self.declare_parameter("wheel_diameter", 0.055)
        self.declare_parameter("ticks_per_revolution", 4920)
        self.declare_parameter("gear_ratio", 20.0)
        self.declare_parameter("wheel_base", 0.22)
        self.declare_parameter("wheel_track", 0.18)
        self.declare_parameter("publish_frequency", 5)

        # Initialize parameters
        self.wheel_radius = self.get_parameter("wheel_diameter").value / 2.0
        self.encoder_ppr = self.get_parameter("ticks_per_revolution").value
        self.wheel_base_x = self.get_parameter("wheel_base").value
        self.wheel_base_y = self.get_parameter("wheel_track").value
        self.publish_frequency = self.get_parameter("publish_frequency").value

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.theta_imu = 0.0
        self.last_time = self.get_clock().now()

        self.previous_pulses = [0.0, 0.0, 0.0, 0.0]  # 存储每个轮子的上一次脉冲数
        self.last_time_encoders = [self.get_clock().now()] * 4  # 每个轮子的上一次更新时间

        # TF buffer and listener
        # 初始化 TF Buffer 和 Listener
        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))  # 设置缓存时间为 20 秒
        self.tf_listener = TransformListener(self.tf_buffer, self)  # 绑定监听器

        # QoS configuration for BEST_EFFORT
        qos_best_effort = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=20, reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # QoS configuration for publishers (RELIABLE)
        qos_reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE
        )

        # Subscribers
        self.create_subscription(Imu, "/imu", self.imu_callback, qos_best_effort)
        self.create_subscription(
            Int32, "/encoder/pulse_count_1", self.encoder_callback_1, qos_best_effort
        )
        self.create_subscription(
            Int32, "/encoder/pulse_count_2", self.encoder_callback_2, qos_best_effort
        )
        self.create_subscription(
            Int32, "/encoder/pulse_count_3", self.encoder_callback_3, qos_best_effort
        )
        self.create_subscription(
            Int32, "/encoder/pulse_count_4", self.encoder_callback_4, qos_best_effort
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "/odom", qos_reliable)

        # Timer
        self.create_timer(1.0 / self.publish_frequency, self.update_odometry)

        # Wheel speeds
        self.wheel_speeds = [0.0, 0.0, 0.0, 0.0]
        self.imu_angular_velocity_z = 0
        self.base_link_quaternion = [0.0, 0.0, 0.0, 1.0]  # 默认四元数（单位四元数）
        self.initial_base_link_quaternion = None
        self.initial_base_link_quaternion_samples = []

        # 初始化 TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("MecanumOdometryNode has been started.")

    def imu_callback(self, msg):
        # Transform IMU orientation from imu_frame to base_link
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link",
                msg.header.frame_id,
                rclpy.time.Time(),  # 查询最近可用的变换
                timeout=Duration(seconds=0.2),  # 设置 0.2 秒的时间容差
            )
            imu_quaternion = [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            ]
            imu_angular_velocity = np.array(
                [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
            )
            # 从 transform 提取旋转四元数
            transform_quaternion = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]
            # Transform IMU orientation to base_link
            self.base_link_quaternion = tf_transformations.quaternion_multiply(
                transform_quaternion, imu_quaternion
            )
            # 如果初始四元数未设置，则保存当前值作为初始值
            if self.initial_base_link_quaternion is None:
                self.initial_base_link_quaternion_samples.append(self.base_link_quaternion)
                if len(self.initial_base_link_quaternion_samples) >= 20:  # 例如采样10次
                    average_quaternion = np.mean(self.initial_base_link_quaternion_samples, axis=0)
                    self.initial_base_link_quaternion = average_quaternion / np.linalg.norm(
                        average_quaternion
                    )  # 归一化
                    self.get_logger().info(
                        f"Initial base_link quaternion: {self.initial_base_link_quaternion}"
                    )

            # 计算从 transform_quaternion 得到的旋转矩阵
            rotation_matrix = tf_transformations.quaternion_matrix(transform_quaternion)[:3, :3]

            # 变换角速度到 base_link 坐标系
            base_link_angular_velocity = rotation_matrix @ imu_angular_velocity

            # 保存变换后的角速度 (rad/s)
            self.imu_angular_velocity_z = base_link_angular_velocity[2]

            # _, _, self.theta_imu = tf_transformations.euler_from_quaternion(self.base_link_quaternion)

            # 打印成功的变换信息
            # self.get_logger().info(f"Transform successful!")
            # self.get_logger().info(f"Transform quaternion: {transform_quaternion}")
            # self.get_logger().info(f"IMU quaternion: {imu_quaternion}")
            # self.get_logger().info(f"Base link quaternion: {self.base_link_quaternion}")
            # self.get_logger().info(f"theta: {self.theta_imu}")
            # self.get_logger().info(f"IMU angular velocity z: {self.imu_angular_velocity_z:.4f} rad/s")
        except Exception as e:
            self.get_logger().warn(f"Transform warning at time {msg.header.stamp}: {e}")

    def get_relative_quaternion(self):
        if self.initial_base_link_quaternion is None:
            self.get_logger().warn(
                "Initial base_link quaternion not set. Returning default quaternion [0, 0, 0, 1]."
            )
            return (0.0, 0.0, 0.0, 1.0)

        # 计算初始四元数的逆
        initial_inv = [
            -self.initial_base_link_quaternion[0],
            -self.initial_base_link_quaternion[1],
            -self.initial_base_link_quaternion[2],
            self.initial_base_link_quaternion[3],
        ]

        # 计算当前相对四元数
        relative_quaternion = tf_transformations.quaternion_multiply(
            initial_inv, self.base_link_quaternion
        )

        # 转换为 numpy.array
        relative_quaternion = np.array(relative_quaternion)

        # 对小值进行归零处理
        vector_norm = np.linalg.norm(relative_quaternion[:3])
        threshold = 1e-4
        if vector_norm < threshold:
            relative_quaternion[:3] = [0.0, 0.0, 0.0]
            relative_quaternion[3] = 1.0
        else:
            relative_quaternion[:3] *= (vector_norm - threshold) / vector_norm  # 平滑衰减

        return tuple(relative_quaternion)

    def get_imu_angular_velocity(self):
        # 需要在 IMU 回调中保存角速度
        try:
            return self.imu_angular_velocity_z
        except AttributeError:
            # 如果尚未接收到 IMU 数据，返回 0，并记录日志
            self.get_logger().info(
                "IMU data not received yet. Returning default angular velocity: 0.0"
            )
            return 0.0

    def encoder_callback_1(self, msg):  # 左前
        self.update_wheel_speed(0, msg.data)
        # self.get_logger().info("/encoder/pulse_count_1 start!")

    def encoder_callback_2(self, msg):  # 左后
        self.update_wheel_speed(1, msg.data)
        # self.get_logger().info("/encoder/pulse_count_2 start!")

    def encoder_callback_3(self, msg):  # 右前
        self.update_wheel_speed(2, msg.data)
        # self.get_logger().info("/encoder/pulse_count_3 start!")

    def encoder_callback_4(self, msg):  # 右后
        self.update_wheel_speed(3, msg.data)
        # self.get_logger().info("/encoder/pulse_count_4 start!")

    def update_wheel_speed(self, wheel_index, current_pulse):
        # 获取当前时间
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time_encoders[wheel_index]).nanoseconds / 1e9

        # 计算增量脉冲数
        delta_pulse = current_pulse - self.previous_pulses[wheel_index]
        # 显示 delta_pulse 的值
        # self.get_logger().info(f"Wheel {wheel_index}: Delta Pulse = {delta_pulse}, DT = {dt}s")

        # 更新轮子的角速度
        self.wheel_speeds[wheel_index] = self.pulse_to_rpm(delta_pulse, dt)

        # 更新上一次脉冲数和时间
        self.previous_pulses[wheel_index] = current_pulse
        self.last_time_encoders[wheel_index] = current_time

    def pulse_to_rpm(self, delta_pulse, dt):
        if dt <= 0:
            self.get_logger().warn("Invalid dt value: dt must be greater than 0. Returning 0.0.")
            return 0.0
        pulse_rate = delta_pulse / dt  # 脉冲频率 (脉冲/秒)
        angular_velocity = (pulse_rate / self.encoder_ppr) * 2 * np.pi  # 转为角速度 (rad/s)

        # 打印角速度
        # self.get_logger().info(f"Calculated angular velocity: {angular_velocity:.6f} rad/s")

        return angular_velocity

    def fuse_quaternions(self, q_computed, q_imu):
        # 计算点积
        dot_product = np.dot(q_computed, q_imu)

        # 限制点积在 [-1, 1] 范围内，避免数值误差
        dot_product = max(min(dot_product, 1.0), -1.0)

        # 根据点积计算融合权重
        alpha = abs(dot_product)  # 例如直接使用点积绝对值作为权重

        # 按比例融合四元数
        q_fused = alpha * np.array(q_computed) + (1 - alpha) * np.array(q_imu)

        # 归一化融合后的四元数
        q_fused /= np.linalg.norm(q_fused)
        return q_fused

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 计算速度
        R = self.wheel_radius
        Lx = self.wheel_base_x / 2
        Ly = self.wheel_base_y / 2

        omega_lf, omega_lb, omega_rf, omega_rb = self.wheel_speeds

        velocity_matrix = np.array(
            [
                [1, 1, 1, 1],  # X 方向速度
                [-1, 1, 1, -1],  # Y 方向速度
                [-1 / (Ly + Lx), -1 / (Ly + Lx), 1 / (Ly + Lx), 1 / (Ly + Lx)],  # 转动速度
            ]
        )
        assert velocity_matrix.shape == (3, 4), "Velocity matrix dimensions must be 3x4."

        wheel_speeds = np.array([omega_lf, omega_lb, omega_rf, omega_rb]) * R
        velocity = np.dot(velocity_matrix, wheel_speeds) / 4.0

        vx, vy, omega = velocity
        # 打印当前速度和角速度信息
        self.get_logger().info(
            f"Calculated velocity -> vx: {vx:.6f} m/s, vy: {vy:.6f} m/s, omega: {omega:.6f} rad/s"
        )
        # # 获取 IMU 提供的角速度
        imu_angular_velocity = self.get_imu_angular_velocity()
        # # 打印 IMU 提供的角速度信息
        self.get_logger().info(
            f"IMU angular velocity -> imu_angular_velocity_z: {imu_angular_velocity:.6f} rad/s"
        )

        # 融合角速度：简单加权平均法
        alpha = 0.90  # 权重，可以根据系统性能调整
        omega = alpha * omega + (1 - alpha) * imu_angular_velocity

        if abs(omega) < 1e-5:
            omega = 0.0

        if abs(vx) < 1e-5:
            vx = 0.0

        if abs(vy) < 1e-5:
            omega = 0.0

        # 更新位置
        self.x += (vx * np.cos(self.theta) - vy * np.sin(self.theta)) * dt
        self.y += (vx * np.sin(self.theta) + vy * np.cos(self.theta)) * dt
        self.theta += omega * dt

        # 发布里程计消息
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y

        # 设置四元数
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)

        # 打印计算出的四元数和 IMU 转换后的四元数
        self.get_logger().info(
            f"Computed quaternion (q): [{q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}, {q[3]:.6f}]"
        )

        # 获取相对四元数
        relative_quaternion = self.get_relative_quaternion()
        self.get_logger().info(
            f"IMU base_link_relative_quaternion: [{relative_quaternion[0]:.6f}, {relative_quaternion[1]:.6f}, "
            f"{relative_quaternion[2]:.6f}, {relative_quaternion[3]:.6f}]"
        )

        # 融合四元数
        q_fuse = self.fuse_quaternions(q, relative_quaternion)
        # q_fuse = self.fuse_quaternions(q, q)

        odom_msg.pose.pose.orientation.x = q_fuse[0]
        odom_msg.pose.pose.orientation.y = q_fuse[1]
        odom_msg.pose.pose.orientation.z = q_fuse[2]
        odom_msg.pose.pose.orientation.w = q_fuse[3]

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = omega

        self.odom_pub.publish(odom_msg)
        # 使用 logger 记录发布的里程计消息
        # self.get_logger().info(
        #     f"Odom published: Position -> x: {self.x:.6f}, y: {self.y:.6f}, theta: {self.theta:.6f} rad; "
        #     f"Velocity -> vx: {vx:.6f} m/s, vy: {vy:.6f} m/s, omega: {omega:.6f} rad/s"
        # )

        # 在 update_odometry 中广播 TF
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        # 使用四元数
        transform.transform.rotation.x = q_fuse[0]
        transform.transform.rotation.y = q_fuse[1]
        transform.transform.rotation.z = q_fuse[2]
        transform.transform.rotation.w = q_fuse[3]

        self.tf_broadcaster.sendTransform(transform)
        # 使用 logger 记录广播的 TF 信息
        # self.get_logger().info(
        #     f"TF broadcasted: Parent frame: 'odom', Child frame: 'base_link', "
        #     f"Translation -> x: {self.x:.6f}, y: {self.y:.6f}, z: 0.0; "
        #     f"Rotation -> qx: {q[0]:.6f}, qy: {q[1]:.6f}, qz: {q[2]:.6f}, qw: {q[3]:.6f}"
        # )


def main(args=None):
    rclpy.init(args=args)
    node = MecanumOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
