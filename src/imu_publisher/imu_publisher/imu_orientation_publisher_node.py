"""
IMUFilterNode for ROS 2
- Subscribes to raw IMU data from '/camera/camera/imu'
- Applies Madgwick orientation filter to compute attitude quaternion
- Republishes filtered IMU message on '/imu' with updated orientation
- Broadcasts dynamic TF transforms at 5 Hz:
    • base_link → camera_link (fixed translation/rotation)
    • camera_link → imu_frame (fixed translation/rotation)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from imu_publisher.madgwick_py.madgwickahrs import MadgwickAHRS
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class IMUFilterNode(Node):
    def __init__(self):
        super().__init__("imu_filter_node")

        # 定义 QoS 配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
        )

        # QoS configuration for publishers (RELIABLE)
        qos_reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE
        )

        # 订阅原始IMU数据
        self.subscription = self.create_subscription(
            Imu, "/camera/camera/imu", self.imu_callback, qos_profile
        )

        # 发布带有姿态信息的IMU数据
        self.publisher = self.create_publisher(Imu, "/imu", qos_reliable)

        # 初始化Madgwick滤波器
        self.filter = MadgwickAHRS(sampleperiod=0.01)

        # 初始化动态 TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)

        # 定时器以固定频率发布 TF
        self.create_timer(0.2, self.publish_dynamic_tf)  # 每 0.2 秒（5 Hz）

        self.get_logger().info(f"IMU subscriber QoS: {qos_profile}")
        self.get_logger().info(f"IMU publisher QoS: {qos_profile}")

    def imu_callback(self, msg):
        # 从IMU消息中提取角速度和加速度
        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z

        # 检查加速度是否有效，防止异常数据导致滤波器错误
        if ax == 0.0 and ay == 0.0 and az == 0.0:
            self.get_logger().warn("Received zero acceleration. Skipping this update.")
            return

        # 更新Madgwick滤波器
        try:
            self.filter.update_imu([gx, gy, gz], [ax, ay, az])

            # 更新消息中的四元数
            msg.orientation.x = self.filter.quaternion[1]
            msg.orientation.y = self.filter.quaternion[2]
            msg.orientation.z = self.filter.quaternion[3]
            msg.orientation.w = self.filter.quaternion[0]

            # 修改 frame_id 为 imu_frame
            msg.header.frame_id = "imu_frame"

            # 发布处理后的IMU消息
            self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Failed to update Madgwick filter: {e}")

    def publish_dynamic_tf(self):
        # 创建 TransformStamped 消息: base_link -> camera_link
        transform_base_to_camera = TransformStamped()
        transform_base_to_camera.header.stamp = self.get_clock().now().to_msg()
        transform_base_to_camera.header.frame_id = "base_link"
        transform_base_to_camera.child_frame_id = "camera_link"

        # 设置 base_link 到 camera_link 的平移和旋转
        transform_base_to_camera.transform.translation.x = 0.1
        transform_base_to_camera.transform.translation.y = 0.0
        transform_base_to_camera.transform.translation.z = 0.0
        transform_base_to_camera.transform.rotation.x = 0.0
        transform_base_to_camera.transform.rotation.y = 0.0
        transform_base_to_camera.transform.rotation.z = 0.0
        transform_base_to_camera.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform_base_to_camera)

        # 创建 TransformStamped 消息: camera_link -> imu_frame
        transform_camera_to_imu = TransformStamped()
        transform_camera_to_imu.header.stamp = self.get_clock().now().to_msg()
        transform_camera_to_imu.header.frame_id = "camera_link"
        transform_camera_to_imu.child_frame_id = "imu_frame"

        # 设置 camera_link 到 imu_frame 的平移和旋转
        transform_camera_to_imu.transform.translation.x = 0.0
        transform_camera_to_imu.transform.translation.y = 0.0
        transform_camera_to_imu.transform.translation.z = 0.0
        transform_camera_to_imu.transform.rotation.x = -0.5
        transform_camera_to_imu.transform.rotation.y = 0.5
        transform_camera_to_imu.transform.rotation.z = -0.5
        transform_camera_to_imu.transform.rotation.w = 0.5

        self.tf_broadcaster.sendTransform(transform_camera_to_imu)


def main(args=None):
    rclpy.init(args=args)
    node = IMUFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
