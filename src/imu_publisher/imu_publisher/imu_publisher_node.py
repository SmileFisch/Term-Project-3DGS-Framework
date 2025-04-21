"""
IMUPublisher Node for ROS 2 Using Intel RealSense IMU
- Initializes RealSense pipeline to stream accelerometer and gyroscope data
- Calibrates gyroscope bias on startup
- Applies gravity compensation to accelerometer and bias correction + moving-average filter to gyroscope
- Publishes filtered IMU messages (linear_acceleration, angular_velocity) at 10 Hz on 'imu'
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import pyrealsense2 as rs
import numpy as np


class IMUPublisher(Node):
    def __init__(self):
        super().__init__("imu_publisher")

        self.publisher_ = self.create_publisher(Imu, "imu", 10)

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
        config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 400)

        try:
            self.pipeline.start(config)
            self.get_logger().info("RealSense Pipeline started successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to start RealSense pipeline: {e}")
            raise e

        self.gravity_vector = np.array([0.0, 0.0, 9.81])

        # 初始化零偏校正和滤波窗口
        self.gyro_bias = self.calibrate_gyro_bias()
        self.gyro_window = np.zeros((10, 3))  # 滑动窗口

        self.timer = self.create_timer(0.1, self.timer_callback)

    def calibrate_gyro_bias(self):
        # 校准陀螺仪零偏
        self.get_logger().info("Calibrating gyro bias...")
        gyro_samples = []
        for _ in range(100):
            frames = self.pipeline.wait_for_frames()
            gyro_frame = frames.first_or_default(rs.stream.gyro)
            gyro_data = gyro_frame.as_motion_frame().get_motion_data()
            gyro_samples.append([gyro_data.x, gyro_data.y, gyro_data.z])
        gyro_bias = np.mean(gyro_samples, axis=0)
        self.get_logger().info(f"Calibrated gyro bias: {gyro_bias}")
        return gyro_bias

    def timer_callback(self):
        try:
            frames = self.pipeline.wait_for_frames()
            accel_frame = frames.first_or_default(rs.stream.accel)
            gyro_frame = frames.first_or_default(rs.stream.gyro)

            accel_data = accel_frame.as_motion_frame().get_motion_data()
            gyro_data = gyro_frame.as_motion_frame().get_motion_data()

            raw_accel = np.array([accel_data.x, accel_data.y, accel_data.z])
            raw_accel_ros = np.array([raw_accel[0], raw_accel[2], -raw_accel[1]])
            net_accel = raw_accel_ros - self.gravity_vector

            # 零偏校正
            gyro_corrected = np.array([gyro_data.x, gyro_data.y, gyro_data.z]) - self.gyro_bias

            # 滤波处理
            self.gyro_window = np.roll(self.gyro_window, -1, axis=0)
            self.gyro_window[-1] = gyro_corrected
            gyro_filtered = np.mean(self.gyro_window, axis=0)

            # 坐标系转换
            gyro_filtered_ros = np.array([gyro_filtered[0], gyro_filtered[2], -gyro_filtered[1]])

            imu_msg = Imu()
            imu_msg.linear_acceleration.x = net_accel[0]
            imu_msg.linear_acceleration.y = net_accel[1]
            imu_msg.linear_acceleration.z = net_accel[2]
            imu_msg.angular_velocity.x = gyro_filtered_ros[0]
            imu_msg.angular_velocity.y = gyro_filtered_ros[1]
            imu_msg.angular_velocity.z = gyro_filtered_ros[2]
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_frame"

            self.publisher_.publish(imu_msg)

        except Exception as e:
            self.get_logger().error(f"Error reading IMU data: {e}")

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except Exception as e:
            self.get_logger().error(f"Failed to stop RealSense pipeline: {e}")
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()
    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        imu_publisher.get_logger().info("IMU Publisher shutting down.")
    finally:
        imu_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
