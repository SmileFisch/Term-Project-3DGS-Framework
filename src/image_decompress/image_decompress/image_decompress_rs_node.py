"""
ImageDecompressorNode for ROS 2 RGB-D Streams
Subscribes to compressed color and depth image topics and camera info,
decompresses images (JPEG/PNG for color, custom header and zlib for depth),
and republishes raw Image messages with dynamic resolution.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import struct
import zlib


class ImageDecompressorNode(Node):
    def __init__(self):
        super().__init__("image_decompressor_node")

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Declare and get parameters
        self.declare_parameter("compressed_color_topic", "/camera/compressed_image")
        self.declare_parameter("compressed_depth_topic", "/camera/compressed_depth_image")
        self.declare_parameter("color_image_topic", "/camera/decompressed_color_image")
        self.declare_parameter("depth_image_topic", "/camera/decompressed_depth_image")
        # Camera info topic for dynamic resolution
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("depth_image_height", 480)  # Default height
        self.declare_parameter("depth_image_width", 640)  # Default width

        self.compressed_color_topic = self.get_parameter("compressed_color_topic").value
        self.compressed_depth_topic = self.get_parameter("compressed_depth_topic").value
        self.color_image_topic = self.get_parameter("color_image_topic").value
        self.depth_image_topic = self.get_parameter("depth_image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.depth_image_height = self.get_parameter("depth_image_height").value
        self.depth_image_width = self.get_parameter("depth_image_width").value

        # QoS configuration for subscribers (BEST_EFFORT)
        qos_best_effort = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # QoS configuration for publishers (RELIABLE)
        qos_reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE
        )

        # Subscribers
        self.compressed_image_sub = self.create_subscription(
            CompressedImage,
            self.compressed_color_topic,
            self.compressed_color_callback,
            qos_reliable,
        )
        self.compressed_depth_image_sub = self.create_subscription(
            CompressedImage,
            self.compressed_depth_topic,
            self.compressed_depth_callback,
            qos_reliable,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, qos_reliable
        )

        # Publishers
        self.image_pub = self.create_publisher(Image, self.color_image_topic, qos_reliable)
        self.depth_image_pub = self.create_publisher(Image, self.depth_image_topic, qos_reliable)

        # Log node startup
        self.get_logger().info("Image Decompressor Node has started!")

    def camera_info_callback(self, msg):
        # Update resolution dynamically based on CameraInfo
        self.depth_image_height = msg.height
        self.depth_image_width = msg.width
        self.get_logger().info(
            f"Updated depth image resolution to {self.depth_image_height}x{self.depth_image_width}"
        )

    def compressed_color_callback(self, msg):
        self.get_logger().info(
            f"Received compressed color image at {msg.header.stamp}. Format: {msg.format}"
        )
        self.decompress_image(msg, "bgr8", self.image_pub)

    def compressed_depth_callback(self, msg):
        self.get_logger().info(
            f"Received compressed depth image at {msg.header.stamp}. Format: {msg.format}"
        )
        depth_image = self.decompress_depth_image(msg)
        if depth_image is not None:
            # Convert OpenCV image to ROS Image message
            ros_image_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="passthrough")
            ros_image_msg.header = msg.header
            self.depth_image_pub.publish(ros_image_msg)
            self.get_logger().info(
                f"Decompressed depth image published to {self.depth_image_pub.topic}"
            )

    def decompress_image(self, msg, encoding, publisher):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            ros_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding=encoding)
            ros_image_msg.header = msg.header
            publisher.publish(ros_image_msg)
            self.get_logger().info(f"Decompressed image published to {publisher.topic}")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert OpenCV image to ROS image: {e}")
        except Exception as e:
            self.get_logger().error(f"Error decompressing image: {e}")

    def decompress_depth_image(self, msg):
        try:
            depth_header_size = 12  # Fixed header size
            raw_header = msg.data[:depth_header_size]
            (raw_header_version, depth_size, compression) = struct.unpack("<III", raw_header)

            if compression == 0:  # Uncompressed
                depth_data = np.frombuffer(msg.data[depth_header_size:], dtype=np.uint16)
            elif compression == 1:  # zlib compressed
                decompressed_data = zlib.decompress(msg.data[depth_header_size:])
                depth_data = np.frombuffer(decompressed_data, dtype=np.uint16)
            else:
                raise ValueError(f"Unsupported compression format: {compression}")

            # Validate data length
            height = self.depth_image_height
            width = self.depth_image_width
            expected_size = height * width
            if len(depth_data) != expected_size:
                self.get_logger().error(
                    f"Decompressed data size {len(depth_data)} does not match expected size {expected_size}"
                )
                return None

            # Reshape
            depth_image = depth_data.reshape((height, width))
            return depth_image
        except Exception as e:
            self.get_logger().error(f"Failed to decompress depth image: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = ImageDecompressorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
