"""
ImageCompressorNode for ROS 2 with configurable topics and formats
- Subscribes to raw color and depth Image topics (configurable via ROS parameters)
- Compresses color images with JPEG and depth images with PNG (default, also configurable)
- Republishes as CompressedImage on configurable topics
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class ImageCompressorNode(Node):
    def __init__(self):
        super().__init__("image_compressor_node")

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Declare and get parameters
        self.declare_parameter("color_image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_image_topic", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("compressed_color_topic", "/camera/compressed_image")
        self.declare_parameter("compressed_depth_topic", "/camera/compressed_depth_image")
        self.declare_parameter("color_compression_format", ".jpg")  # Default: JPEG
        self.declare_parameter("depth_compression_format", ".png")  # Default: PNG

        self.color_image_topic = self.get_parameter("color_image_topic").value
        self.depth_image_topic = self.get_parameter("depth_image_topic").value
        self.compressed_color_topic = self.get_parameter("compressed_color_topic").value
        self.compressed_depth_topic = self.get_parameter("compressed_depth_topic").value
        self.color_compression_format = self.get_parameter("color_compression_format").value
        self.depth_compression_format = self.get_parameter("depth_compression_format").value

        # QoS configuration for subscribers (BEST_EFFORT)
        qos_best_effort = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # QoS configuration for publishers (RELIABLE)
        qos_reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, self.color_image_topic, self.color_image_callback, qos_best_effort
        )
        self.depth_image_sub = self.create_subscription(
            Image, self.depth_image_topic, self.depth_image_callback, qos_best_effort
        )

        # Publishers
        self.compressed_image_pub = self.create_publisher(
            CompressedImage, self.compressed_color_topic, qos_reliable
        )
        self.compressed_depth_image_pub = self.create_publisher(
            CompressedImage, self.compressed_depth_topic, qos_reliable
        )

        # Log node startup
        self.get_logger().info("Image Compressor Node has started!")

    def color_image_callback(self, msg):
        self.get_logger().info(f"Received color image at {msg.header.stamp}. Compressing...")
        self.process_image(msg, "bgr8", self.color_compression_format, self.compressed_image_pub)

    def depth_image_callback(self, msg):
        self.get_logger().info(f"Received depth image at {msg.header.stamp}. Compressing...")
        self.process_image(
            msg, "passthrough", self.depth_compression_format, self.compressed_depth_image_pub
        )

    def process_image(self, msg, encoding, format, publisher):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)

            # Ensure correct format for depth images
            if format == ".png" and cv_image.dtype != np.uint16:
                cv_image = cv_image.astype(np.uint16)

            # Compress image
            success, compressed_image = cv2.imencode(format, cv_image)
            if not success:
                self.get_logger().warn(f"Failed to compress image with format {format}")
                return

            # Create and publish CompressedImage message
            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = format.strip(".")
            compressed_msg.data = compressed_image.tobytes()
            publisher.publish(compressed_msg)
            self.get_logger().info(f"Image compressed and published to {publisher.topic}")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert ROS image to OpenCV image: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageCompressorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
