"""
ImageCompressorNode for ROS 2 Camera Streams
Subscribes to raw color and aligned depth Image topics,
compresses them (JPEG for color, PNG for depth),
and republishes as CompressedImage messages.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np


class ImageCompressorNode(Node):
    def __init__(self):
        super().__init__("image_compressor_node")

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.image_callback, 10
        )
        # self.depth_image_sub = self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_image_callback, 10)
        self.depth_image_sub = self.create_subscription(
            Image, "/camera/camera/aligned_depth_to_color/image_raw", self.depth_image_callback, 10
        )

        # Publishers
        self.compressed_image_pub = self.create_publisher(
            CompressedImage, "/camera/compressed_image", 10
        )
        self.compressed_depth_image_pub = self.create_publisher(
            CompressedImage, "/camera/compressed_depth_image", 10
        )

        # Log node startup message
        self.get_logger().info("Image Compressor Node has started!")

    def image_callback(self, msg):
        try:
            self.get_logger().info("Received a color image. Compressing...")

            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Compress image
            compressed_image = self.compress_image(cv_image, ".jpg")

            if compressed_image is not None:
                # Create CompressedImage message
                compressed_msg = CompressedImage()
                compressed_msg.header = msg.header
                compressed_msg.format = "jpeg"
                compressed_msg.data = compressed_image.tobytes()

                # Publish compressed image
                self.compressed_image_pub.publish(compressed_msg)
                self.get_logger().info("Color image compressed and published.")
            else:
                self.get_logger().warn("Failed to compress color image")
        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {str(e)}")

    def depth_image_callback(self, msg):
        try:
            self.get_logger().info("Received a depth image. Compressing...")

            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # Compress depth image
            compressed_image = self.compress_image(cv_image, ".png")

            if compressed_image is not None:
                # Create CompressedImage message
                compressed_msg = CompressedImage()
                compressed_msg.header = msg.header
                compressed_msg.format = "png"  # Use PNG for lossless compression
                compressed_msg.data = compressed_image.tobytes()

                # Publish compressed depth image
                self.compressed_depth_image_pub.publish(compressed_msg)
                self.get_logger().info("Depth image compressed and published.")
            else:
                self.get_logger().warn("Failed to compress depth image")
        except Exception as e:
            self.get_logger().error(f"Error in depth_image_callback: {str(e)}")

    def compress_image(self, cv_image, format):
        try:
            # Ensure the image is in the correct format for compression
            if format == ".png" and cv_image.dtype != np.uint16:
                cv_image = cv_image.astype(np.uint16)

            # Compress image
            success, compressed_image = cv2.imencode(format, cv_image)
            if not success:
                self.get_logger().error(f"Failed to compress image with format {format}")
                return None
            return compressed_image
        except Exception as e:
            self.get_logger().error(f"Error in compress_image: {str(e)}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = ImageCompressorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
