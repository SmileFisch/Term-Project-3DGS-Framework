"""
ImageDecompressorNode for ROS 2 RGB-D Dataset Extraction
- Subscribes to compressed color and depth image topics
- Decompresses and saves each frame as PNG files in per‐bag directories
- Records timestamps and filenames in text files (color.txt, depth.txt)
- Queries TF for odom → camera_color_frame at each image timestamp
  and logs/publishes extrinsics as TransformStamped messages
"""

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration


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
        # Default value for source file
        self.declare_parameter("source_file", "rgbd_dataset_Test.bag")

        self.compressed_color_topic = self.get_parameter("compressed_color_topic").value
        self.compressed_depth_topic = self.get_parameter("compressed_depth_topic").value
        self.color_image_topic = self.get_parameter("color_image_topic").value
        self.depth_image_topic = self.get_parameter("depth_image_topic").value
        self.source_file = self.get_parameter("source_file").value

        # Use source_file as the output directory name
        self.output_dir = self.source_file
        os.makedirs(self.output_dir, exist_ok=True)

        # Directories to save images
        self.color_image_dir = os.path.join(self.output_dir, "rgb")
        self.depth_image_dir = os.path.join(self.output_dir, "depth")
        os.makedirs(self.color_image_dir, exist_ok=True)
        os.makedirs(self.depth_image_dir, exist_ok=True)

        # Output text files for timestamps and filenames
        self.color_file_path = os.path.join(self.output_dir, "color.txt")
        self.depth_file_path = os.path.join(self.output_dir, "depth.txt")
        self.tf_file_path = os.path.join(self.output_dir, "groundtruth.txt")

        self.color_file_list = open(self.color_file_path, "w")
        self.depth_file_list = open(self.depth_file_path, "w")
        self.tf_file_list = open(self.tf_file_path, "w")

        # Add header lines with file info from parameter, appending .bag
        self.color_file_list.write(
            f"# color images\n# file: '{self.source_file}.bag'\n# timestamp filename\n"
        )
        self.depth_file_list.write(
            f"# depth maps\n# file: '{self.source_file}.bag'\n# timestamp filename\n"
        )
        self.tf_file_list.write(
            f"# ground truth trajectory\n# file: '{self.source_file}.bag'\n# timestamp tx ty tz qx qy qz qw\n"
        )

        # Initialize TF buffer and listener
        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))  # Cache time of 20 seconds
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize extrinsics publisher
        self.extrinsics_pub = self.create_publisher(TransformStamped, "/camera_extrinsics", 10)

        # QoS configuration for publishers and subscribers
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

        # Publishers
        self.image_pub = self.create_publisher(Image, self.color_image_topic, qos_reliable)
        self.depth_image_pub = self.create_publisher(Image, self.depth_image_topic, qos_reliable)

        # Log node startup
        self.get_logger().info(
            f"Image Decompressor Node has started! Source file: {self.source_file}"
        )

    def compressed_color_callback(self, msg):
        self.get_logger().info(
            f"Received compressed color image at {msg.header.stamp}. Decompressing..."
        )
        self.decompress_and_save_image(
            msg, "bgr8", self.color_image_dir, self.color_file_list, self.image_pub
        )

    def compressed_depth_callback(self, msg):
        self.get_logger().info(
            f"Received compressed depth image at {msg.header.stamp}. Decompressing..."
        )
        self.decompress_and_save_image(
            msg, "passthrough", self.depth_image_dir, self.depth_file_list, self.depth_image_pub
        )

    def decompress_and_save_image(self, msg, encoding, save_dir, file_list, publisher):
        try:
            # Decode the compressed image data
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)

            # Ensure correct format for depth images
            if encoding == "passthrough" and cv_image.dtype != np.uint16:
                cv_image = cv_image.astype(np.uint16)

            # Convert OpenCV image to ROS Image message
            ros_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding=encoding)
            ros_image_msg.header = msg.header

            # Save image to file
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            filename = f"{timestamp:.6f}.png"
            filepath = os.path.join(save_dir, filename)
            cv2.imwrite(filepath, cv_image)

            # Log and record the image file path
            file_list.write(f"{timestamp:.6f} {save_dir}/{filename}\n")
            file_list.flush()
            self.get_logger().info(f"Image saved: {filepath}")

            # Publish decompressed image
            publisher.publish(ros_image_msg)
            self.get_logger().info(f"Decompressed image published to {publisher.topic}")

            # Save TF transform to file and publish extrinsics
            self.save_and_publish_tf_transform(timestamp, filename)

        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert OpenCV image to ROS image: {e}")
        except Exception as e:
            self.get_logger().error(f"Error decompressing image: {e}")

    def save_and_publish_tf_transform(self, timestamp, filename):
        try:
            # Convert timestamp to ROS2 time
            ros_time = rclpy.time.Time(seconds=timestamp)

            # Query the transform: odom -> camera_color_frame
            transform = self.tf_buffer.lookup_transform(
                "odom",  # Parent frame
                "camera_color_frame",  # Child frame
                ros_time,  # Use the image's timestamp for TF lookup
                timeout=Duration(seconds=0.2),
            )

            # Extract translation and rotation
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            # Write to tf file
            self.tf_file_list.write(
                f"{timestamp:.6f} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f} \n"
            )
            self.tf_file_list.flush()
            self.get_logger().info(
                f"TF saved for {filename}: tx={tx:.6f}, ty={ty:.6f}, tz={tz:.6f}, qx={qx:.6f}, qy={qy:.6f}, qz={qz:.6f}, qw={qw:.6f}"
            )

            # Publish extrinsics as a TransformStamped message
            extrinsics_msg = TransformStamped()
            extrinsics_msg.header.stamp = ros_time.to_msg()
            extrinsics_msg.header.frame_id = "odom"
            extrinsics_msg.child_frame_id = "camera_color_frame"
            extrinsics_msg.transform.translation.x = tx
            extrinsics_msg.transform.translation.y = ty
            extrinsics_msg.transform.translation.z = tz
            extrinsics_msg.transform.rotation.x = qx
            extrinsics_msg.transform.rotation.y = qy
            extrinsics_msg.transform.rotation.z = qz
            extrinsics_msg.transform.rotation.w = qw
            self.extrinsics_pub.publish(extrinsics_msg)

            self.get_logger().info(f"Extrinsics published for {filename}: {extrinsics_msg}")

        except Exception as e:
            self.get_logger().warn(f"Failed to lookup transform for timestamp {timestamp}: {e}")

    def destroy_node(self):
        # Close file handlers before shutting down
        self.color_file_list.close()
        self.depth_file_list.close()
        self.tf_file_list.close()
        super().destroy_node()


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
