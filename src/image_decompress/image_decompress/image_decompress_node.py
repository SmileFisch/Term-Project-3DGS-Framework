"""
ImageDecompressorNode for ROS 2 RGB-D Dataset Extraction
- Subscribes to compressed color and depth image topics
- Synchronizes color and depth frames within 10 ms threshold
- Decompresses images, saves to disk with timestamps, and republishes as raw Image
- Queries and publishes camera extrinsics (odom → camera_color_frame) per frame
- Logs image filenames and ground truth transforms to text files
"""

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge, CvBridgeError
from collections import deque
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
import time


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
        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("child_frame", "camera_color_frame")

        self.compressed_color_topic = self.get_parameter("compressed_color_topic").value
        self.compressed_depth_topic = self.get_parameter("compressed_depth_topic").value
        self.color_image_topic = self.get_parameter("color_image_topic").value
        self.depth_image_topic = self.get_parameter("depth_image_topic").value
        self.source_file = self.get_parameter("source_file").value
        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value

        # Use source_file as the output directory name
        self.output_dir = self.source_file
        os.makedirs(self.output_dir, exist_ok=True)

        # Directories to save images
        self.color_image_dir = os.path.join(self.output_dir, "rgb")
        self.depth_image_dir = os.path.join(self.output_dir, "depth")
        os.makedirs(self.color_image_dir, exist_ok=True)
        os.makedirs(self.depth_image_dir, exist_ok=True)

        # Output text files for timestamps and filenames
        self.color_file_path = os.path.join(self.output_dir, "rgb.txt")
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

        # Initialize caches
        self.color_image_cache = deque(maxlen=200)
        self.depth_image_cache = deque(maxlen=200)

        # # Maximum allowable time difference for synchronization (in seconds)
        # self.cache_timeout = 0.1

        # TF buffer and listener
        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for extrinsics
        self.extrinsics_pub = self.create_publisher(TransformStamped, "/camera_extrinsics", 10)

        # QoS configuration
        qos_reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT
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

        # Logging
        self.get_logger().info("Image Decompressor Node has started!")

        self.timer = self.create_timer(0.5, self.check_and_publish_images)

    def compressed_color_callback(self, msg):
        self.get_logger().info(f"Received compressed color image at {msg.header.stamp}.")
        self.color_image_cache.append((msg, self.get_current_time()))
        self.check_and_publish_images()

    def compressed_depth_callback(self, msg):
        self.get_logger().info(f"Received compressed depth image at {msg.header.stamp}.")
        self.depth_image_cache.append((msg, self.get_current_time()))
        self.check_and_publish_images()

    def check_and_publish_images(self):
        max_wait_time = 2.0  # Maximum waiting time in seconds for unmatched images
        current_time = (
            self.get_current_time().sec + self.get_current_time().nanosec * 1e-9
        )  # Convert current ROS time to seconds

        while self.color_image_cache and self.depth_image_cache:
            color_msg, color_arrival_time = self.color_image_cache[0]
            depth_msg, depth_arrival_time = self.depth_image_cache[0]

            color_stamp = color_msg.header.stamp.sec + color_msg.header.stamp.nanosec * 1e-9
            depth_stamp = depth_msg.header.stamp.sec + depth_msg.header.stamp.nanosec * 1e-9

            # Convert arrival times to seconds
            color_arrival_time_sec = color_arrival_time.sec + color_arrival_time.nanosec * 1e-9
            depth_arrival_time_sec = depth_arrival_time.sec + depth_arrival_time.nanosec * 1e-9

            # Check if timestamps match within the threshold
            if abs(color_stamp - depth_stamp) <= 0.02:
                # If timestamps match, publish and remove from cache
                self.color_image_cache.popleft()
                self.depth_image_cache.popleft()
                self.publish_aligned_images(color_msg, depth_msg)
            elif color_stamp < depth_stamp:
                # Check if the color image has been waiting too long
                if current_time - color_arrival_time_sec > max_wait_time:
                    self.get_logger().warn(
                        f"Dropping outdated color image with timestamp {color_stamp:.6f} due to timeout."
                    )
                    self.color_image_cache.popleft()
                else:
                    # Wait for more depth images
                    break
            else:
                # Check if the depth image has been waiting too long
                if current_time - depth_arrival_time_sec > max_wait_time:
                    self.get_logger().warn(
                        f"Dropping outdated depth image with timestamp {depth_stamp:.6f} due to timeout."
                    )
                    self.depth_image_cache.popleft()
                else:
                    # Wait for more color images
                    break

    # def publish_aligned_images(self, color_msg, depth_msg):
    #     self.get_logger().info("Publishing aligned images.")
    #     try:
    #         # Decompress and publish color image
    #         self.decompress_and_save_image(color_msg, 'bgr8', self.color_image_dir, self.color_file_list, self.image_pub)

    #         # Decompress and publish depth image
    #         self.decompress_and_save_image(depth_msg, 'passthrough', self.depth_image_dir, self.depth_file_list, self.depth_image_pub)

    #         # Publish camera extrinsics
    #         self.publish_camera_extrinsics(color_msg.header.stamp)
    #     except Exception as e:
    #         self.get_logger().error(f"Error during image publishing: {e}")
    def publish_aligned_images(self, color_msg, depth_msg):
        self.get_logger().info("Publishing aligned images.")
        try:
            # First, ensure that camera extrinsics can be published
            if not self.publish_camera_extrinsics(color_msg.header.stamp):
                self.get_logger().error(
                    "Failed to publish camera extrinsics. Skipping image publishing."
                )
                return

            # Decompress and publish color image
            self.decompress_and_save_image(
                color_msg, "bgr8", self.color_image_dir, self.color_file_list, self.image_pub
            )

            # Decompress and publish depth image
            self.decompress_and_save_image(
                depth_msg,
                "passthrough",
                self.depth_image_dir,
                self.depth_file_list,
                self.depth_image_pub,
            )
            # self.color_image_cache.clear()
            # self.depth_image_cache.clear()

        except Exception as e:
            self.get_logger().error(f"Error during image publishing: {e}")

    # def is_image_empty_or_black(self, msg, image_type):
    #     """检查图像是否为空或全黑"""
    #     if not msg.data:
    # return True, f"{image_type} image is empty (msg.data is missing)."

    #     # Decode the compressed image
    #     np_arr = np.frombuffer(msg.data, np.uint8)
    #     cv_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)

    #     if cv_image is None:
    # return True, f"{image_type} image decoding failed (cv2.imdecode returned
    # None)."

    #     # Check if the image is completely black (all zeros)
    #     if np.all(cv_image == 0):
    # return True, f"{image_type} image is entirely black (all pixel values
    # are 0)."

    #     return False, ""

    # def publish_aligned_images(self, color_msg, depth_msg):
    #     self.get_logger().info("Publishing aligned images.")

    #     try:
    #         # 检查彩色图像是否为空或全黑
    #         is_empty_color, color_warn_msg = self.is_image_empty_or_black(color_msg, "Color")
    #         is_empty_depth, depth_warn_msg = self.is_image_empty_or_black(depth_msg, "Depth")

    #         if is_empty_color or is_empty_depth:
    #             # 如果任意一个图像为空或全黑，跳过处理，并输出警告
    #             if is_empty_color:
    #                 self.get_logger().warn(color_warn_msg)
    #             if is_empty_depth:
    #                 self.get_logger().warn(depth_warn_msg)
    #             self.get_logger().warn("Skipping image processing due to empty or black image.")
    #             return  # 直接返回，不执行后续处理

    #         # Decompress and publish color image
    #         self.decompress_and_save_image(color_msg, 'bgr8', self.color_image_dir, self.color_file_list, self.image_pub)

    #         # Decompress and publish depth image
    #         self.decompress_and_save_image(depth_msg, 'passthrough', self.depth_image_dir, self.depth_file_list, self.depth_image_pub)

    #         # Publish camera extrinsics
    #         self.publish_camera_extrinsics(color_msg.header.stamp)

    #     except Exception as e:
    #         self.get_logger().error(f"Error during image publishing: {e}")

    def decompress_and_save_image(self, msg, encoding, save_dir, file_list, publisher):
        try:
            # Detect and adjust save_dir if necessary
            if save_dir.endswith("rgb"):
                file_dir = "rgb"
            elif save_dir.endswith("depth"):
                file_dir = "depth"

            # Decode the compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)

            # Ensure depth images have the correct format
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
            file_list.write(f"{timestamp:.6f} {file_dir}/{filename}\n")
            file_list.flush()
            self.get_logger().info(f"Image saved: {filepath}")

            # Publish decompressed image
            publisher.publish(ros_image_msg)
            self.get_logger().info(f"Decompressed image published to {publisher.topic}")

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error decompressing image: {e}")

    # def decompress_and_save_image(self, msg, encoding, save_dir, file_list, publisher):
    #     try:
    #         # Detect and adjust save_dir if necessary
    #         if save_dir.endswith('rgb'):
    #             file_dir = 'rgb'
    #         elif save_dir.endswith('depth'):
    #             file_dir = 'depth'

    #         # Decode the compressed image
    #         np_arr = np.frombuffer(msg.data, np.uint8)
    #         cv_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)

    #         # Normalize depth images
    #         if save_dir.endswith('depth'):
    #             cv_image = cv2.normalize(cv_image, None, 0, 65535, cv2.NORM_MINMAX).astype(np.uint16)

    #         # Convert OpenCV image to ROS Image message
    #         ros_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding=encoding)
    #         ros_image_msg.header = msg.header

    #         # Save image to file (overwriting with normalized image)
    #         timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    #         filename = f"{timestamp:.6f}.png"
    #         filepath = os.path.join(save_dir, filename)
    #         cv2.imwrite(filepath, cv_image)

    #         # Log and record the image file path
    #         file_list.write(f"{timestamp:.6f} {file_dir}/{filename}\n")
    #         file_list.flush()
    #         self.get_logger().info(f"Image saved: {filepath}")

    #         # Publish decompressed image
    #         publisher.publish(ros_image_msg)
    #         self.get_logger().info(f"Decompressed image published to {publisher.topic}")

    #     except CvBridgeError as e:
    #         self.get_logger().error(f"CvBridge error: {e}")
    #     except Exception as e:
    #         self.get_logger().error(f"Error decompressing image: {e}")

    # def publish_camera_extrinsics(self, timestamp):
    #     try:
    #         # Convert timestamp to ROS2 time
    #         ros_time = rclpy.time.Time.from_msg(timestamp)

    #         # Query the transform: odom -> camera_color_frame
    #         transform = self.tf_buffer.lookup_transform(
    #             'odom',  # Parent frame
    #             'camera_color_frame',  # Child frame
    #             ros_time,  # Use the image's timestamp for TF lookup
    #             timeout=Duration(seconds=0.1)
    #         )

    #         # Use TF's timestamp and convert to seconds using nanoseconds
    #         tf_time = transform.header.stamp  # TF's timestamp
    #         tf_ros_time = rclpy.time.Time.from_msg(tf_time)
    #         tf_seconds = tf_ros_time.nanoseconds * 1e-9

    #         # Write transform to file
    #         tx = transform.transform.translation.x
    #         ty = transform.transform.translation.y
    #         tz = transform.transform.translation.z
    #         qx = transform.transform.rotation.x
    #         qy = transform.transform.rotation.y
    #         qz = transform.transform.rotation.z
    #         qw = transform.transform.rotation.w

    #         self.tf_file_list.write(f"{tf_seconds:.6f} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")
    #         self.tf_file_list.flush()

    #         # Publish extrinsics as a TransformStamped message
    #         extrinsics_msg = TransformStamped()
    #         extrinsics_msg.header.stamp = tf_time
    #         extrinsics_msg.header.frame_id = 'odom'
    #         extrinsics_msg.child_frame_id = 'camera_color_frame'
    #         extrinsics_msg.transform = transform.transform
    #         self.extrinsics_pub.publish(extrinsics_msg)

    #         self.get_logger().info(f"Published camera extrinsics at TF's timestamp {tf_seconds:.6f}")

    #     except Exception as e:
    #         self.get_logger().warn(f"Failed to lookup transform for timestamp {timestamp}: {e}")
    def publish_camera_extrinsics(self, timestamp):
        try:
            # Convert timestamp to ROS2 time
            ros_time = rclpy.time.Time.from_msg(timestamp)

            # Query the transform: odom -> camera_color_frame
            transform = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                ros_time,  # Use the image's timestamp for TF lookup
                timeout=Duration(seconds=0.1),
            )

            # Use TF's timestamp and convert to seconds using nanoseconds
            tf_time = transform.header.stamp  # TF's timestamp
            tf_ros_time = rclpy.time.Time.from_msg(tf_time)
            tf_seconds = tf_ros_time.nanoseconds * 1e-9
            
            # tf_ros_time = rclpy.time.Time.from_msg(tf_time)
            # tf_seconds = tf_ros_time.seconds + tf_ros_time.nanoseconds * 1e-9


            # Write transform to file
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            self.tf_file_list.write(
                f"{tf_seconds:.6f} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n"
            )
            self.tf_file_list.flush()

            # Publish extrinsics as a TransformStamped message
            extrinsics_msg = TransformStamped()
            extrinsics_msg.header.stamp = tf_time
            extrinsics_msg.header.frame_id = self.parent_frame
            extrinsics_msg.child_frame_id = self.child_frame
            extrinsics_msg.transform = transform.transform
            self.extrinsics_pub.publish(extrinsics_msg)

            self.get_logger().info(
                f"Published camera extrinsics at TF's timestamp {tf_seconds:.6f}"
            )
            return True  # Return True upon successful publication

        except Exception as e:
            self.get_logger().warn(f"Failed to lookup transform for timestamp {timestamp}: {e}")
            return False  # Return False if an exception occurs

    def get_current_time(self):
        return self.get_clock().now().to_msg()

    def destroy_node(self):
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
