"""
SensorSyncNode for ROS 2
- Subscribes to IMU, Odometry, and LaserScan topics with a custom QoS profile
- Uses message_filters.ApproximateTimeSynchronizer to align messages within 0.1 s
- Publishes synchronized LaserScan on '/synced_scan' and Odometry on '/synced_odom'
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import message_filters
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry


class MySyncedNode(Node):
    def __init__(self):
        super().__init__("sensor_sync_node")

        # Define a QoS profile for best effort reliability
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=50, reliability=ReliabilityPolicy.RELIABLE
        )

        # Create publishers for synced data
        self.synced_scan_pub = self.create_publisher(LaserScan, "/synced_scan", qos_profile)
        self.synced_odom_pub = self.create_publisher(Odometry, "/synced_odom", qos_profile)

        # Standard ROS 2 subscriptions with custom QoS
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_callback, qos_profile)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, qos_profile)
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile
        )

        # Message filters with default QoS
        imu_filter = message_filters.SimpleFilter()
        odom_filter = message_filters.SimpleFilter()
        scan_filter = message_filters.SimpleFilter()

        # Link subscriptions to filters
        self.imu_sub_filter = imu_filter
        self.odom_sub_filter = odom_filter
        self.scan_sub_filter = scan_filter

        # ApproximateTimeSynchronizer
        ats = message_filters.ApproximateTimeSynchronizer(
            [imu_filter, odom_filter, scan_filter],
            queue_size=50,
            slop=0.1,  # Allow up to 20ms of time difference
        )
        ats.registerCallback(self.callback)

        self.get_logger().info("Node initialized with custom QoS and synced publishers")

    def imu_callback(self, msg):
        """Handle incoming IMU data."""
        self.imu_sub_filter.signalMessage(msg)

    def odom_callback(self, msg):
        """Handle incoming Odometry data."""
        self.odom_sub_filter.signalMessage(msg)

    def scan_callback(self, msg):
        """Handle incoming LaserScan data."""
        self.scan_sub_filter.signalMessage(msg)

    def callback(self, imu_msg, odom_msg, scan_msg):
        """
        Callback for synchronized messages.
        Publishes the synchronized LaserScan and Odometry data directly.
        """
        self.get_logger().info(
            f"Synchronized messages received:\n"
            f"IMU: {imu_msg.header.stamp.sec}.{imu_msg.header.stamp.nanosec}, "
            f"Odom: {odom_msg.header.stamp.sec}.{odom_msg.header.stamp.nanosec}, "
            f"Scan: {scan_msg.header.stamp.sec}.{scan_msg.header.stamp.nanosec}"
        )

        # Publish the synchronized LaserScan message
        self.synced_scan_pub.publish(scan_msg)

        # Publish the synchronized Odometry message
        self.synced_odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MySyncedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
