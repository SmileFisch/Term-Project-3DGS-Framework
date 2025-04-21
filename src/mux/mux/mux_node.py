"""
MuxNode for ROS 2 Control Signal Multiplexing
- Subscribes to joystick, keyboard, and algorithm control topics plus a mode selector
- Chooses which control signal to forward based on current mode ('j', 'k', or 'l')
- Publishes the chosen Float64MultiArray to '/controller/mux' for the drive system
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Twist
import time


class MuxNode(Node):
    """
    MuxNode handles switching between different control sources (keyboard, joystick, algorithms, etc.)
    and publishes the selected control signal to the car drive system.

    - Subscriptions:
        - `/controller/joy_stick`:
            - Published by: Joystick
            - Type: Float64MultiArray [x (angular velocity), y (linear velocity), t (time)]
            - Description: Control signals from the joystick.
        - `/controller/key_board`:
            - Published by: Keyboard
            - Type: Float64MultiArray [x (angular velocity), y (linear velocity), t (time)]
            - Description: Control signals from the keyboard.
        - `/cmd_vel`:
            - Published by: Navigation or Algorithm
            - Type: Twist
            - Description: Velocity command in the form of linear and angular velocities.
        - `/controller/mode`:
            - Published by: External mode selector
            - Type: String
            - Description: Switches between control modes ('j' for joystick, 'k' for keyboard, 'l' for algorithm).

    - Publications:
        - `/controller/mux`:
            - Subscribed by: Car drive system
            - Type: Float64MultiArray [x (angular velocity), y (linear velocity), t (time)]
            - Description: Publishes the selected control signal based on the current mode.
    """

    def __init__(self):
        super().__init__("mux_node")

        self.get_logger().info("MuxNode has been started.")

        # Subscriptions to various control sources and mode selector
        self.js_sub = self.create_subscription(
            Float64MultiArray, "/controller/joy_stick", self.js_control, 10
        )
        self.kb_sub = self.create_subscription(
            Float64MultiArray, "/controller/key_board", self.kb_control, 10
        )
        self.al_sub = self.create_subscription(Twist, "/cmd_vel", self.al_control, 10)
        self.mode_sub = self.create_subscription(String, "/controller/mode", self.callback, 10)

        # Publisher for the selected control signal
        self.drive_pub = self.create_publisher(Float64MultiArray, "/controller/mux", 10)

        # Current mode and control messages
        self.mode = "j"  # Default mode: joystick
        self.kb_msg = Float64MultiArray()
        self.js_msg = Float64MultiArray()
        self.al_msg = Float64MultiArray()

        # Timer to periodically publish the selected control message
        self.timer = self.create_timer(0.2, self.publish_mux)

        # Flags to indicate whether new messages have been received
        self.kb_updated = False
        self.js_updated = False
        self.al_updated = False

    def kb_control(self, msg):
        """
        Callback function for keyboard control messages.
        Updates the keyboard message and marks it as updated.
        """
        self.kb_msg.data = msg.data
        self.kb_updated = True

    def js_control(self, msg):
        """
        Callback function for joystick control messages.
        Updates the joystick message and marks it as updated.
        """
        self.js_msg.data = msg.data
        self.js_updated = True

    def al_control(self, msg):
        """
        Callback function for algorithm-generated control messages.
        Converts Twist messages into Float64MultiArray format with time information.
        """
        al_x = msg.linear.x
        al_y = msg.linear.y
        al_theta = msg.angular.z

        # Get the current time in seconds
        current_time = self.get_clock().now()
        time_in_sec = (
            current_time.seconds_nanoseconds()[0] + current_time.seconds_nanoseconds()[1] * 1e-9
        )

        # Construct the algorithm control message
        self.al_msg.data = [al_x, al_y, al_theta, time_in_sec]
        self.al_updated = True

    def callback(self, msg):
        """
        Callback function for mode switching.
        Changes the control mode based on the received message.
        """
        if msg.data == "j":  # Joystick mode
            self.mode = "j"
            self.get_logger().info("Control mode switched to: Joystick (mode: j)")

        elif msg.data == "k":  # Keyboard mode
            self.mode = "k"
            self.get_logger().info("Control mode switched to: Keyboard (mode: k)")

        elif msg.data == "l":  # Algorithm mode
            self.mode = "l"
            self.get_logger().info("Control mode switched to: Algorithm (mode: l)")

        else:
            self.get_logger().warn(f"Received unknown mode: {msg.data}")

    def publish_mux(self):
        """
        Timer callback function to publish the control message based on the current mode.
        Only publishes if the corresponding message has been updated.
        """
        if self.mode == "j" and self.js_updated:
            self.drive_pub.publish(self.js_msg)
            self.js_updated = False
        elif self.mode == "k" and self.kb_updated:
            self.drive_pub.publish(self.kb_msg)
            self.kb_updated = False
        elif self.mode == "l" and self.al_updated:
            self.drive_pub.publish(self.al_msg)
            self.al_updated = False


def main(args=None):
    rclpy.init(args=args)
    muxNode = MuxNode()
    try:
        rclpy.spin(muxNode)
    except KeyboardInterrupt:
        pass
    finally:
        muxNode.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
