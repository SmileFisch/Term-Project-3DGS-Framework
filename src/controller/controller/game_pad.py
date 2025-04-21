"""Xbox 360 controller support for Python
11/9/13 - Steven Jacobs

This class module supports reading a connected xbox controller.
It requires that xboxdrv be installed first:

    sudo apt-get install xboxdrv

See http://pingus.seul.org/~grumbel/xboxdrv/ for details on xboxdrv

Example usage:

    import xbox
    joy = xbox.Joystick()         #Initialize joystick

    if joy.A():                   #Test state of the A button (1=pressed, 0=not pressed)
        print 'A button pressed'
    x_axis   = joy.leftX()        #X-axis of the left stick (values -1.0 to 1.0)
    (x,y)    = joy.leftStick()    #Returns tuple containing left X and Y axes (values -1.0 to 1.0)
    trigger  = joy.rightTrigger() #Right trigger position (values 0 to 1.0)

    joy.close()                   #Cleanup before exit
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64, String
from controller.xbox import Joystick


class ControlPublisher(Node):
    """
    Node:
        - name: Game pad controller
        - function:
            - B button: stop car
            - A button: restart car
            - X button: manual control mode
            - Y button: autonomous driving mode
            - Left stick: driving signal
        - Subscription: None
        - Publisher:
            - /controller/joy_stick
                - type: Float64MultiArray
                - description: publish control value and time
            - /controller/mode
                - type: String
                - description: publish mode,
                    j for joy-con,
                    l for self navigation or any other algorithm that could produce cmd_vel in [x, y] form
    """

    def __init__(self) -> None:
        super().__init__("game_pad")
        self.get_logger().info("Game pad controller node has been started.")
        self.publisher = self.create_publisher(Float64MultiArray, "/controller/joy_stick", 10)
        self.mode_pub = self.create_publisher(String, "/controller/mode", 10)
        self.pub_time_period = 0.2
        self.timer = self.create_timer(self.pub_time_period, self.timer_callback)
        self.joy = Joystick()
        self.stop_flag = False
        self.spd_rate = 0.25
        self.mode = "j"

    def timer_callback(self) -> None:
        # publish controll topic with a fixed rate
        if self.joy.B():
            self.stop_flag = True
        if self.joy.A():
            self.stop_flag = False
        if self.joy.X():
            self.mode = "j"  # joy stick mode (Manual Control)
        if self.joy.Y():
            self.mode = "l"  # algorithm testing mode (Automatic Control)

        mode_msg = String()
        mode_msg.data = self.mode
        self.mode_pub.publish(mode_msg)

        msg = Float64MultiArray()

        x, y = self.joy.leftStick()
        t = time.time()

        if self.stop_flag:
            msg.data = [0.0, 0.0, t]
        else:
            msg.data = [self.spd_rate * x, self.spd_rate * y, t]

        self.publisher.publish(msg)

    """
    def publishment(self) -> None:

        # To publish topic as fast as possible (rate: 4000Hz)

        while rclpy.ok():

            if self.joy.B():
                self.stop_flag = True
            if self.joy.A():
                self.stop_flag = False
            msg = Float64MultiArray()
            x, y = self.joy.leftStick()
            t = time.time()
            if self.stop_flag:
                msg.data = [0.0, 0.0, t]
            else:
                msg.data = [self.spd_rate*x, self.spd_rate*y, t]
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0)
            # time.sleep(0.02)
    """


def main(args=None):
    rclpy.init(args=args)
    print("Publish Joystick Control Value")
    pub = ControlPublisher()

    rclpy.spin(pub)
    # pub.publishment()

    pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
