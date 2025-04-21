"""
Motor and CarController for Mecanum Robot with ROS 2 and GPIOZero
- Motor: PWM + H-bridge direction, optional encoder subscription, PID velocity control
- CarController: differential, axial, and mecanum steering algorithms for four Motor instances
- main(): initializes motors, runs a 5 s open-loop test at 50% duty, logs speeds
"""

from gpiozero import PWMOutputDevice, DigitalOutputDevice
import time
from std_msgs.msg import Int32
from rclpy.node import Node
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import traceback


class Motor:
    """
    Encapsulation of a single motor, including PWM control, direction control, encoder, and PID control logic.
    """

    def __init__(
        self, pwm_pin, dir1_pin, dir2_pin, enc_a_pin=None, enc_b_pin=None, compensate_rate=1
    ):
        self.pwm = PWMOutputDevice(pwm_pin)
        self.dir1 = DigitalOutputDevice(dir1_pin)
        self.dir2 = DigitalOutputDevice(dir2_pin)
        self.compensate_rate = compensate_rate

        # PID parameters
        self.kp = 0.85  # Proportional coefficient
        self.ki = 0.01  # Integral coefficient
        self.kd = 0.00  # Derivative coefficient
        self.integral = 0
        self.previous_error = 0

        # Control-related parameters
        self.target_speed = 0  # Target speed (m/s)
        self.duty_cycle = 0  # Initial duty cycle
        self.last_update_time = time.time()  # Last PID update time

        # Encoder parameters
        self.enc_a_pin = enc_a_pin
        self.enc_b_pin = enc_b_pin
        self.pulse_count = 0
        self.last_speed = 0
        self.previous_pulse_count = 0
        self.last_time = time.time()
        self.last_time_for_update = time.time()
        self.previous_pulse_count_for_update = 0
        self.last_speed_for_update = 0

        if self.enc_a_pin is not None and self.enc_b_pin is not None:
            self._init_encoder()

    def _init_encoder(self):
        """
        Initialize the ROS 2 node and subscription for the encoder.
        """
        # rclpy.init()

        self.node = rclpy.create_node(f"encoder_node_{self.enc_a_pin}_{self.enc_b_pin}")
        self.topic_name = self._select_topic(self.enc_a_pin, self.enc_b_pin)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=20, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.subscription = self.node.create_subscription(
            Int32, self.topic_name, self._pulse_count_callback, qos
        )
        self.node.get_logger().info(f"encoder_node_{self.enc_a_pin}_{self.enc_b_pin}start!")

    def _select_topic(self, enc_a_pin, enc_b_pin):
        """
        Select the corresponding topic based on encoder pins.
        """
        encoder_configs = {
            (25, 17): "/encoder/pulse_count_1",
            (26, 21): "/encoder/pulse_count_2",
            (23, 24): "/encoder/pulse_count_3",
            (0, 11): "/encoder/pulse_count_4",
        }
        return encoder_configs.get((enc_a_pin, enc_b_pin))

    def _pulse_count_callback(self, msg):
        """
        Callback function for encoder pulse count.
        """
        # if self.pulse_count != msg.data:
        #     self.node.get_logger().info(f"Pulse count updated: {msg.data}")
        # else:
        #     self.node.get_logger().info(f"Pulse count unchanged: {msg.data}")
        self.pulse_count = msg.data

    def get_speed(self, wheel_diameter, ticks_per_revolution, gear_ratio):
        """
        Calculate the linear speed of the motor (m/s).
        :param wheel_diameter: Diameter of the wheel (meters)
        :param ticks_per_revolution: Pulses per revolution of the encoder
        :param gear_ratio: Gear reduction ratio
        """
        current_time = time.time()
        elapsed_time = current_time - self.last_time
        delta_pulse = self.pulse_count - self.previous_pulse_count
        self.previous_pulse_count = self.pulse_count
        # self.node.get_logger().info(f"Elapsed time: {elapsed_time:.6f} seconds, Delta pulse: {delta_pulse}")
        # self.node.get_logger().info("Call stack:\n" + "".join(traceback.format_stack()))

        if elapsed_time == 0:
            return self.last_speed

        revolutions = delta_pulse / (ticks_per_revolution)
        distance = revolutions * (wheel_diameter * 3.1416)
        speed = distance / elapsed_time

        self.last_speed = speed
        self.last_time = current_time
        return speed

    def calculate_speed_for_update(self, wheel_diameter, ticks_per_revolution, gear_ratio):
        """
        Speed calculation specifically for the update function to avoid affecting shared state in get_speed.
        :param wheel_diameter: Diameter of the wheel (meters)
        :param ticks_per_revolution: Pulses per revolution of the encoder
        :param gear_ratio: Gear reduction ratio
        """
        current_time = time.time()
        elapsed_time = current_time - self.last_time_for_update
        delta_pulse = self.pulse_count - self.previous_pulse_count_for_update

        self.previous_pulse_count_for_update = self.pulse_count
        self.last_time_for_update = current_time

        # self.node.get_logger().info(f"[Update Speed] Elapsed time: {elapsed_time:.6f} seconds, Delta pulse: {delta_pulse}")

        if elapsed_time == 0:
            return self.last_speed_for_update

        revolutions = delta_pulse / (ticks_per_revolution)
        distance = revolutions * (wheel_diameter * 3.1416)
        speed = distance / elapsed_time

        self.last_speed_for_update = speed
        return speed

    def speed_to_duty_cycle(self, target_speed, max_rpm, wheel_diameter, gear_ratio):
        """
        Convert linear speed (m/s) to duty cycle (-1 to 1).
        :param target_speed: Target linear speed (m/s)
        :param max_rpm: Maximum revolutions per minute of the motor
        :param wheel_diameter: Diameter of the wheel (meters)
        :param gear_ratio: Gear reduction ratio
        """
        max_omega = 2 * 3.1416 * max_rpm / 60
        max_speed = (max_omega / gear_ratio) * (wheel_diameter / 2)
        duty_cycle = max(-1.0, min(1.0, target_speed / max_speed))
        return duty_cycle

    def duty_cycle_to_speed(self, duty_cycle, max_rpm, wheel_diameter, gear_ratio):
        """
        Convert duty cycle (-1 to 1) to linear speed (m/s).
        :param duty_cycle: Duty cycle (-1 to 1)
        :param max_rpm: Maximum revolutions per minute of the motor
        :param wheel_diameter: Diameter of the wheel (meters)
        :param gear_ratio: Gear reduction ratio
        """
        max_omega = 2 * 3.1416 * max_rpm / 60  # Maximum angular velocity (rad/s)
        max_speed = (max_omega / gear_ratio) * (wheel_diameter / 2)  # Maximum linear speed (m/s)
        target_speed = duty_cycle * max_speed
        return target_speed

    def set_duty_cycle(self, target_speed, max_rpm, wheel_diameter, gear_ratio):
        """
        Set the duty cycle based on the target speed.
        """
        duty_cycle = self.speed_to_duty_cycle(target_speed, max_rpm, wheel_diameter, gear_ratio)
        self.duty_cycle = duty_cycle  # Store the duty cycle

    def set_pid_params(self, kp, ki, kd):
        """
        Dynamically set PID parameters.
        :param kp: Proportional gain
        :param ki: Integral gain
        :param kd: Derivative gain
        """
        self.kp, self.ki, self.kd = kp, ki, kd

    def set_speed(self, speed):
        """
        Set motor speed and direction.
        :param speed: Speed as a fraction of maximum speed (-1 to 1)
        """
        speed = max(-1.0, min(1.0, speed))  # Limit speed range
        if speed > 0:  # Forward
            self.dir1.on()
            self.dir2.off()
        elif speed < 0:  # Rollback
            self.dir1.off()
            self.dir2.on()
        else:  # Stop
            self.dir1.off()
            self.dir2.off()

        self.pwm.value = abs(speed)

    def stop(self):
        """
        Stop the motor.
        """
        self.set_speed(0)

    def update(self, target_duty_cycle, wheel_diameter, ticks_per_revolution, gear_ratio, max_rpm):
        """
        Update motor speed using PID control.
        :param target_duty_cycle: Target duty cycle (-1.0 to 1.0)
        :param wheel_diameter: Wheel diameter (meters)
        :param ticks_per_revolution: Pulses per revolution
        :param gear_ratio: Gear reduction ratio
        :param max_rpm: Maximum RPM of the motor
        """
        # Get current speed (m/s)
        current_speed = self.calculate_speed_for_update(
            wheel_diameter, ticks_per_revolution, gear_ratio
        )

        # Convert target duty cycle to target speed
        target_speed = self.duty_cycle_to_speed(
            target_duty_cycle, max_rpm, wheel_diameter, gear_ratio
        )

        # Calculate speed error
        error = target_speed - current_speed

        # Time difference
        current_time = time.time()
        delta_time = current_time - self.last_update_time
        if delta_time == 0:
            delta_time = 0.01

        # PID control calculation
        self.integral += error * delta_time
        self.integral = max(-10.0, min(10.0, self.integral))
        derivative = (error - self.previous_error) / delta_time
        output_speed = current_speed + (
            self.kp * error + self.ki * self.integral + self.kd * derivative
        )

        # Convert output speed to duty cycle
        output_duty_cycle = self.speed_to_duty_cycle(
            output_speed, max_rpm, wheel_diameter, gear_ratio
        )
        # Apply compensation rate and limit range
        output_duty_cycle = max(-1.0, min(1.0, output_duty_cycle * self.compensate_rate))

        # Set the final duty cycle
        # Test
        # output_duty_cycle = 0.5
        self.set_speed(output_duty_cycle)

        # Log Variables
        # self.node.get_logger().info(f"Target Speed: {target_speed:.3f} m/s, Current Speed: {current_speed:.3f} m/s, Error: {error:.3f}")
        # self.node.get_logger().info(f"PID Output: {output_speed:.3f} m/s, Output Duty Cycle: {output_duty_cycle:.3f}")
        # self.node.get_logger().info(f"Delta_time Output: {delta_time:.3f} m/s")

        # Log target speed and current speed to a file
        with open("motor_speed_log.txt", "a") as log_file:
            log_file.write(
                f"{time.time()}, Target Speed: {target_speed:.3f}, Current Speed: {current_speed:.3f}, Error: {error:.3f}\n"
            )

        # Update state
        self.previous_error = error
        self.last_update_time = current_time


class CarController:
    """
    Controller for differential, axial, and mecanum steering.
    """

    def __init__(self, motors, wheel_diameter, ticks_per_revolution, gear_ratio, max_rpm):
        """
        Initialize the car controller with motor parameters.
        :param motors: List of motor objects
        :param wheel_diameter: Wheel diameter (meters)
        :param ticks_per_revolution: Pulses per revolution
        :param gear_ratio: Gear reduction ratio
        :param max_rpm: Maximum RPM of the motor
        """
        self.motors = motors
        self.wheel_diameter = wheel_diameter
        self.ticks_per_revolution = ticks_per_revolution
        self.gear_ratio = gear_ratio
        self.max_rpm = max_rpm  # Maximum RPM for speed to duty cycle conversion

    def stop(self):
        for motor in self.motors:
            motor.stop()

    def differential_turn(self, x, y):
        """
        Differential steering.
        :param x: Longitudinal linear speed (m/s)
        :param y: Lateral linear speed (m/s)
        """
        # Calculate left and right wheel linear speeds (m/s)
        left_speed = x
        right_speed = x

        # Convert to duty cycle
        left_duty_cycle = self.motors[0].speed_to_duty_cycle(
            left_speed, self.max_rpm, self.wheel_diameter, self.gear_ratio
        )
        right_duty_cycle = self.motors[2].speed_to_duty_cycle(
            right_speed, self.max_rpm, self.wheel_diameter, self.gear_ratio
        )

        # Update motor speeds (closed-loop control)
        self.motors[0].update(
            left_duty_cycle,
            self.wheel_diameter,
            self.ticks_per_revolution,
            self.gear_ratio,
            self.max_rpm,
        )  # lf
        self.motors[1].update(
            left_duty_cycle,
            self.wheel_diameter,
            self.ticks_per_revolution,
            self.gear_ratio,
            self.max_rpm,
        )  # lb
        self.motors[2].update(
            right_duty_cycle,
            self.wheel_diameter,
            self.ticks_per_revolution,
            self.gear_ratio,
            self.max_rpm,
        )  # rf
        self.motors[3].update(
            right_duty_cycle,
            self.wheel_diameter,
            self.ticks_per_revolution,
            self.gear_ratio,
            self.max_rpm,
        )  # rb

    def axial_turn(self, x, y):
        """
        Axial steering.
        :param x: Longitudinal linear speed (m/s)
        :param y: Lateral linear speed (m/s)
        """
        # Set left and right wheel linear speeds (m/s)
        left_speed = -x
        right_speed = x

        # Convert to duty cycle
        left_duty_cycle = self.motors[0].speed_to_duty_cycle(
            left_speed, self.max_rpm, self.wheel_diameter, self.gear_ratio
        )
        right_duty_cycle = self.motors[2].speed_to_duty_cycle(
            right_speed, self.max_rpm, self.wheel_diameter, self.gear_ratio
        )

        # Update motor speeds (closed-loop control)
        self.motors[0].update(
            left_duty_cycle,
            self.wheel_diameter,
            self.ticks_per_revolution,
            self.gear_ratio,
            self.max_rpm,
        )  # lf
        self.motors[1].update(
            left_duty_cycle,
            self.wheel_diameter,
            self.ticks_per_revolution,
            self.gear_ratio,
            self.max_rpm,
        )  # lb
        self.motors[2].update(
            right_duty_cycle,
            self.wheel_diameter,
            self.ticks_per_revolution,
            self.gear_ratio,
            self.max_rpm,
        )  # rf
        self.motors[3].update(
            right_duty_cycle,
            self.wheel_diameter,
            self.ticks_per_revolution,
            self.gear_ratio,
            self.max_rpm,
        )  # rb

    def mecanum_drive(self, x, y, xr, wheel_base, wheel_track):
        """
        Mecanum steering.
        :param x: Longitudinal linear speed (m/s)
        :param y: Lateral linear speed (m/s)
        :param xr: Rotational speed (m/s)
        :param wheel_base: Distance between front and rear wheels (meters)
        :param wheel_track: Distance between left and right wheels (meters)
        """
        # Calculate rotational factor
        rotation_factor = (wheel_base + wheel_track) / 2

        # Calculate the linear speed for each wheel (m/s)
        front_left_speed = x - y - xr * rotation_factor
        front_right_speed = x + y - xr * rotation_factor
        rear_left_speed = x + y + xr * rotation_factor
        rear_right_speed = x - y + xr * rotation_factor

        # Convert to duty cycle
        front_left_duty_cycle = self.motors[0].speed_to_duty_cycle(
            front_left_speed, self.max_rpm, self.wheel_diameter, self.gear_ratio
        )
        front_right_duty_cycle = self.motors[1].speed_to_duty_cycle(
            front_right_speed, self.max_rpm, self.wheel_diameter, self.gear_ratio
        )
        rear_left_duty_cycle = self.motors[2].speed_to_duty_cycle(
            rear_left_speed, self.max_rpm, self.wheel_diameter, self.gear_ratio
        )
        rear_right_duty_cycle = self.motors[3].speed_to_duty_cycle(
            rear_right_speed, self.max_rpm, self.wheel_diameter, self.gear_ratio
        )

        # Update the speed of each motor (closed-loop control)
        self.motors[0].update(
            front_left_duty_cycle,
            self.wheel_diameter,
            self.ticks_per_revolution,
            self.gear_ratio,
            self.max_rpm,
        )  # lf
        self.motors[1].update(
            front_right_duty_cycle,
            self.wheel_diameter,
            self.ticks_per_revolution,
            self.gear_ratio,
            self.max_rpm,
        )  # lb
        self.motors[2].update(
            rear_left_duty_cycle,
            self.wheel_diameter,
            self.ticks_per_revolution,
            self.gear_ratio,
            self.max_rpm,
        )  # rf
        self.motors[3].update(
            rear_right_duty_cycle,
            self.wheel_diameter,
            self.ticks_per_revolution,
            self.gear_ratio,
            self.max_rpm,
        )  # rb


def main():
    """
    Main function to initialize the controller and test motor performance.
    """
    # Wheel parameters
    wheel_diameter = 0.055  # Unit: meters
    ticks_per_revolution = 4920  # Pulses per revolution
    gear_ratio = 20.0  # Gear reduction ratio
    max_rpm = 400  # Maximum RPM, based on motor specifications

    # Initialize four motors (with encoders)
    motors = [
        Motor(pwm_pin=12, dir1_pin=1, dir2_pin=27, enc_a_pin=25, enc_b_pin=17),
        # lf
        Motor(pwm_pin=19, dir1_pin=20, dir2_pin=16, enc_a_pin=26, enc_b_pin=21),
        # lb
        Motor(pwm_pin=18, dir1_pin=14, dir2_pin=15, enc_a_pin=23, enc_b_pin=24),
        # rf
        Motor(pwm_pin=13, dir1_pin=6, dir2_pin=5, enc_a_pin=0, enc_b_pin=11),
        # rb
    ]

    car_controller = CarController(
        motors, wheel_diameter, ticks_per_revolution, gear_ratio, max_rpm
    )

    target_duty_cycle = 0.5

    print("Testing all four motors with the same duty cycle...")

    for motor in motors:
        motor.set_speed(target_duty_cycle)

    duration = 5
    start_time = time.time()

    with open("motor_test_log.txt", "w") as log_file:
        log_file.write("Time, Motor 1 Speed, Motor 2 Speed, Motor 3 Speed, Motor 4 Speed\n")

        while time.time() - start_time < duration:
            # get speed of every motor
            motor_speeds = [
                motor.get_speed(wheel_diameter, ticks_per_revolution, gear_ratio)
                for motor in motors
            ]

            log_line = (
                f"{time.time():.3f}, " + ", ".join(f"{speed:.3f}" for speed in motor_speeds) + "\n"
            )
            log_file.write(log_line)

            print(
                f"Time: {time.time():.3f}, Speeds: {', '.join(f'{speed:.3f}' for speed in motor_speeds)}"
            )

            time.sleep(0.1)

        print("Test finish, stop all motors and save datas in motor_test_log.txt.")

    car_controller.stop()


if __name__ == "__main__":
    main()
