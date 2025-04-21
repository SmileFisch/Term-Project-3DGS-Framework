"""
Xbox Joystick Interface via xboxdrv
Launches the xboxdrv subprocess to read controller events.
Parses raw 140‑char event strings into:
  • Scaled analog inputs: leftX, leftY, rightX, rightY, triggers
  • Digital inputs: buttons (A, B, X, Y, bumpers, start, back, guide), D‑pad
  • Connection monitoring: connected(), automatic refresh at up to refreshRate Hz
Provides convenience methods leftStick() and rightStick() returning (x, y).
Includes a demo main() that prints all input states in real time.
"""

import subprocess
import time
import select
import os


class Joystick:
    """Initializes the joystick/wireless receiver, launching 'xboxdrv' as a subprocess
    and checking that the wired joystick or wireless receiver is attached.
    The refreshRate determines the maximnum rate at which events are polled from xboxdrv.
    Calling any of the Joystick methods will cause a refresh to occur, if refreshTime has elapsed.
    Routinely call a Joystick method, at least once per second, to avoid overfilling the event buffer.

    Usage:
        joy = xbox.Joystick()
    """

    def __init__(self, refreshRate=30):
        # # 卸载系统默认驱动
        # os.system('sudo rmmod xpad')

        # 设置独立环境变量
        env = os.environ.copy()
        env["SDL_JOYSTICK_DRIVER"] = "xboxdrv"

        # 启动 xboxdrv 子进程
        self.proc = subprocess.Popen(
            ["sudo", "xboxdrv", "--no-uinput", "--detach-kernel-driver"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=env,
        )
        self.pipe = self.proc.stdout

        # 初始化手柄状态
        self.connectStatus = False
        self.reading = "0" * 140
        self.refreshTime = 0
        self.refreshDelay = 1.0 / refreshRate

        # 检查 xboxdrv 输出，确保成功启动
        found = False
        waitTime = time.time() + 5  # 增加超时时间
        while waitTime > time.time() and not found:
            readable, _, _ = select.select([self.pipe], [], [], 0)
            if readable:
                response = self.pipe.readline().decode("utf-8", errors="ignore")
                print(f"xboxdrv output: {response.strip()}")
                # 如果输出中包含 "Controller:" 则认为手柄已检测到
                if "Controller:" in response:
                    found = True
                    self.connectStatus = True

        if not found:
            self.close()
            raise IOError("Unable to detect Xbox controller/receiver - Check xboxdrv configuration")

    """Used by all Joystick methods to read the most recent events from xboxdrv.
    The refreshRate determines the maximum frequency with which events are checked.
    If a valid event response is found, then the controller is flagged as 'connected'.
    """

    def refresh(self):
        # Refresh the joystick readings based on regular defined freq
        if self.refreshTime < time.time():
            self.refreshTime = time.time() + self.refreshDelay  # set next refresh time
            # If there is text available to read from xboxdrv, then read it.
            readable, writeable, exception = select.select([self.pipe], [], [], 0)
            if readable:
                # Read every line that is availabe.  We only need to decode the
                # last one.
                while readable:
                    response = self.pipe.readline()
                    # A zero length response means controller has been
                    # unplugged.
                    if len(response) == 0:
                        raise IOError("Xbox controller disconnected from USB")
                    readable, writeable, exception = select.select([self.pipe], [], [], 0)
                # Valid controller response will be 140 chars.
                if len(response) == 140:
                    self.connectStatus = True
                    self.reading = response
                else:  # Any other response means we have lost wireless or controller battery
                    self.connectStatus = False

    """Return a status of True, when the controller is actively connected.
    Either loss of wireless signal or controller powering off will break connection.  The
    controller inputs will stop updating, so the last readings will remain in effect.  It is
    good practice to only act upon inputs if the controller is connected.  For instance, for
    a robot, stop all motors if "not connected()".

    An inital controller input, stick movement or button press, may be required before the connection
    status goes True.  If a connection is lost, the connection will resume automatically when the
    fault is corrected.
    """

    def connected(self):
        self.refresh()
        return self.connectStatus

    # Left stick X axis value scaled between -1.0 (left) and 1.0 (right) with
    # deadzone tolerance correction
    def leftX(self, deadzone=4000):
        self.refresh()
        raw = int(self.reading[3:9])
        return self.axisScale(raw, deadzone)

    # Left stick Y axis value scaled between -1.0 (down) and 1.0 (up)
    def leftY(self, deadzone=4000):
        self.refresh()
        raw = int(self.reading[13:19])
        return self.axisScale(raw, deadzone)

    # Right stick X axis value scaled between -1.0 (left) and 1.0 (right)
    def rightX(self, deadzone=4000):
        self.refresh()
        raw = int(self.reading[24:30])
        return self.axisScale(raw, deadzone)

    # Right stick Y axis value scaled between -1.0 (down) and 1.0 (up)
    def rightY(self, deadzone=4000):
        self.refresh()
        raw = int(self.reading[34:40])
        return self.axisScale(raw, deadzone)

    # Scale raw (-32768 to +32767) axis with deadzone correcion
    # Deadzone is +/- range of values to consider to be center stick (ie. 0.0)
    def axisScale(self, raw, deadzone):
        if abs(raw) < deadzone:
            return 0.0
        else:
            if raw < 0:
                return (raw + deadzone) / (32768.0 - deadzone)
            else:
                return (raw - deadzone) / (32767.0 - deadzone)

    # Dpad Up status - returns 1 (pressed) or 0 (not pressed)
    def dpadUp(self):
        self.refresh()
        return int(self.reading[45:46])

    # Dpad Down status - returns 1 (pressed) or 0 (not pressed)
    def dpadDown(self):
        self.refresh()
        return int(self.reading[50:51])

    # Dpad Left status - returns 1 (pressed) or 0 (not pressed)
    def dpadLeft(self):
        self.refresh()
        return int(self.reading[55:56])

    # Dpad Right status - returns 1 (pressed) or 0 (not pressed)
    def dpadRight(self):
        self.refresh()
        return int(self.reading[60:61])

    # Back button status - returns 1 (pressed) or 0 (not pressed)
    def Back(self):
        self.refresh()
        return int(self.reading[68:69])

    # Guide button status - returns 1 (pressed) or 0 (not pressed)
    def Guide(self):
        self.refresh()
        return int(self.reading[76:77])

    # Start button status - returns 1 (pressed) or 0 (not pressed)
    def Start(self):
        self.refresh()
        return int(self.reading[84:85])

    # Left Thumbstick button status - returns 1 (pressed) or 0 (not pressed)
    def leftThumbstick(self):
        self.refresh()
        return int(self.reading[90:91])

    # Right Thumbstick button status - returns 1 (pressed) or 0 (not pressed)
    def rightThumbstick(self):
        self.refresh()
        return int(self.reading[95:96])

    # A button status - returns 1 (pressed) or 0 (not pressed)
    def A(self):
        self.refresh()
        return int(self.reading[100:101])

    # B button status - returns 1 (pressed) or 0 (not pressed)
    def B(self):
        self.refresh()
        return int(self.reading[104:105])

    # X button status - returns 1 (pressed) or 0 (not pressed)
    def X(self):
        self.refresh()
        return int(self.reading[108:109])

    # Y button status - returns 1 (pressed) or 0 (not pressed)
    def Y(self):
        self.refresh()
        return int(self.reading[112:113])

    # Left Bumper button status - returns 1 (pressed) or 0 (not pressed)
    def leftBumper(self):
        self.refresh()
        return int(self.reading[118:119])

    # Right Bumper button status - returns 1 (pressed) or 0 (not pressed)
    def rightBumper(self):
        self.refresh()
        return int(self.reading[123:124])

    # Left Trigger value scaled between 0.0 to 1.0
    def leftTrigger(self):
        self.refresh()
        return int(self.reading[129:132]) / 255.0

    # Right trigger value scaled between 0.0 to 1.0
    def rightTrigger(self):
        self.refresh()
        return int(self.reading[136:139]) / 255.0

    # Returns tuple containing X and Y axis values for Left stick scaled between -1.0 to 1.0
    # Usage:
    #     x,y = joy.leftStick()
    def leftStick(self, deadzone=4000):
        self.refresh()
        return (self.leftX(deadzone), self.leftY(deadzone))

    # Returns tuple containing X and Y axis values for Right stick scaled between -1.0 to 1.0
    # Usage:
    #     x,y = joy.rightStick()
    def rightStick(self, deadzone=4000):
        self.refresh()
        return (self.rightX(deadzone), self.rightY(deadzone))

    # Cleanup by ending the xboxdrv subprocess
    def close(self):
        os.system("pkill xboxdrv")


# Example usage of the Joystick class


def main():
    # 实例化 Joystick 类
    try:
        joy = Joystick()  # 每秒刷新 30 次
        print(
            "Joystick initialized. Press buttons or move sticks to see the output.\nPress Ctrl+C to exit."
        )
    except IOError as e:
        print(f"Error initializing joystick: {e}")
        return

    try:
        while True:
            # 检查手柄连接状态
            if not joy.connected():
                print("Controller gets no response. Please press some button.")
                time.sleep(1)
                continue

            # 按钮状态检测
            print("==== Button States ====")
            print(f"A: {joy.A()}, B: {joy.B()}, X: {joy.X()}, Y: {joy.Y()}")
            print(f"Back: {joy.Back()}, Start: {joy.Start()}, Guide: {joy.Guide()}")
            print(
                f"Left Thumbstick Button: {joy.leftThumbstick()}, Right Thumbstick Button: {joy.rightThumbstick()}"
            )
            print(f"Left Bumper: {joy.leftBumper()}, Right Bumper: {joy.rightBumper()}")

            # 摇杆状态检测
            left_x, left_y = joy.leftStick()
            right_x, right_y = joy.rightStick()
            print("\n==== Stick States ====")
            print(f"Left Stick - X: {left_x:.2f}, Y: {left_y:.2f}")
            print(f"Right Stick - X: {right_x:.2f}, Y: {right_y:.2f}")

            # 触发键状态检测
            print("\n==== Trigger States ====")
            print(f"Left Trigger: {joy.leftTrigger():.2f}, Right Trigger: {joy.rightTrigger():.2f}")

            # 方向键状态检测
            print("\n==== D-Pad States ====")
            print(
                f"Up: {joy.dpadUp()}, Down: {joy.dpadDown()}, Left: {joy.dpadLeft()}, Right: {joy.dpadRight()}"
            )

            # 暂停 0.1 秒，避免刷屏太快
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        joy.close()  # 关闭 joystick 对象，释放资源
        print("Joystick closed.")


if __name__ == "__main__":
    main()
