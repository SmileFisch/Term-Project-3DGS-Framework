"""
EncoderManager Node for ROS 2 Quadrature Encoders
Monitors GPIO A/B channels, maintains pulse count with overflow correction,
and publishes Int32 pulse counts on configurable topics at 10 Hz.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from gpiozero import DigitalInputDevice
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class EncoderManager(Node):
    """
    编码器管理节点，负责读取编码器数据并发布脉冲计数。
    """

    def __init__(self, enc_a_pin, enc_b_pin, topic_name, max_value=32767):
        super().__init__("encoder_manager_" + topic_name.split("/")[-1])  # 动态命名节点

        # 初始化编码器引脚
        self.enc_a = DigitalInputDevice(enc_a_pin)
        self.enc_b = DigitalInputDevice(enc_b_pin)

        # 编码器状态
        self.pulse_count = 0  # 当前脉冲计数
        self.last_pulse_count = 0  # 上次记录的脉冲计数
        self.direction = 1  # 当前方向
        self.max_value = max_value  # 最大计数值，用于溢出校正

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=20, reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.publisher = self.create_publisher(Int32, topic_name, qos)

        self.timer = self.create_timer(0.1, self.publish_pulse_count)

        # Test
        # self.delta_timer = self.create_timer(5.0, self.log_delta_pulse)

        # 绑定编码器 A 相的回调
        self.enc_a.when_activated = self._a_phase_changed

        self.get_logger().info("EncoderManagerNode has been started.")

    def _a_phase_changed(self):
        """
        A 相信号变化时的回调函数，更新脉冲计数和方向。
        """
        if self.enc_b.value == 0:
            self.direction = 1  # 顺时针
        else:
            self.direction = -1  # 逆时针

        self.pulse_count += self.direction
        self.pulse_count = self.correct_overflow(self.pulse_count)

    def correct_overflow(self, value):
        """
        校正编码器计数的溢出。
        :param value: 当前脉冲计数
        :return: 校正后的值
        """
        if value > self.max_value:
            value -= (self.max_value + 1) * 2
        elif value < -self.max_value - 1:
            value += (self.max_value + 1) * 2
        return value

    def publish_pulse_count(self):
        """
        发布当前编码器的脉冲计数。
        """
        msg = Int32()
        msg.data = self.pulse_count
        self.publisher.publish(msg)

    def log_delta_pulse(self):
        """
        在时间间隔内计算并通过日志显示脉冲计数变化。
        """
        delta_pulse = self.pulse_count - self.last_pulse_count
        self.last_pulse_count = self.pulse_count

        self.get_logger().info(f"Delta pulse count over last second: {delta_pulse}")

    def destroy(self):
        """
        销毁节点，清理资源，并重置编码器状态。
        """
        # 重置编码器状态
        self.pulse_count = 0
        self.direction = 1

        # 关闭 GPIO 引脚
        self.enc_a.close()
        self.enc_b.close()

        # 销毁 ROS 2 节点
        super().destroy_node()


def main(args=None):
    """
    主函数，启动多个编码器管理节点。
    """
    rclpy.init(args=args)

    # 配置每个编码器的引脚和话题
    encoder_configs = [
        {"enc_a_pin": 25, "enc_b_pin": 17, "topic_name": "/encoder/pulse_count_1"},  # 左前轮
        {"enc_a_pin": 26, "enc_b_pin": 21, "topic_name": "/encoder/pulse_count_2"},  # 左后轮
        {"enc_a_pin": 23, "enc_b_pin": 24, "topic_name": "/encoder/pulse_count_3"},  # 右前轮
        {"enc_a_pin": 0, "enc_b_pin": 11, "topic_name": "/encoder/pulse_count_4"},  # 右后轮
    ]

    # 创建所有编码器节点实例
    encoder_nodes = [EncoderManager(**config) for config in encoder_configs]

    # 创建一个多线程执行器
    executor = rclpy.executors.MultiThreadedExecutor()

    # 添加所有节点到执行器
    for node in encoder_nodes:
        executor.add_node(node)

    try:
        # 运行所有节点
        executor.spin()
    except KeyboardInterrupt:
        for node in encoder_nodes:
            node.get_logger().info("Stopping Encoder Manager...")
            node.destroy_node()
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
