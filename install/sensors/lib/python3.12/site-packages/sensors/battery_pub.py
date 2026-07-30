import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np


class BatteryProcessor(Node):
    def __init__(self):
        super().__init__('battery_processor')

        # Voltage divider values
        self.R1 = 30000.0
        self.R2 = 7500.0
        self.ref_voltage = 5.0
        self.clamped_vol = 0.0

        # Battery points
        self.vol_points = [10.6, 11.5, 12.5, 12.7, 12.8, 12.9, 13.0, 13.1, 13.2, 13.4, 14.6]
        self.soc_points = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
        self.factor =  13.14 / 12.82

        # Battery range (your system)
        self.batt_min = 12.0
        self.batt_max = 13.4

        # Subscriber (ADC input)
        self.sub = self.create_subscription(
            Float32,
            'battery_adc',
            self.adc_callback,
            10
        )

        # Publisher (percentage output)
        self.pub = self.create_publisher(Float32, 'battery_percentage', 10)

        self.get_logger().info("Battery Processor Node Started")

    def adc_callback(self, msg: Float32):
        adc = msg.data

        voltage = self.adc_to_voltage(adc)
        percent = self.voltage_to_percentage(voltage)

        out_msg = Float32()
        out_msg.data = percent
        self.pub.publish(out_msg)

        self.get_logger().info(
            f"Voltage={voltage:.2f}V | Battery={percent:.1f}%"
        )

    def adc_to_voltage(self, adc):
        adc_voltage = (adc * self.ref_voltage) / 1024.0
        input_voltage = (adc_voltage * (self.R1 + self.R2) / self.R2)*self.factor
        self.clamped_vol = np.clip(input_voltage, self.vol_points[0], self.soc_points[-1])
        return self.clamped_vol

    def voltage_to_percentage(self, v):
        percent = np.interp(self.clamped_vol, self.vol_points, self.soc_points)
        return percent


def main(args=None):
    rclpy.init(args=args)
    node = BatteryProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()