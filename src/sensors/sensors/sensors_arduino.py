#!/home/project/ASV/vir_env_main/bin/python3
"""
arduino_bridge_node.py

Reads lines like "Ping: 441, 100" from the Arduino over USB serial and
publishes them as a ROS 2 Range + confidence topic. No tag dispatch table,
no handler registry - just read the line, pull the numbers out, publish.
"""

import threading

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Int32


class ArduinoBridgeNode(Node):

    def __init__(self):
        super().__init__('arduino_bridge_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'ping_sonar_link')
        self.declare_parameter('min_range_m', 0.3)
        self.declare_parameter('max_range_m', 100.0)
        self.declare_parameter('field_of_view_rad', 0.4363)  # 25 degrees

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.min_range_m = float(self.get_parameter('min_range_m').value)
        self.max_range_m = float(self.get_parameter('max_range_m').value)
        self.field_of_view_rad = float(self.get_parameter('field_of_view_rad').value)

        self.range_pub = self.create_publisher(Range, 'ping_sonar/range', 10)
        self.confidence_pub = self.create_publisher(Int32, 'ping_sonar/confidence', 10)

        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
        self.get_logger().info(f'Opened serial port {self.serial_port}')

        self._stop = threading.Event()
        threading.Thread(target=self._read_loop, daemon=True).start()

    def _read_loop(self):
        while not self._stop.is_set():
            try:
                line = self.ser.readline().decode('ascii', errors='replace').strip()
            except serial.SerialException as exc:
                self.get_logger().error(f'Serial read error: {exc}')
                break

            if not line.startswith('Ping:'):
                continue  # ignore "#..." status lines and anything else

            # line looks like "Ping: 441, 100"
            try:
                values = line[len('Ping:'):].split(',')
                distance_mm = float(values[0].strip())
                confidence_pct = int(float(values[1].strip()))
            except (ValueError, IndexError):
                self.get_logger().warn(f'Could not parse line: "{line}"')
                continue

            self._publish(distance_mm, confidence_pct)
            self.get_logger().info(f'distance={distance_mm:.0f} mm, confidence={confidence_pct}%')

    def _publish(self, distance_mm, confidence_pct):
        stamp = self.get_clock().now().to_msg()

        range_msg = Range()
        range_msg.header.stamp = stamp
        range_msg.header.frame_id = self.frame_id
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = self.field_of_view_rad
        range_msg.min_range = self.min_range_m
        range_msg.max_range = self.max_range_m
        range_msg.range = distance_mm / 1000.0
        self.range_pub.publish(range_msg)

        conf_msg = Int32()
        conf_msg.data = confidence_pct
        self.confidence_pub.publish(conf_msg)

    def destroy_node(self):
        self._stop.set()
        try:
            self.ser.close()
        except serial.SerialException:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()