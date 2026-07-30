#!/home/project/ASV/vir_env_main/bin/python3
# Shebang specifying the Python interpreter inside the virtual environment

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial


class ThrusterSubscriber(Node):
    """
    ROS2 node that subscribes to left and right thruster topics
    and sends them over a serial connection to an Arduino.
    """

    def __init__(self):
        # Initialize the ROS2 node
        super().__init__('thruster_subscriber')

        # Subscribe to left thruster topic
        self.left_sub = self.create_subscription(
            Float32,
            'thrust_left',
            self.listener_callback_1,
            10
        )

        # Subscribe to right thruster topic
        self.right_sub = self.create_subscription(
            Float32,
            'thrust_right',
            self.listener_callback_2,
            10
        )

        self.battery_pub = self.create_publisher(Float32, 'battery_adc', 10)

        # Open serial connection to the thruster controller
        try:
            self.serial_conn = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            raise


        # Timer periods
        self.timer_period = 0.1   # 5 Hz: send thrust commands
        self.timer_period_2 = 5   # Every 5 seconds: clear serial buffers

        # Timers
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.timer_2 = self.create_timer(self.timer_period_2, self.timer_callback_2)

        # Latest thrust values from subscribers
        self.thrust_left = 0.0
        self.thrust_right = 0.0

        self.get_logger().info("Commands are now being published to Thrusters!")

    def listener_callback_1(self, left: Float32):
        """
        Callback for left thruster topic.
        Stores the latest left thrust value.
        """
        self.thrust_left = left.data

    def listener_callback_2(self, right: Float32):
        """
        Callback for right thruster topic.
        Stores the latest right thrust value.
        """
        self.thrust_right = right.data

    def timer_callback(self):
        """
        Periodic callback that:
        - Logs thrust values
        - Sends thrust commands over serial
        """
        self.get_logger().info(f'L: {self.thrust_left}, R: {self.thrust_right}\n')

        # Send thrust data as a formatted string
        self.serial_conn.write(
            f'L: {self.thrust_left}, R: {self.thrust_right}\n'.encode()
        )

        # Ensure data is transmitted immediately
        self.serial_conn.flush()

        try:
            line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()

            if line:  # only process when something arrives
                if "Battery_adc" in line:
                    parts = line.split("=")

                    if len(parts) == 2:
                        adc_value = float(parts[1].strip())

                        msg = Float32()
                        msg.data = adc_value
                        self.battery_pub.publish(msg)

                        self.get_logger().info(f"Battery ADC: {adc_value}\n")

        except Exception as e:
            self.get_logger().warn(f"ADC read error: {e}")

    def timer_callback_2(self):
        """
        Periodically reset serial input/output buffers
        to avoid overflow or stale data.
        """
        self.serial_conn.reset_input_buffer()
        self.serial_conn.reset_output_buffer()

    def destroy_node(self):
        """
        Clean shutdown:
        - Close serial connection
        - Destroy ROS node
        """
        self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    """
    Entry point for the thruster subscriber node.
    """
    rclpy.init(args=args)
    node = ThrusterSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
