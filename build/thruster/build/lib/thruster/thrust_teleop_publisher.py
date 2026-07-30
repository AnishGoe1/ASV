#!/home/project/ASV/vir_env_main/bin/python3
# Shebang to ensure the script runs with the correct Python interpreter

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import sys
import termios
import tty
import select
import time


class ThrusterTeleopPublisher(Node):
    """
    ROS2 node for keyboard-based teleoperation of Surface Robot
    Publishes Float32 thrust commands for left and right thrusters.
    """

    def __init__(self):
        super().__init__('thruster_teleop_publisher')

        # Publishers for left and right thruster topics
        self.left_pub = self.create_publisher(Float32, 'thrust_left', 10)
        self.right_pub = self.create_publisher(Float32, 'thrust_right', 10)

        # Initial throttle magnitude
        self.throttle = 10.0

        # Direction multipliers for each thruster
        self.direction1 = 0.0
        self.direction2 = 0.0

        # --- Inactivity timeout setup ---
        self.last_input_time = time.time()
        self.inactivity_timeout = 100.0  # seconds

        # --- Terminal setup for non-blocking keyboard input ---
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

        # Print user instructions
        self.get_logger().info(
            "Thruster teleop started\n"
            "w/s: forward/back\n"
            "a/d: left/right\n"
            "+/-: throttle increase/decrease\n"
            "q: STOP\n"
            "x: EXIT\n"
            "(Auto-stop after 5s inactivity)"
        )

        # Timer callback running at 10 Hz
        self.timer_duration = 0.1
        self.timer = self.create_timer(self.timer_duration, self.loop)

    def get_key_nonblocking(self):
        """
        Read a single key press if available.
        Does not block execution.
        """
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def update_direction_and_throttle(self, key):
        """
        Update thruster directions and throttle based on keyboard input.
        """
        if key == 'w':          # Forward
            self.direction1 = 1.0
            self.direction2 = -1.0
        elif key == 's':        # Backward
            self.direction1 = -1.0
            self.direction2 = 1.0
        elif key == 'a':        # Turn left
            self.direction1 = -1.0
            self.direction2 = -1.0
        elif key == 'd':        # Turn right
            self.direction1 = 1.0
            self.direction2 = 1.0
        elif key == 'q':        # Stop and reset throttle
            self.direction1 = 0.0
            self.direction2 = 0.0
            self.throttle = 10.0
        elif key == '+':        # Increase throttle
            self.throttle = min(50.0, self.throttle + 5)
        elif key == '-':        # Decrease throttle
            self.throttle = max(10.0, self.throttle - 5)

    def loop(self):
        """
        Main control loop:
        - Read keyboard input
        - Apply inactivity timeout
        - Publish thrust values
        """
        key = self.get_key_nonblocking()

        if key:
            self.last_input_time = time.time()

            if key == 'x':      # Exit command
                self.stop_and_exit()
                return

            self.update_direction_and_throttle(key)

        # --- Inactivity safety stop ---
        if time.time() - self.last_input_time > self.inactivity_timeout:
            self.direction1 = 0.0
            self.direction2 = 0.0
            self.throttle = 10.0

        # Compute thrust values
        left_thrust = self.throttle * self.direction1
        right_thrust = self.throttle * self.direction2

        # Publish thrust commands
        self.left_pub.publish(Float32(data=left_thrust))
        self.right_pub.publish(Float32(data=right_thrust))

        # Log current thrust values
        self.get_logger().info(f"L: {left_thrust:.1f}, R: {right_thrust:.1f}")

    def stop_and_exit(self):
        """
        Safely stop thrusters, restore terminal settings, and shut down ROS.
        """
        self.left_pub.publish(Float32(data=0.0))
        self.right_pub.publish(Float32(data=0.0))
        self.get_logger().info("Stopping thrusters and exiting")

        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        rclpy.shutdown()

    def destroy_node(self):
        """
        Ensure terminal settings are restored when node is destroyed.
        """
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterTeleopPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
