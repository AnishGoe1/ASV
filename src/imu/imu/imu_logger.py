#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import time
import csv
import os
from datetime import datetime


class IMUDataLogger(Node):

    def __init__(self):
        super().__init__('imu_data_logger')

        # ========================
        # Fixed log directory
        # ========================
        self.LOG_DIR = "/home/project/ASV/DATA/IMU_Data"
        os.makedirs(self.LOG_DIR, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        base_filename = f"imu_data_{timestamp}"
        self.file_name = self._get_unique_filename(base_filename)

        # ========================
        # CSV setup
        # ========================
        self.header = [
            'timestamp(s)',
            'roll(deg)', 'pitch(deg)', 'yaw(deg)',
            'ang_vel_x(rad/s)', 'ang_vel_y(rad/s)', 'ang_vel_z(rad/s)'
        ]

        with open(self.file_name, 'w', newline='') as f:
            csv.writer(f).writerow(self.header)

        self.get_logger().info(f"Logging IMU data to {self.file_name}")

        # ========================
        # Subscriptions
        # ========================
        self.create_subscription(
            Vector3,
            'imu/euler_degrees',
            self.euler_callback,
            10
        )

        self.create_subscription(
            Vector3,
            'imu/angular_velocity',
            self.angular_velocity_callback,
            10
        )

        # ========================
        # Data storage
        # ========================
        self.euler = None
        self.angular_velocity = None

        self.start_time = time.monotonic()
        self.data_batch = []
        self.BATCH_SIZE = 50

        # ========================
        # Timer (50 Hz)
        # ========================
        self.timer = self.create_timer(0.02, self.log_data)

    # ========================
    # Filename helper
    # ========================
    def _get_unique_filename(self, base_name):
        filename = os.path.join(self.LOG_DIR, base_name + ".csv")

        counter = 1
        while os.path.exists(filename):
            filename = os.path.join(
                self.LOG_DIR,
                f"{base_name}_{counter}.csv"
            )
            counter += 1

        return filename

    # ========================
    # Callbacks
    # ========================
    def euler_callback(self, msg: Vector3):
        self.euler = [msg.x, msg.y, msg.z]

    def angular_velocity_callback(self, msg: Vector3):
        self.angular_velocity = [msg.x, msg.y, msg.z]

    # ========================
    # Logger
    # ========================
    def log_data(self):
        if self.euler is None:
            return

        timestamp = time.monotonic() - self.start_time
        roll, pitch, yaw = self.euler

        if self.angular_velocity is not None:
            wx, wy, wz = self.angular_velocity
        else:
            wx = wy = wz = float('nan')

        row = [
            f"{timestamp:.6f}",
            f"{roll:.2f}", f"{pitch:.2f}", f"{yaw:.2f}",
            f"{wx:.5f}", f"{wy:.5f}", f"{wz:.5f}"
        ]

        self.data_batch.append(row)

        self.get_logger().info(
            f"T:{timestamp:.2f} | "
            f"R:{roll:.2f} P:{pitch:.2f} Y:{yaw:.2f} | "
            f"wx:{wx:.4f} wy:{wy:.4f} wz:{wz:.4f}"
        )

        if len(self.data_batch) >= self.BATCH_SIZE:
            self.flush_to_disk()

    def flush_to_disk(self):
        try:
            with open(self.file_name, 'a', newline='') as f:
                csv.writer(f).writerows(self.data_batch)
            self.data_batch.clear()
        except IOError as e:
            self.get_logger().error(f"CSV write failed: {e}")

    # ========================
    # Shutdown
    # ========================
    def destroy_node(self):
        self.get_logger().info("Shutting down, saving remaining data...")
        if self.data_batch:
            self.flush_to_disk()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IMUDataLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
