#!/home/project/ASV/vir_env_main/bin/python3

import rclpy
from rclpy.node import Node
import math
import board, busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_GAME_ROTATION_VECTOR,   # replaces ROTATION_VECTOR — smoother yaw
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_LINEAR_ACCELERATION,
)
from geometry_msgs.msg import Quaternion, Vector3
import signal


# ── EMA filter coefficient ────────────────────────────────────────────────────
# alpha=1.0 → no filtering (raw)
# alpha=0.2 → strong smoothing, ~5-sample lag at 10 Hz
YAW_EMA_ALPHA = 0.2


class IMU_Data_Publisher(Node):
    def __init__(self):
        super().__init__('imu_data_publisher')

        self.pub_quat     = self.create_publisher(Quaternion, 'imu/quaternion',            10)
        self.pub_euler    = self.create_publisher(Vector3,    'imu/euler_degrees',          10)
        self.pub_ang_vel  = self.create_publisher(Vector3,    'imu/angular_velocity',       10)
        self.pub_lin_acc  = self.create_publisher(Vector3,    'imu/linear_acceleration',    10)

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        try:
            self.i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
            self.bno = BNO08X_I2C(self.i2c)

            # ── KEY CHANGE: use GAME_ROTATION_VECTOR instead of ROTATION_VECTOR ──
            # GAME_ROTATION_VECTOR fuses gyro + accelerometer without magnetometer.
            # Benefits for ASV:
            #   1. Immune to magnetic interference from motors and ESCs
            #   2. Onboard fusion gives smoother yaw vs raw ROTATION_VECTOR
            #   3. No sudden heading jumps from magnetic disturbances
            # Access via: self.bno.game_quaternion  (instead of self.bno.quaternion)
            self.bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            self.bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)

            self.get_logger().info('BNO08X initialized with GAME_ROTATION_VECTOR report.')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize BNO08X: {e}')
            self.bno = None

        self.zero_yaw = None

        # ── EMA filter state ──────────────────────────────────────────────────
        # Smooths the yaw angle after quaternion conversion.
        # Applied only to yaw (the noisy axis for a surface vehicle).
        self._yaw_filtered  = None
        self._pitch_filtered = None
        self._roll_filtered  = None

    @staticmethod
    def quat_to_euler_deg(quat):
        """Convert quaternion (i, j, k, real) → (yaw, pitch, roll) in degrees."""
        qi, qj, qk, qr = quat
        w, x, y, z = qr, qi, qj, qk
        roll  = math.degrees(math.atan2(2*(w*x - y*z), 1 - 2*(x*x + y*y)))
        pitch = math.degrees(math.asin (2*(w*y + z*x)))
        yaw   = math.degrees(math.atan2(2*(w*z - x*y), 1 - 2*(y*y + z*z)))
        return yaw, pitch, roll

    @staticmethod
    def _ema(new_val, prev_val, alpha):
        """Exponential moving average: smooth = alpha*new + (1-alpha)*prev."""
        if prev_val is None:
            return new_val          # initialise on first sample
        return alpha * new_val + (1.0 - alpha) * prev_val

    def timer_callback(self):
        if self.bno is None:
            return

        try:
            # ── Use game_quaternion for smoother heading ───────────────────────
            quat_bno         = self.bno.game_quaternion
            angular_velocity = self.bno.gyro
            linear_accel     = self.bno.linear_acceleration

        except KeyError as e:
            self.get_logger().warn(f"Corrupted IMU packet (KeyError): {e}")
            return
        except Exception as e:
            self.get_logger().error(f"Unexpected IMU error: {e}")
            return

        if quat_bno is None:
            return

        yaw, pitch, roll = self.quat_to_euler_deg(quat_bno)

        # ── EMA filter on all three angles ────────────────────────────────────
        # Yaw is filtered most aggressively since that's what the PID uses.
        # Roll/pitch are filtered with the same alpha for consistency.
        self._yaw_filtered   = self._ema(yaw,   self._yaw_filtered,   YAW_EMA_ALPHA)
        self._pitch_filtered = self._ema(pitch, self._pitch_filtered, YAW_EMA_ALPHA)
        self._roll_filtered  = self._ema(roll,  self._roll_filtered,  YAW_EMA_ALPHA)

        yaw_out   = self._yaw_filtered
        pitch_out = self._pitch_filtered
        roll_out  = self._roll_filtered

        # ── Zero-yaw reference (set once on startup) ──────────────────────────
        if self.zero_yaw is None:
            self.zero_yaw = yaw_out
            self.get_logger().info(f"Zero yaw set to {self.zero_yaw:.2f}°")

        # ── Publish quaternion (raw — consumers can filter themselves) ─────────
        quat_msg = Quaternion(
            x=float(quat_bno[0]),
            y=float(quat_bno[1]),
            z=float(quat_bno[2]),
            w=float(quat_bno[3])
        )
        self.pub_quat.publish(quat_msg)

        # ── Publish filtered euler angles ─────────────────────────────────────
        euler_msg = Vector3(x=roll_out, y=pitch_out, z=yaw_out)
        self.pub_euler.publish(euler_msg)

        # ── Publish angular velocity ──────────────────────────────────────────
        if angular_velocity is not None:
            ang_vel_msg = Vector3(
                x=float(angular_velocity[0]),
                y=float(angular_velocity[1]),
                z=float(angular_velocity[2])
            )
            self.pub_ang_vel.publish(ang_vel_msg)
            self.get_logger().info(
                f"AngVel (rad/s): x={angular_velocity[0]:.4f}  "
                f"y={angular_velocity[1]:.4f}  z={angular_velocity[2]:.4f}"
            )

        # ── Publish linear acceleration ───────────────────────────────────────
        if linear_accel is not None:
            lin_acc_msg = Vector3(
                x=float(linear_accel[0]),
                y=float(linear_accel[1]),
                z=float(linear_accel[2])
            )
            self.pub_lin_acc.publish(lin_acc_msg)
            self.get_logger().info(
                f"LinAccel (m/s²): x={linear_accel[0]:.4f}  "
                f"y={linear_accel[1]:.4f}  z={linear_accel[2]:.4f}"
            )

        self.get_logger().info(
            f"Roll:{roll_out:.2f}°  Pitch:{pitch_out:.2f}°  Yaw:{yaw_out:.2f}°\n"
        )

    def destroy_node(self):
        self.get_logger().info("IMU node shutting down.")
        super().destroy_node()


def main(args=None):
    def signal_handler(sig, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, signal_handler)

    rclpy.init(args=args)
    node = IMU_Data_Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()