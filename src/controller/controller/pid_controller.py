#!/usr/bin/env python3
"""
PID Heading Controller + Keyboard Motion Control for ASV
=========================================================
Nomoto first-order model parameters:
    K = 0.0134   rad/s per %thrust
    T = 3.490883 s
    c = 0.122    rad/s  (bias offset)

Ziegler-Nichols Method 1 (Reaction Curve) PID parameters:
    R  = K/T  = 0.003839
    L  = 0.1*T = 0.3491 s
    Kp = 1.2/(R*L) = 896.17
    Ti = 2*L        = 0.6982 s  →  Ki = 1283.52
    Td = 0.5*L      = 0.1746 s  →  Kd = 156.43

Subscriptions:
    /imu/euler_degrees  (geometry_msgs/Vector3)  — yaw from z field

Publications:
    /thrust_left   (std_msgs/Float32)
    /thrust_right  (std_msgs/Float32)

Keyboard controls (non-blocking, read every control tick):
    w   — forward surge          (PID holds heading)
    s   — backward surge         (PID holds heading)
    a   — turn left  open-loop   (PID bypassed, integral frozen)
    d   — turn right open-loop   (PID bypassed, integral frozen)
    q   — stop all + latch current yaw as new heading
    +   — increase throttle
    -   — decrease throttle
    h   — latch current yaw as desired heading (without stopping)
    x   — exit

Thruster mixing:
    w/s  (PID active):
        left  =  surge + pid_output
        right = -surge + pid_output

    a/d  (PID bypassed — open-loop spin):
        turn left  → left = -throttle,  right = -throttle
        turn right → left = +throttle,  right = +throttle
        On q: current yaw is latched as new desired heading.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
import math
import sys
import select
import termios
import tty


# ── Model parameters ──────────────────────────────────────────────────────────
K_MODEL = 0.0134
T_MODEL = 3.490883
C_BIAS  = 0.122

# ── PID parameters ────────────────────────────────────────────────────────────
KP_DEFAULT = 97.950628216005
KI_DEFAULT = 55.2395528717595
KD_DEFAULT = -18.3025288716835
N_FILTER   =  5.35175378783574   # Kd low-pass filter coefficient
# D term with filter: D(s) = Kd * N*s / (s + N)
# Discrete: filtered_D = (Kd*N*error - Kd*N*prev_error - prev_D*(−N*dt)) / (1 + N*dt)
# Simplified: prev_D updated each tick as below

# ── Thruster limits ───────────────────────────────────────────────────────────
THRUST_MIN = -50.0
THRUST_MAX =  50.0

# ── Throttle steps ────────────────────────────────────────────────────────────
THROTTLE_STEP = 5.0
THROTTLE_MIN  = 10.0
THROTTLE_MAX  = 50.0

# ── PID output rate limiter ───────────────────────────────────────────────────
# Maximum change in PID output allowed per second.
# Prevents the thruster command from jumping violently between ticks.
# At 10 Hz with MAX_OUTPUT_RATE=20, output can change at most 2% per tick.
MAX_OUTPUT_RATE = 20.0   # % thrust per second


def wrap_angle(angle: float) -> float:
    """Wrap angle to [-180, 180] degrees."""
    return (angle + 180.0) % 360.0 - 180.0


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class ASVController(Node):
    """
    Combined keyboard motion + PID heading controller for an ASV.

    Modes
    -----
    SURGE   (w/s) : PID holds heading, keyboard sets forward/backward thrust
    TURNING (a/d) : Open-loop spin, PID integral frozen, no heading correction
    HOLD    (idle): PID holds last latched heading, zero surge
    """

    def __init__(self):
        super().__init__('asv_controller')

        # ── ROS parameters ────────────────────────────────────────────────────
        self.declare_parameter('desired_heading_deg', 0.0)
        self.declare_parameter('Kp', KP_DEFAULT)
        self.declare_parameter('Ki', KI_DEFAULT)
        self.declare_parameter('Kd', KD_DEFAULT)
        self.declare_parameter('control_rate_hz', 10.0)

        self.desired_heading = self.get_parameter('desired_heading_deg').value
        self.Kp  = self.get_parameter('Kp').value
        self.Ki  = self.get_parameter('Ki').value
        self.Kd  = self.get_parameter('Kd').value
        rate_hz  = self.get_parameter('control_rate_hz').value
        self.dt  = 1.0 / rate_hz

        # ── PID state ─────────────────────────────────────────────────────────
        self.integral       = 0.0
        self.prev_error     = 0.0
        self.prev_pid_out   = 0.0   # for rate limiting
        self.filtered_D     = 0.0   # Kd filter state
        self.first_tick     = True
        self.integral_limit = THRUST_MAX / max(KI_DEFAULT, 1e-6)

        # ── IMU state ─────────────────────────────────────────────────────────
        self.current_yaw  = 0.0
        self.imu_received = False

        # ── Motion state ──────────────────────────────────────────────────────
        self.surge          = 0.0     # +fwd / -rev, used in SURGE mode
        self.throttle       = 20.0    # magnitude for all motion commands
        self.turning        = False   # True while a/d held
        self.turn_direction = 0.0     # +1.0 = right,  -1.0 = left

        # ── Publishers / subscribers ──────────────────────────────────────────
        self.left_pub  = self.create_publisher(Float32, 'thrust_left',  10)
        self.right_pub = self.create_publisher(Float32, 'thrust_right', 10)

        self.imu_sub = self.create_subscription(
            Vector3, 'imu/euler_degrees', self.imu_callback, 10)

        # ── Non-blocking keyboard setup ───────────────────────────────────────
        self.fd           = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

        # ── Control loop timer ────────────────────────────────────────────────
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f"\n{'='*54}\n"
            f"  ASV Controller started\n"
            f"  Model : K={K_MODEL}  T={T_MODEL}  c={C_BIAS}\n"
            f"  PID   : Kp={self.Kp:.2f}  Ki={self.Ki:.2f}  Kd={self.Kd:.2f}\n"
            f"  Desired heading : {self.desired_heading:.1f} deg\n"
            f"  Rate  : {rate_hz:.0f} Hz\n"
            f"{'='*54}\n"
            f"  w/s : forward / backward   (PID holds heading)\n"
            f"  a/d : turn left / right    (open-loop, PID off)\n"
            f"  q   : stop all + latch heading\n"
            f"  +/- : throttle             (now {self.throttle:.0f}%)\n"
            f"  h   : latch heading without stopping\n"
            f"  x   : exit\n"
            f"{'='*54}"
        )

    # ── IMU callback ──────────────────────────────────────────────────────────
    def imu_callback(self, msg: Vector3):
        self.current_yaw  = msg.z
        self.imu_received = True

    # ── Non-blocking key read ─────────────────────────────────────────────────
    def _get_key(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    # ── Keyboard handler ──────────────────────────────────────────────────────
    def _handle_key(self, key: str):

        if key == 'w':
            # Forward surge — PID will hold heading
            self.surge          =  self.throttle
            self.turning        = False
            self.turn_direction = 0.0

        elif key == 's':
            # Backward surge — PID will hold heading
            self.surge          = -self.throttle
            self.turning        = False
            self.turn_direction = 0.0

        elif key == 'a':
            # Open-loop left turn — both thrusters same sign (teleop convention)
            # PID integral is frozen during turn to prevent windup
            self.surge          = 0.0
            self.turning        = True
            self.turn_direction = -1.0

        elif key == 'd':
            # Open-loop right turn
            self.surge          = 0.0
            self.turning        = True
            self.turn_direction =  1.0

        elif key == 'q':
            # Full stop — latch current yaw so PID holds the new heading
            self.surge          = 0.0
            self.turning        = False
            self.turn_direction = 0.0
            self._latch_heading()

        elif key == '+':
            self.throttle = clamp(self.throttle + THROTTLE_STEP,
                                  THROTTLE_MIN, THROTTLE_MAX)
            # Update surge magnitude live if moving
            if self.surge != 0.0:
                self.surge = math.copysign(self.throttle, self.surge)
            self.get_logger().info(f"Throttle → {self.throttle:.0f}%")

        elif key == '-':
            self.throttle = clamp(self.throttle - THROTTLE_STEP,
                                  THROTTLE_MIN, THROTTLE_MAX)
            if self.surge != 0.0:
                self.surge = math.copysign(self.throttle, self.surge)
            self.get_logger().info(f"Throttle → {self.throttle:.0f}%")

        elif key == 'h':
            # Latch heading without stopping motion
            self._latch_heading()

        elif key == 'x':
            self.get_logger().info("Exit key pressed.")
            self._stop()
            raise KeyboardInterrupt

    def _latch_heading(self):
        """Set desired heading to current yaw and reset PID state."""
        self.desired_heading = wrap_angle(self.current_yaw)
        self.integral        = 0.0
        self.prev_error      = 0.0
        self.prev_pid_out    = 0.0
        self.filtered_D      = 0.0
        self.first_tick      = True
        self.get_logger().info(f"Heading latched → {self.desired_heading:.2f}°")

    # ── Main control loop (10 Hz) ─────────────────────────────────────────────
    def control_loop(self):
        # Read keyboard (non-blocking — never stalls the loop)
        key = self._get_key()
        if key:
            self._handle_key(key)

        if not self.imu_received:
            self.get_logger().warn(
                'Waiting for IMU...', throttle_duration_sec=2.0)
            self._publish_thrust(0.0, 0.0)
            return

        # ── Re-read tunable ROS params ────────────────────────────────────────
        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value

        # ── PID computation (always runs, but output only used outside TURN) ──
        error = wrap_angle(self.desired_heading - self.current_yaw)

        P = self.Kp * error

        # Freeze integral during open-loop turns only
        if not self.turning:
            self.integral = clamp(
                self.integral + error * self.dt,
                -self.integral_limit, self.integral_limit)
        I = self.Ki * self.integral

        # ── Filtered derivative: D(s) = Kd * N*s / (s + N) ──────────────────
        # Discrete form (Euler backward):
        #   filtered_D = (filtered_D + N*dt * Kd * (error - prev_error)/dt) / (1 + N*dt)
        # Equivalently:
        #   alpha = N*dt / (1 + N*dt)
        #   filtered_D = (1-alpha)*filtered_D + alpha*Kd*N*(error - prev_error)
        if self.first_tick:
            self.filtered_D = 0.0
            self.first_tick = False
        else:
            N   = N_FILTER
            alpha = (N * self.dt) / (1.0 + N * self.dt)
            self.filtered_D = ((1.0 - alpha) * self.filtered_D
                               + alpha * self.Kd * N * (error - self.prev_error))
        D = self.filtered_D
        self.prev_error = error

        pid_output = P + I + D

        # ── Rate limiter ──────────────────────────────────────────────────────
        # Clamp how fast the PID output can change each tick.
        # This smooths out sudden jumps caused by noise spikes or setpoint changes.
        max_delta = MAX_OUTPUT_RATE * self.dt
        pid_output = clamp(
            pid_output,
            self.prev_pid_out - max_delta,
            self.prev_pid_out + max_delta
        )
        self.prev_pid_out = pid_output

        # ── Thruster mixing ───────────────────────────────────────────────────
        if self.turning:
            # Open-loop yaw — both thrusters same sign
            # Teleop convention: right → both +throttle, left → both -throttle
            t = self.throttle * self.turn_direction
            left_thrust  = clamp(t, THRUST_MIN, THRUST_MAX)
            right_thrust = clamp(t, THRUST_MIN, THRUST_MAX)
            mode_str = f"TURN {'R' if self.turn_direction > 0 else 'L'}"

        else:
            # Surge + PID heading hold
            # Forward:  left = +surge,  right = -surge
            # Heading:  left = +pid,    right = +pid
            # Combined: left = surge + pid,  right = -surge + pid
            left_thrust  = clamp( self.surge + pid_output, THRUST_MIN, THRUST_MAX)
            right_thrust = clamp(-self.surge + pid_output, THRUST_MIN, THRUST_MAX)
            mode_str = ("FWD" if self.surge > 0
                        else "REV" if self.surge < 0
                        else "HOLD")


        self._publish_thrust(left_thrust, right_thrust)

        self.get_logger().info(
            f"[{mode_str:5s}]  yaw={self.current_yaw:+7.2f}°  "
            f"des={self.desired_heading:+7.2f}°  err={error:+6.2f}°  "
            f"surge={self.surge:+5.1f}%  pid={pid_output:+7.2f}  "
            f"L={left_thrust:+6.1f}%  R={right_thrust:+6.1f}%",
            throttle_duration_sec=0.5
        )

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _publish_thrust(self, left: float, right: float):
        l = Float32(); l.data = float(left)
        r = Float32(); r.data = float(right)
        self.left_pub.publish(l)
        self.right_pub.publish(r)

    def _stop(self):
        self._publish_thrust(0.0, 0.0)
        self.get_logger().info("Thrusters stopped.")

    def destroy_node(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ASVController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()