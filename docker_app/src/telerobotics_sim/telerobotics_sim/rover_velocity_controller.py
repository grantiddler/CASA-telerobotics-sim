#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Vector3
import math

# ---------------------------------------------------------------------------
# Rover geometry constants
#   WHEEL_RADIUS  : radius of each wheel [m]
#   HALF_TRACK    : lateral distance from rover centre-line to wheel centre [m]
#
# These are derived from the MuJoCo model:
#   - Wheel cylinders: size="0.05 0.05"  -> radius = 0.05 m
#   - Wheel X offsets: ±0.21779 m       -> half-track ≈ 0.218 m
# ---------------------------------------------------------------------------
WHEEL_RADIUS = 0.05   # [m]  (cylinder radius in MJCF)
HALF_TRACK   = 0.218  # [m]  (approx. lateral wheel offset)

# 2.5 m/s max linear wheel surface speed → 2.5 / 0.05 = 50 rad/s
MAX_WHEEL_VEL  = 50.0  # [rad/s]  hard saturation (from real-model spec)

# Slew rate limit: max change in wheel angular velocity per second.
# At 100 Hz this is 0.05 rad/s per control step → ~1.0 s to reach 5 rad/s cmd.
# Increase to ramp faster; decrease to be gentler on the rockers.
MAX_SLEW_RATE  = 5.0  # [rad/s²]


class RoverVelocityController(Node):
    """Closed-loop rover controller that commands wheel angular velocities.

    The MuJoCo model uses <velocity> actuators, so writing to ctrl[] sets a
    desired angular velocity in rad/s for each wheel.  This controller:
    1. Converts the cmd_vel (v, ω) into desired left/right wheel angular
       velocities using differential-drive kinematics (feedforward).
    2. Adds a P-correction term using the chassis pose to estimate actual
       forward velocity and yaw rate (outer feedback loop).
    3. Publishes the setpoints on the 'control' topic consumed by mujoco_node.
    """

    def __init__(self):
        super().__init__('rover_velocity_controller')

        # ---------- subscribers ----------
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.pose_sub = self.create_subscription(
            Pose, 'pose', self.pose_callback, 10)

        # ---------- publisher ----------
        self.control_pub = self.create_publisher(Vector3, 'control', 10)

        # ---------- state ----------
        self.target_v     = 0.0  # desired forward speed   [m/s]
        self.target_omega = 0.0  # desired yaw rate        [rad/s]

        self.last_pose = None
        self.last_time = None
        self.last_yaw  = 0.0

        # P-controller gains
        # kp_linear  : [wheel rad/s] added per [m/s]   of forward-velocity error
        # kp_angular : [wheel rad/s] added per [rad/s] of yaw-rate error
        # Start conservative — feedforward already does most of the work.
        self.kp_linear  = 2.0
        self.kp_angular = 1.0

        # Low-pass filter coefficient for velocity estimates (0 = no filter, 1 = frozen)
        # Higher alpha → smoother but slower; 0.7 is a reasonable starting point.
        self.alpha = 0.7
        self.v_filt     = 0.0  # filtered forward velocity [m/s]
        self.omega_filt = 0.0  # filtered yaw rate        [rad/s]

        # Slew state: track the last output so we can rate-limit the next one
        self.last_omega_left  = 0.0  # [rad/s]
        self.last_omega_right = 0.0  # [rad/s]

        self.get_logger().info(
            "Rover Velocity Controller started. "
            f"wheel_radius={WHEEL_RADIUS} m, half_track={HALF_TRACK} m")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _apply_slew(target: float, current: float, max_delta: float) -> float:
        """Clamp the step from current to target to ±max_delta."""
        return current + max(min(target - current, max_delta), -max_delta)

    def cmd_vel_callback(self, msg: Twist):
        self.target_v     = msg.linear.x
        self.target_omega = msg.angular.z

    def euler_yaw(self, x, y, z, w) -> float:
        """Return yaw (rotation about Z) from a quaternion."""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ------------------------------------------------------------------
    # Main control loop (called at pose publish rate, ~100 Hz)
    # ------------------------------------------------------------------

    def pose_callback(self, msg: Pose):
        current_time = self.get_clock().now().nanoseconds / 1e9
        current_yaw  = self.euler_yaw(
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w)

        if self.last_pose is None or self.last_time is None:
            self.last_pose = msg
            self.last_time = current_time
            self.last_yaw  = current_yaw
            return

        dt = current_time - self.last_time
        if dt <= 0.0:
            return

        # ---- estimate actual forward speed from chassis displacement ----
        # MuJoCo model convention: X is right, Y is forward.
        dx = msg.position.x - self.last_pose.position.x
        dy = msg.position.y - self.last_pose.position.y

        # Project world-frame displacement into vehicle body Y axis
        v_forward = (dy * math.cos(current_yaw) - dx * math.sin(current_yaw)) / dt

        # ---- estimate actual yaw rate ----
        dyaw  = math.atan2(
            math.sin(current_yaw - self.last_yaw),
            math.cos(current_yaw - self.last_yaw))
        omega = dyaw / dt

        # ---- low-pass filter velocity estimates ----
        # Raw finite-difference estimates are noisy at high rates; smooth them.
        self.v_filt    = self.alpha * self.v_filt    + (1 - self.alpha) * v_forward
        self.omega_filt = self.alpha * self.omega_filt + (1 - self.alpha) * omega

        # ---- feedforward: convert cmd_vel → wheel angular velocities ----
        #   v_wheel_left  = (v - ω · L) / r
        #   v_wheel_right = (v + ω · L) / r
        #   where L = HALF_TRACK, r = WHEEL_RADIUS
        ff_left  = (self.target_v - self.target_omega * HALF_TRACK) / WHEEL_RADIUS
        ff_right = (self.target_v + self.target_omega * HALF_TRACK) / WHEEL_RADIUS

        # ---- P-correction on velocity errors ----
        # Gains are in [wheel rad/s per chassis velocity unit] — no extra kinematic
        # scaling so the gains stay intuitive and bounded.
        error_v     = self.target_v     - self.v_filt
        error_omega = self.target_omega - self.omega_filt

        corr_v     = self.kp_linear  * error_v
        corr_omega = self.kp_angular * error_omega

        # Positive omega (left turn) → right wheel faster, left wheel slower
        omega_left  = ff_left  + corr_v - corr_omega
        omega_right = ff_right + corr_v + corr_omega

        # ---- saturate ----
        omega_left  = max(-MAX_WHEEL_VEL, min(MAX_WHEEL_VEL, omega_left))
        omega_right = max(-MAX_WHEEL_VEL, min(MAX_WHEEL_VEL, omega_right))

        # ---- slew rate limit ----
        # Prevent large step changes that jerk the rocker suspension.
        max_delta = MAX_SLEW_RATE * dt
        omega_left  = self._apply_slew(omega_left,  self.last_omega_left,  max_delta)
        omega_right = self._apply_slew(omega_right, self.last_omega_right, max_delta)
        self.last_omega_left  = omega_left
        self.last_omega_right = omega_right

        # ---- publish ----
        # mujoco_node maps:  ctrl = [msg.x, msg.x, msg.y, msg.y, 0, 0]
        #   msg.x → left  wheels (front + back)
        #   msg.y → right wheels (front + back)
        control_msg = Vector3()
        control_msg.x = float(omega_left)
        control_msg.y = float(omega_right)
        control_msg.z = 0.0
        self.control_pub.publish(control_msg)

        # ---- advance state ----
        self.last_pose = msg
        self.last_time = current_time
        self.last_yaw  = current_yaw


def main(args=None):
    rclpy.init(args=args)
    node = RoverVelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
