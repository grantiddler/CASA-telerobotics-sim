#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3

# ---------------------------------------------------------------------------
# Rover geometry — derived from the MuJoCo model:
#   Wheel cylinder radius  : size="0.05 0.05"  → r = 0.05 m
#   Wheel lateral offset   : ±0.21779 m        → half-track L ≈ 0.218 m
# ---------------------------------------------------------------------------
WHEEL_RADIUS = 0.05   # [m]
HALF_TRACK   = 0.218  # [m]

# Max wheel angular velocity from real-model spec: 2.5 m/s ÷ 0.05 m = 50 rad/s
MAX_WHEEL_VEL = 50.0  # [rad/s]

# Slew rate: how fast the setpoint is allowed to change.
# At 100 Hz → 0.05 rad/s per step → ~1 s ramp to 5 rad/s command.
# Increase to ramp faster; decrease to be gentler on the rockers.
MAX_SLEW_RATE = 5.0   # [rad/s²]


class RoverVelocityController(Node):
    """Pure feedforward wheel velocity controller.

    Converts a cmd_vel (v [m/s], ω [rad/s]) into left/right wheel angular
    velocity setpoints using differential-drive kinematics, then applies a
    slew rate limit before publishing.  The MuJoCo <velocity> actuator closes
    the inner torque loop on each wheel — no outer chassis-velocity feedback
    is needed here.

    Differential-drive kinematics:
        ω_left  = (v − ω · L) / r
        ω_right = (v + ω · L) / r
    where L = HALF_TRACK, r = WHEEL_RADIUS.
    """

    def __init__(self):
        super().__init__('rover_velocity_controller')

        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.control_pub = self.create_publisher(Vector3, 'control', 10)

        self.target_v     = 0.0  # desired forward speed [m/s]
        self.target_omega = 0.0  # desired yaw rate      [rad/s]

        # Slew state
        self.last_omega_left  = 0.0  # [rad/s]
        self.last_omega_right = 0.0  # [rad/s]

        # Publish at a fixed rate (cmd_vel may arrive less frequently)
        self._timer = self.create_timer(0.01, self._publish)  # 100 Hz

        self.get_logger().info(
            f'Rover Velocity Controller started — '
            f'r={WHEEL_RADIUS} m, L={HALF_TRACK} m, '
            f'max={MAX_WHEEL_VEL} rad/s, slew={MAX_SLEW_RATE} rad/s²')

    # ------------------------------------------------------------------

    def cmd_vel_callback(self, msg: Twist):
        self.target_v     = msg.linear.x
        self.target_omega = msg.angular.z

    @staticmethod
    def _slew(target: float, current: float, max_delta: float) -> float:
        return current + max(min(target - current, max_delta), -max_delta)

    def _publish(self):
        # ---- kinematic feedforward ----
        omega_left  = (self.target_v - self.target_omega * HALF_TRACK) / WHEEL_RADIUS
        omega_right = (self.target_v + self.target_omega * HALF_TRACK) / WHEEL_RADIUS

        # ---- hard saturation ----
        omega_left  = max(-MAX_WHEEL_VEL, min(MAX_WHEEL_VEL, omega_left))
        omega_right = max(-MAX_WHEEL_VEL, min(MAX_WHEEL_VEL, omega_right))

        # ---- slew rate limit ----
        dt = 0.01  # matches timer period
        max_delta   = MAX_SLEW_RATE * dt
        omega_left  = self._slew(omega_left,  self.last_omega_left,  max_delta)
        omega_right = self._slew(omega_right, self.last_omega_right, max_delta)
        self.last_omega_left  = omega_left
        self.last_omega_right = omega_right

        # ---- publish ----
        # mujoco_node maps: ctrl = [msg.x, msg.x, msg.y, msg.y, 0, 0]
        #   msg.x → left wheels (front + back)
        #   msg.y → right wheels (front + back)
        msg = Vector3()
        msg.x = float(omega_left)
        msg.y = float(omega_right)
        msg.z = 0.0
        self.control_pub.publish(msg)


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
