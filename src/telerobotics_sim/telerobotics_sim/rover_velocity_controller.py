#!/usr/bin/env python3
"""
rover_velocity_controller.py

Closed-loop P-controller for a 4-wheel tank-drive rover with force-driven
(motor) actuators in MuJoCo.

Topology
--------
Subscribed topics:
  /cmd_vel              geometry_msgs/Twist   – desired v [m/s], ω [rad/s]
  wheel_joint_states    sensor_msgs/JointState – actual wheel angular velocities
                          from MuJoCo (published by mujoco_node at sim rate)

Published topics:
  control               geometry_msgs/Vector3 – motor torque commands [N·m]
                          x = left  wheels (front + back share same torque)
                          y = right wheels (front + back share same torque)

Control law
-----------
  1. Differential-drive kinematics (feedforward):
       ω_left_ref  = (v − ω_cmd · L) / r     [rad/s]
       ω_right_ref = (v + ω_cmd · L) / r     [rad/s]

  2. Read actual wheel ω from wheel_joint_states:
       ω_left_act  = mean(wheel_f_left, wheel_b_left)
       ω_right_act = mean(wheel_f_right, wheel_b_right)

  3. P-control on angular velocity error → torque command:
       τ_left  = Kp · (ω_left_ref  − ω_left_act)
       τ_right = Kp · (ω_right_ref − ω_right_act)

  4. Slew-rate limit on τ (optional, protects drivetrain from impulses).

  5. Clamp to ±MAX_TORQUE (mirrors MuJoCo ctrlrange / forcerange).

Physical parameters
-------------------
  Wheel radius r     = 0.05 m   (cylinder geom size="0.05 0.05")
  Half-track   L     = 0.218 m  (lateral wheel offset ±0.21779 m)
  Max torque         = 4.5 N·m  (real-motor spec, mirrors MJCF ctrlrange)
  Wheel damping      = 1.89 N·m·s/rad (set on joint in MJCF; models back-EMF)

Kp tuning guide
---------------
  The feedforward term (B·ω_ref) handles steady-state torque, so Kp only
  needs to reject disturbances and correct transient errors.  A good starting
  point is Kp ≈ 0.5–2.0 × damping.
  - Too low  → slow to correct disturbances (e.g. terrain bumps).
  - Too high → noisy / oscillatory torque commands.
  Pure P will still have residual error from un-modelled ground friction;
  add a small I-gain (ki ≈0.5) if you need zero SS error on rough terrain.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time as RosTime

# ---------------------------------------------------------------------------
# Rover geometry — must match MJCF values
# ---------------------------------------------------------------------------
WHEEL_RADIUS = 0.05    # [m]
HALF_TRACK   = 0.218   # [m]  (lateral distance from chassis centre to wheel centre)

# Motor / actuator limits — must match MJCF ctrlrange / forcerange
MAX_TORQUE   = 4.5     # [N·m]

# Feedforward gain: model the steady-state damping torque so the P-term only
# has to correct residual errors.  Set to the MJCF joint damping value.
# Without FF, a pure-P controller reaches:  ω_ss = Kp·ω_ref/(Kp+B)
# i.e. with Kp=B the wheel settles at 50% of the setpoint.
WHEEL_DAMPING = 1.89   # [N·m·s/rad]  — MUST match MJCF joint damping
MAX_WHEEL_VEL = MAX_TORQUE / WHEEL_DAMPING  # ~2.38 rad/s — no-load max speed

# Slew rate on the *torque* output [N·m/s].
# At 100 Hz → 1.0 N·m per step → full torque ramp in ~4.5 steps (0.045 s).
MAX_TORQUE_SLEW = 100.0  # [N·m/s]

# Default proportional gain [N·m / (rad/s)]
# With FF active, Kp only corrects disturbances — can be lower than damping.
DEFAULT_KP = 1.89


class RoverVelocityController(Node):
    """Closed-loop P velocity controller driving force-based wheel motors.

    cmd_vel → desired wheel ω (feedforward) → torque = Kp·(ω_ref − ω_act)
    """

    def __init__(self):
        super().__init__('rover_velocity_controller')

        # ---- Tunable ROS parameters ----
        self.declare_parameter('kp',           DEFAULT_KP)
        self.declare_parameter('max_torque',   MAX_TORQUE)
        self.declare_parameter('max_slew',     MAX_TORQUE_SLEW)

        self.kp         = self.get_parameter('kp').value
        self.max_torque = self.get_parameter('max_torque').value
        self.max_slew   = self.get_parameter('max_slew').value

        # Derived kinematic limit: no-load max ω when full torque overcomes damping
        self._max_wheel_vel = self.max_torque / WHEEL_DAMPING

        # ---- Desired chassis velocity (from cmd_vel) ----
        self.cmd_v     = 0.0   # [m/s]
        self.cmd_omega = 0.0   # [rad/s]

        # ---- Actual wheel angular velocities (from MuJoCo feedback) ----
        # Order: [wheel_f_left, wheel_b_left, wheel_f_right, wheel_b_right]
        self._actual_omega = [0.0, 0.0, 0.0, 0.0]

        # ---- Slew state for torque output ----
        self._last_tau_left  = 0.0  # [N·m]
        self._last_tau_right = 0.0  # [N·m]

        # ---- ROS I/O ----
        self.control_pub   = self.create_publisher(Vector3,    'control',            10)
        # wheel_vel_setpoints: ω_ref in rad/s — plot this against wheel_joint_states
        # to validate tracking (both traces are in the same units)
        self.setpoint_pub  = self.create_publisher(JointState, 'wheel_vel_setpoints', 10)

        self.create_subscription(Twist,      'cmd_vel',           self._cmd_vel_cb,    10)
        self.create_subscription(JointState, 'wheel_joint_states', self._joint_state_cb, 10)

        # Publish at fixed rate — handles cmd_vel arriving slower than sim
        self._timer = self.create_timer(0.01, self._control_loop)  # 100 Hz

        self.get_logger().info(
            f'[RoverVelocityController] force-driven P-controller started\n'
            f'  Kp={self.kp:.3f} N·m/(rad/s)  '
            f'max_torque=±{self.max_torque} N·m  '
            f'max_slew={self.max_slew} N·m/s\n'
            f'  r={WHEEL_RADIUS} m  L={HALF_TRACK} m  '
            f'ω_max≈{MAX_WHEEL_VEL:.2f} rad/s'
        )

    # -----------------------------------------------------------------------
    # Subscribers
    # -----------------------------------------------------------------------

    def _cmd_vel_cb(self, msg: Twist):
        self.cmd_v     = msg.linear.x
        self.cmd_omega = msg.angular.z

    def _joint_state_cb(self, msg: JointState):
        """Cache actual wheel angular velocities from MuJoCo joint states."""
        name_to_idx = {name: i for i, name in enumerate(msg.name)}
        order = ['wheel_f_left', 'wheel_b_left', 'wheel_f_right', 'wheel_b_right']
        for i, name in enumerate(order):
            idx = name_to_idx.get(name)
            if idx is not None and idx < len(msg.velocity):
                self._actual_omega[i] = msg.velocity[idx]

    # -----------------------------------------------------------------------
    # Control loop
    # -----------------------------------------------------------------------

    @staticmethod
    def _slew(target: float, current: float, max_delta: float) -> float:
        return current + max(min(target - current, max_delta), -max_delta)

    def _control_loop(self):
        dt = 0.01  # matches timer period [s]

        # ----- 1. Kinematics: cmd_vel → target wheel angular velocities -----
        omega_left_ref  = (self.cmd_v - self.cmd_omega * HALF_TRACK) / WHEEL_RADIUS
        omega_right_ref = (self.cmd_v + self.cmd_omega * HALF_TRACK) / WHEEL_RADIUS

        # Clamp reference to physically reachable no-load max speed
        omega_left_ref  = max(-self._max_wheel_vel, min(self._max_wheel_vel, omega_left_ref))
        omega_right_ref = max(-self._max_wheel_vel, min(self._max_wheel_vel, omega_right_ref))

        # ----- 2. Actual wheel ω (average front+back per side) -----
        omega_left_act  = 0.5 * (self._actual_omega[0] + self._actual_omega[1])
        omega_right_act = 0.5 * (self._actual_omega[2] + self._actual_omega[3])

        # ----- 3. Feedforward + P torque command -----
        #
        # FF term:  B·ω_ref  — pre-supplies the steady-state damping torque, so
        #           the wheel can reach ω_ref even when P-error → 0.
        # P  term:  Kp·e    — corrects transient errors and terrain disturbances.
        #
        # Without FF, pure-P settles at ω_ss = Kp·ω_ref/(Kp+B) < ω_ref.
        e_left  = omega_left_ref  - omega_left_act
        e_right = omega_right_ref - omega_right_act

        tau_left  = WHEEL_DAMPING * omega_left_ref  + self.kp * e_left
        tau_right = WHEEL_DAMPING * omega_right_ref + self.kp * e_right

        # ----- 4. Slew-rate limit on torque -----
        max_delta = self.max_slew * dt
        tau_left  = self._slew(tau_left,  self._last_tau_left,  max_delta)
        tau_right = self._slew(tau_right, self._last_tau_right, max_delta)
        self._last_tau_left  = tau_left
        self._last_tau_right = tau_right

        # ----- 5. Hard clamp to motor torque limit -----
        tau_left  = max(-self.max_torque, min(self.max_torque, tau_left))
        tau_right = max(-self.max_torque, min(self.max_torque, tau_right))

        # ----- 6. Publish torque commands -----
        ctrl_msg = Vector3()
        ctrl_msg.x = float(tau_left)
        ctrl_msg.y = float(tau_right)
        ctrl_msg.z = 0.0
        self.control_pub.publish(ctrl_msg)

        # ----- 7. Publish ω_ref setpoints (rad/s) for PlotJuggler comparison -----
        # Plot wheel_vel_setpoints/*/velocity vs wheel_joint_states/*/velocity
        # — both are in rad/s so the traces are directly comparable.
        now = self.get_clock().now().to_msg()
        sp = JointState()
        sp.header.stamp = now
        sp.name     = ['wheel_f_left', 'wheel_b_left', 'wheel_f_right', 'wheel_b_right']
        sp.velocity = [
            float(omega_left_ref),   # front-left  setpoint
            float(omega_left_ref),   # back-left   setpoint (same side → same target)
            float(omega_right_ref),  # front-right setpoint
            float(omega_right_ref),  # back-right  setpoint
        ]
        self.setpoint_pub.publish(sp)

        self.get_logger().debug(
            f'ref=({omega_left_ref:+.3f}, {omega_right_ref:+.3f}) rad/s  '
            f'act=({omega_left_act:+.3f}, {omega_right_act:+.3f}) rad/s  '
            f'err=({e_left:+.3f}, {e_right:+.3f})  '
            f'τ=({tau_left:+.3f}, {tau_right:+.3f}) N·m'
        )


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
