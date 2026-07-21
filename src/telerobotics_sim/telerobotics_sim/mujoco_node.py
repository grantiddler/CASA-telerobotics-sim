import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState

import mujoco
import mujoco.viewer

from ament_index_python.packages import get_package_share_directory
import os
import math

from rcl_interfaces.msg import SetParametersResult



# 0.001 and rk4 integrator
# add sensor to joint?? use sensor array instead of qvel


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.publisher_ = self.create_publisher(
            Pose, 
            'pose', 
            10)
            
        self.wheel_fl_pub = self.create_publisher(Pose, 'wheel_f_left_pose', 10)
        self.wheel_bl_pub = self.create_publisher(Pose, 'wheel_b_left_pose', 10)
        self.wheel_fr_pub = self.create_publisher(Pose, 'wheel_f_right_pose', 10)
        self.wheel_br_pub = self.create_publisher(Pose, 'wheel_b_right_pose', 10)

        # Wheel velocity topics for PlotJuggler (actual vs commanded)
        self.wheel_actual_vel_pub  = self.create_publisher(JointState, 'wheel_joint_states',  10)
        # wheel_torque_cmds: the torque [N·m] last written to each motor actuator
        self.wheel_torque_cmd_pub  = self.create_publisher(JointState, 'wheel_torque_cmds',   10)

        # Last commanded wheel velocities [fl, bl, fr, br] in rad/s
        self._cmd_wheel_vels = [0.0, 0.0, 0.0, 0.0]
        
        self.subscription = self.create_subscription(
            Vector3,
            'control',
            self.set_control_callback,
            10)
        
        pkg_share = get_package_share_directory('telerobotics_sim')
        model_path = os.path.join(pkg_share, 'models', 'scene.xml')

        self.declare_parameter('start_x', 3.0)
        self.declare_parameter('start_y', 3.0)
        self.declare_parameter('start_z', 1.0)
        self.declare_parameter('start_yaw', 0.0)  # degrees
        self.declare_parameter('wheel_friction_sliding', 2.0)
        self.declare_parameter('wheel_friction_torsional', 0.050)
        self.declare_parameter('wheel_friction_rolling', 0.015)

        self.m = mujoco.MjModel.from_xml_path(model_path)
        self.d = mujoco.MjData(self.m)

        self.wheel_geom_ids = [
            self.m.geom(name).id for name in
            ['wheel-f-left-geom', 'wheel-b-left-geom', 'wheel-f-right-geom', 'wheel-b-right-geom']
        ]

        self.update_wheel_friction()  # apply initial values
        self.add_on_set_parameters_callback(self.on_param_change)

        # Apply initial pose to the chassis free joint
        x = self.get_parameter('start_x').value
        y = self.get_parameter('start_y').value
        z = self.get_parameter('start_z').value
        yaw = math.radians(self.get_parameter('start_yaw').value)

        adr = self.m.joint('chassis_free').qposadr[0]  # don't hardcode index 0
        self.d.qpos[adr:adr+3] = [x, y, z]
        self.d.qpos[adr+3:adr+7] = [math.cos(yaw/2), 0, 0, math.sin(yaw/2)]
        mujoco.mj_forward(self.m, self.d)  # propagate before first mj_step

        self.declare_parameter('enable_viewer', False)

        self.enable_viewer = self.get_parameter(
            'enable_viewer'
        ).get_parameter_value().bool_value

        self.viewer = None

        if self.enable_viewer:
            self.get_logger().info("Launching MuJoCo viewer")
            self.viewer = mujoco.viewer.launch_passive(self.m, self.d)
        else:
            self.get_logger().info("Running headless (no viewer)")

        timer_period = self.m.opt.timestep
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        print(self.m.hfield_nrow)
        print(self.m.hfield_ncol)
        print(self.m.hfield_size)

    def update_wheel_friction(self):
        sliding = self.get_parameter('wheel_friction_sliding').value
        torsional = self.get_parameter('wheel_friction_torsional').value
        rolling = self.get_parameter('wheel_friction_rolling').value
        for gid in self.wheel_geom_ids:
            self.m.geom_friction[gid] = [sliding, torsional, rolling]

    def on_param_change(self, params):
        for p in params:
            if p.name.startswith('wheel_friction'):
                self.update_wheel_friction()
                self.get_logger().info(f'Updated {p.name} -> {p.value}')
        return SetParametersResult(successful=True)
        

    def set_control_callback(self, msg):
        # msg.x = left torque [N·m], msg.y = right torque [N·m]
        # Motor actuators: ctrl is directly applied as joint torque.
        # ctrlrange/forcerange clamps are enforced by MuJoCo.
        left_torque  = msg.x
        right_torque = msg.y
        self.d.ctrl = [left_torque, left_torque, right_torque, right_torque]
        # Store for diagnostics / PlotJuggler
        self._cmd_wheel_vels = [left_torque, left_torque, right_torque, right_torque]

    
    def timer_callback(self):
        msg = Pose()
        msg.position.x = self.d.body("chassis").xpos[0]
        msg.position.y = self.d.body("chassis").xpos[1]
        msg.position.z = self.d.body("chassis").xpos[2]

        msg.orientation.w = self.d.body("chassis").xquat[0]
        msg.orientation.x = self.d.body("chassis").xquat[1]
        msg.orientation.y = self.d.body("chassis").xquat[2]
        msg.orientation.z = self.d.body("chassis").xquat[3]
        
        
        mujoco.mj_step(self.m, self.d)

        if self.viewer is not None:
            self.viewer.sync()
        self.publisher_.publish(msg)
        
        # Publish wheel poses
        wheel_names = ["wheel-f-left", "wheel-b-left", "wheel-f-right", "wheel-b-right"]
        wheel_pubs = [self.wheel_fl_pub, self.wheel_bl_pub, self.wheel_fr_pub, self.wheel_br_pub]
        
        for name, pub in zip(wheel_names, wheel_pubs):
            w_msg = Pose()
            w_msg.position.x = self.d.body(name).xpos[0]
            w_msg.position.y = self.d.body(name).xpos[1]
            w_msg.position.z = self.d.body(name).xpos[2]
            w_msg.orientation.w = self.d.body(name).xquat[0]
            w_msg.orientation.x = self.d.body(name).xquat[1]
            w_msg.orientation.y = self.d.body(name).xquat[2]
            w_msg.orientation.z = self.d.body(name).xquat[3]
            pub.publish(w_msg)

        # ---- Publish actual wheel angular velocities (from MuJoCo qvel) ----
        WHEEL_JOINTS = [
            'wheel-f-left-hinge',
            'wheel-b-left-hinge',
            'wheel-f-right-hinge',
            'wheel-b-right-hinge',
        ]
        WHEEL_NAMES = ['wheel_f_left', 'wheel_b_left', 'wheel_f_right', 'wheel_b_right']
        now = self.get_clock().now().to_msg()

        actual_js = JointState()
        actual_js.header.stamp = now
        actual_js.name     = WHEEL_NAMES
        actual_js.velocity = [
            float(self.d.joint(j).qvel[0]) for j in WHEEL_JOINTS
        ]
        self.wheel_actual_vel_pub.publish(actual_js)

        # ---- Publish commanded motor torques [N·m] ----
        cmd_js = JointState()
        cmd_js.header.stamp = now
        cmd_js.name     = WHEEL_NAMES
        cmd_js.effort   = [float(v) for v in self._cmd_wheel_vels]  # effort = torque
        self.wheel_torque_cmd_pub.publish(cmd_js)

        self.i += 1


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
