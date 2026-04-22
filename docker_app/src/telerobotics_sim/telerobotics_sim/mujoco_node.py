import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState


import mujoco
import mujoco.viewer


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

        self.m = mujoco.MjModel.from_xml_string("""<mujoco model="rover">
<compiler angle="degree"/>
<option timestep="0.01" gravity="0 0 -1.81"/>
<asset>

</asset>
<worldbody>
<!--  Ground plane  -->
<!--  <geom name="terrain_collision" type="hfield" hfield="pit" pos="20 15 0" rgba="0.5 0.35 0.2 1" contype="1" conaffinity="1" group="0"/>  -->
<geom name="surface_terrain_geom" type="box" size="50 50 0.05" pos="0 0 0"/>
<!--  Chassis  -->
<body name="chassis" pos="3 3 1">
<inertial pos="0 -0.073 -0.1090" mass="8" diaginertia="1 1 1"/>
<joint name="chassis_free" type="free"/>
<geom type="box" size="0.1 0.15 0.1" pos="0 -0.05 0" rgba="0 0 1 1" mass="0.05"/>
<!--  Left rocker  -->
<body name="rocker-left" pos="0 0 0">
<!--  Add hinge for suspension  -->
<joint name="rocker-left-hinge" type="hinge" range="-35 35" frictionloss="0.135" axis="1 0 0" pos="-0.18 -0.073 -0.109"/>
<geom type="box" size="0.01 0.15 0.01" pos="-.11 -.05 -.125" rgba="0 0 1 1" mass="0.2"/>
<!--  Front Left Wheel  -->
<body name="wheel-f-left" pos="-0.21779 0.0694 -0.16307" euler="0 90 0">
<joint name="wheel-f-left-hinge" damping="1.89" type="hinge" axis="0 0 1"/>
<!--  sliding friction | rotational friction | rolling friction -->
<geom type="cylinder" size="0.05 0.05" rgba="0.1 0.1 0.1 1" density="100" condim="6" friction="2.0 0.050 0.015"/>
<inertial pos="0 0 0" mass="0.24" diaginertia="0.001 0.001 0.001"/>
</body>
<!--  Back Left Wheel  -->
<body name="wheel-b-left" pos="-0.21779 -0.21384 -0.16307" euler="0 90 0">
<joint name="wheel-b-left-hinge" type="hinge" damping="1.89" axis="0 0 1"/>
<geom type="cylinder" size="0.05 0.05" rgba="0.1 0.1 0.1 1" density="100" condim="6" friction="2.0 0.050 0.015"/>
<inertial pos="0 0 0" mass="0.24" diaginertia="0.001 0.001 0.001"/>
</body>
</body>
<!--  Right rocker  -->
<body name="rocker-right" pos="0 0 0">
<!--  Add hinge for suspension  -->
<joint name="rocker-right-hinge" type="hinge" range="-35 35" frictionloss="0.135" axis="1 0 0" pos="0.18 -0.073 -0.109"/>
<geom type="box" size="0.01 0.15 0.01" pos=".11 -.05 -.125" rgba="0 0 1 1" mass="0.2"/>
<!--  Front Right Wheel  -->
<body name="wheel-f-right" pos="0.21779 0.0694 -0.16307" euler="0 90 0">
<joint name="wheel-f-right-hinge" type="hinge" damping="1.89" axis="0 0 1"/>
<geom type="cylinder" size="0.05 0.05" rgba="0.1 0.1 0.1 1" density="100" condim="6" friction="2.0 0.050 0.015"/>
<inertial pos="0 0 0" mass="0.24" diaginertia="0.001 0.001 0.001"/>
</body>
<!--  Back Right Wheel  -->
<body name="wheel-b-right" pos="0.21779 -0.21384 -0.16307" euler="0 90 0">
<joint name="wheel-b-right-hinge" type="hinge" damping="1.89" axis="0 0 1"/>
<geom type="cylinder" size="0.05 0.05" rgba="0.1 0.1 0.1 1" density="100" condim="6" friction="2.0 0.050 0.015"/>
<inertial pos="0 0 0" mass="0.24" diaginertia="0.001 0.001 0.001"/>
</body>
</body>
</body>
</worldbody>
<actuator>
<!--  Wheel drive motors: force-driven, max 4.5 N·m (matches real motor spec)  -->
<!--  Joint damping=1.89 N·m·s/rad models viscous friction / back-EMF            -->
<motor name="wheel-f-left-motor"  joint="wheel-f-left-hinge"  ctrlrange="-4.5 4.5" forcerange="-4.5 4.5"/>
<motor name="wheel-b-left-motor"  joint="wheel-b-left-hinge"  ctrlrange="-4.5 4.5" forcerange="-4.5 4.5"/>
<motor name="wheel-f-right-motor" joint="wheel-f-right-hinge" ctrlrange="-4.5 4.5" forcerange="-4.5 4.5"/>
<motor name="wheel-b-right-motor" joint="wheel-b-right-hinge" ctrlrange="-4.5 4.5" forcerange="-4.5 4.5"/>
</actuator>
<equality>
<!--  Differential constraint: rocker-left + rocker-right = 0  -->
<joint joint1="rocker-left-hinge" joint2="rocker-right-hinge" polycoef="0 -1 0 0 0"/>
</equality>
<contact>
<!--  Disable self-collisions  -->
<exclude body1="wheel-f-right" body2="rocker-right"/>
<exclude body1="wheel-b-right" body2="rocker-right"/>
<exclude body1="wheel-f-left" body2="rocker-left"/>
<exclude body1="wheel-b-left" body2="rocker-left"/>
<exclude body1="wheel-f-right" body2="chassis"/>
<exclude body1="wheel-b-right" body2="chassis"/>
<exclude body1="wheel-f-left" body2="chassis"/>
<exclude body1="wheel-b-left" body2="chassis"/>
<exclude body1="rocker-right" body2="chassis"/>
<exclude body1="rocker-left" body2="chassis"/>
</contact>
</mujoco>""")
        self.d = mujoco.MjData(self.m)
        self.viewer = mujoco.viewer.launch_passive(self.m, self.d)

        timer_period = self.m.opt.timestep
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

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
