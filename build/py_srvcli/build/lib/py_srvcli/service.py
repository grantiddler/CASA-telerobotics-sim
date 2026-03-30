from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import mujoco
import mujoco.viewer


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.set_control_callback)
        self.publisher_ = self.create_publisher(String, 'pose', 10)
        
        self.subscription = self.create_subscription(
            String,
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
<geom name="surface_terrain_geom" type="box" size="5 5 0.05" pos="0 0 0"/>
<!--  Chassis  -->
<body name="chassis" pos="3 3 1">
<geom type="box" size="0.05 0.05 0.05" pos="0 0 0" rgba="0.7 0.7 0.7 1"/>
<inertial pos="0 -0.073 -0.1090" mass="8" diaginertia="1 1 1"/>
<joint name="chassis_free" type="free"/>
<geom type="box" size="0.05 0.05 0.05" rgba="0 0 1 1" mass="0.05"/>
<geom type="box" size="0.05 0.05 0.05" rgba="0 0 1 1" mass="0.05"/>
<!--  Left rocker  -->
<body name="rocker-left" pos="1 0 0">
<!--  Add hinge for suspension  -->
<joint name="rocker-left-hinge" type="hinge" range="-45 45" frictionloss="0.135" axis="1 0 0" pos="-0.18 -0.073 -0.109"/>
<geom type="box" size="0.05 0.05 0.05" rgba="0 0 1 1" mass="0.2"/>
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
<body name="rocker-right" pos="-1 0 0">
<!--  Add hinge for suspension  -->
<joint name="rocker-right-hinge" type="hinge" range="-45 45" frictionloss="0.0" axis="1 0 0" pos="0.18 -0.073 -0.109"/>
<geom type="box" size="0.05 0.05 0.05" rgba="0 0 1 1" mass="0.2"/>
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
<!--  Left wheel motors  -->
<motor name="wheel-f-left-motor" joint="wheel-f-left-hinge" gear="1" ctrllimited="true" ctrlrange="-4.5 4.5"/>
<motor name="wheel-b-left-motor" joint="wheel-b-left-hinge" gear="1" ctrllimited="true" ctrlrange="-4.5 4.5"/>
<!--  Right wheel motors  -->
<motor name="wheel-f-right-motor" joint="wheel-f-right-hinge" gear="1" ctrllimited="true" ctrlrange="-4.5 4.5"/>
<motor name="wheel-b-right-motor" joint="wheel-b-right-hinge" gear="1" ctrllimited="true" ctrlrange="-4.5 4.5"/>
<motor name="rocker-left-test" joint="rocker-left-hinge" gear="1" ctrlrange="-10 10"/>
<motor name="rocker-right-test" joint="rocker-right-hinge" gear="1" ctrlrange="-10 10"/>
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
        # response.sum = request.a + request.b
        self.get_logger().info(str(self.d.ctrl))
        self.get_logger().info(str(msg.data))
        # self.d.ctrl = [request.a, request.a, request.b, request.b, 0, 0]

        # return response

    
    def timer_callback(self):
        msg = String()
        msg.data = str(self.d.body("chassis").xpos)
        
        mujoco.mj_step(self.m, self.d)

        self.viewer.sync()

        self.publisher_.publish(msg)
        # self.get_logger().info(str(self.d.body("chassis").xpos))
        self.i += 1


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
