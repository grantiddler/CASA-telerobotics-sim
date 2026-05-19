"""
rover_teleop.launch.py

Launches the MuJoCo simulator node and the rover velocity controller together.

Nodes
-----
  mujoco                 – telerobotics_sim.mujoco_node       (sim + viewer)
  rover_velocity_controller – telerobotics_sim.rover_velocity_controller

Topic wiring (automatic via shared namespace)
---------------------------------------------
  /cmd_vel              → rover_velocity_controller  (Twist input)
  /control              ← rover_velocity_controller → mujoco_node  (torque cmd)
  /wheel_joint_states   ← mujoco_node → rover_velocity_controller  (actual ω)
  /wheel_vel_setpoints  ← rover_velocity_controller  (ω_ref, for PlotJuggler)
  /wheel_torque_cmds    ← mujoco_node                (τ applied, for PlotJuggler)
  /pose                 ← mujoco_node                (chassis pose)

Controller parameters (can be overridden on the command line via --ros-args -p)
-----------------
  kp          [N·m/(rad/s)]  Proportional gain         default: 1.89
  max_torque  [N·m]          Motor torque clamp         default: 4.5
  max_slew    [N·m/s]        Torque slew rate limit     default: 100.0
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    LD = []
    for i in range(1):
        LD.append(Node(
            package='telerobotics_sim',
            executable='mujoco',
            name=f'mujoco_node_{i}',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'sim number':         i,
                'friction':           [2.0, 0.050, 0.015]
            }],
        ))

    
    return LaunchDescription(LD)
