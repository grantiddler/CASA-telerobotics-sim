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
import numpy as np

base_friction = [2.0, 0.050, 0.015]
friction_step_size = 0.05
friction_steps = 10
duplicates = 1

def get_friction(var, sweep):
    friction = base_friction.copy()
    friction[var] = base_friction[var] * (1 + friction_step_size * (sweep - friction_steps))
    
    return friction


def generate_launch_description():
    LD = []

    for i in range(3):
        
        friction = base_friction.copy()
        for j in range(2 * friction_steps + 1):
            
            friction = get_friction(i, j)
            
            for k in range(duplicates):
                num = 1000 * i + j*10+k

                LD.append(Node(
                package='telerobotics_sim',
                executable='mujoco',
                name=f'mujoco_node_{num}',
                parameters=[{
                    'sim_ID':         num,
                    'friction':       friction
                }],
                ))
                
    # LD.append(Node(
    #             package='telerobotics_sim',
    #             executable='control',
    #             name=f'control_publisher',
    #             output='screen',
    #             emulate_tty=True,

    #             ))

    
    return LaunchDescription(LD)
