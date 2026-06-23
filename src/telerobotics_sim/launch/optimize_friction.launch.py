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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    enable_viewer = LaunchConfiguration('enable_viewer')
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    start_z = LaunchConfiguration('start_z')
    start_yaw = LaunchConfiguration('start_yaw')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_viewer',
            default_value='false',
            description='Launch MuJoCo viewer GUI'
        ),
        DeclareLaunchArgument('start_x', default_value='3.0'),
        DeclareLaunchArgument('start_y', default_value='3.0'),
        DeclareLaunchArgument('start_z', default_value='1.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.0'),

        # ------------------------------------------------------------------
        # MuJoCo simulation node
        # Runs the physics engine, renders the viewer, and publishes:
        #   /pose, /wheel_joint_states, /wheel_torque_cmds,
        #   /wheel_*_pose topics
        # ------------------------------------------------------------------
        Node(
            package='telerobotics_sim',
            executable='mujoco',
            name='mujoco_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'enable_viewer': enable_viewer,
                'start_x': start_x,
                'start_y': start_y,
                'start_z': start_z,
                'start_yaw': start_yaw,
            }],
        ),

        ## tcp connector for unity
        Node(
            package="ros_tcp_endpoint",
            executable="default_server_endpoint",
            output="screen"
        ),

        # ------------------------------------------------------------------
        # Rover velocity controller (feedforward + P)
        # Subscribes: /cmd_vel, /wheel_joint_states
        # Publishes:  /control (torque), /wheel_vel_setpoints
        # ------------------------------------------------------------------
        Node(
            package='telerobotics_sim',
            executable='rover_velocity_controller',
            name='rover_velocity_controller',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'kp':         1.89,   # [N·m/(rad/s)] — tweak to tune transient response
                'max_torque': 4.5,    # [N·m]         — must match MJCF ctrlrange/forcerange
                'max_slew':   100.0,  # [N·m/s]       — reduce to soften torque steps
            }],
        ),
    ])