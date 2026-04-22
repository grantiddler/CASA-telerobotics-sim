#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Vector3
import math
import time

class RoverVelocityController(Node):
    def __init__(self):
        super().__init__('rover_velocity_controller')
        
        # Subscriptions
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.pose_sub = self.create_subscription(
            Pose,
            'pose',
            self.pose_callback,
            10
        )
        
        # Publisher for the rover's motors
        self.control_pub = self.create_publisher(
            Vector3,
            'control',
            10
        )
        
        # State variables
        self.target_v = 0.0
        self.target_omega = 0.0
        
        self.last_pose = None
        self.last_time = None
        self.last_yaw = 0.0
        
        # P-controller gains
        self.kp_linear = 5.0
        self.kp_angular = 2.0
        
        # We publish the control at a fixed rate, doing the P-loop in a timer,
        # or we can do it directly in the pose_callback. Since pose_callback is at 100hz,
        # that's a perfect place for a control loop.
        self.get_logger().info("Rover Velocity Controller Node has been started.")

    def cmd_vel_callback(self, msg):
        self.target_v = msg.linear.x
        self.target_omega = msg.angular.z

    def euler_from_quaternion(self, x, y, z, w):
        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def pose_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Extract current yaw
        current_yaw = self.euler_from_quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        
        if self.last_pose is None or self.last_time is None:
            self.last_pose = msg
            self.last_time = current_time
            self.last_yaw = current_yaw
            return
            
        dt = current_time - self.last_time
        if dt <= 0:
            return
            
        # Calculate global velocities
        dx = msg.position.x - self.last_pose.position.x
        dy = msg.position.y - self.last_pose.position.y
        
        # In the mujoco model: X is right, Y is forward.
        # Rotating velocity from world frame to body frame:
        # A positive yaw is a CCW rotation (Left is positive).
        # v_forward is the velocity along the vehicle's local Y axis.
        v_forward = (dy * math.cos(current_yaw) - dx * math.sin(current_yaw)) / dt
        
        # Angular velocity
        dyaw = math.atan2(math.sin(current_yaw - self.last_yaw), math.cos(current_yaw - self.last_yaw))
        omega = dyaw / dt
        
        # Calculate errors
        error_v = self.target_v - v_forward
        error_omega = self.target_omega - omega
        
        # P-control for linear and angular commands
        u_v = self.kp_linear * error_v
        u_omega = self.kp_angular * error_omega
        
        # Tank drive differential mixing
        # Positive omega means turning left -> right wheels move faster, left wheels move slower.
        u_left = u_v - u_omega
        u_right = u_v + u_omega
        
        # Clamp values to motor ctrlrange (-4.5 to 4.5 in mujoco_node.py)
        max_torque = 4.5
        u_left = max(-max_torque, min(max_torque, u_left))
        u_right = max(-max_torque, min(max_torque, u_right))
        
        # Publish control vector
        control_msg = Vector3()
        # mojoco_node sets: self.d.ctrl = [msg.x, msg.x, msg.y, msg.y, 0, 0]
        # x corresponds to left wheels and y corresponds to right wheels
        control_msg.x = float(u_left)
        control_msg.y = float(u_right)
        control_msg.z = 0.0
        
        self.control_pub.publish(control_msg)
        
        # Move state forward
        self.last_pose = msg
        self.last_time = current_time
        self.last_yaw = current_yaw

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
