import rclpy
from rclpy.node import Node

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState

from scipy.spatial.transform import Rotation as R
import numpy as np

from bayes_opt import acquisition, BayesianOptimization

import time


width = 0.21779 * 2
radius = .05


class Optimize(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.ctrl_pub = self.create_publisher(Vector3, 'control', 10)
        
        self.mj_node_name = "mujoco"
        
        self.starting_pos = {'start_x': 3, 'start_y': 3, 'start_z': 1, 'start_yaw': 0}
        
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer.cancel()
        self.itr = 0
        
        self.param_client = self.create_client(SetParameters, f'/{self.mj_node_name}/set_parameters')

                
        self.subscription = self.create_subscription(JointState, 'wheel_joint_states', self.pose_callback, 10)
        
        acq = acquisition.UpperConfidenceBound(kappa=2.5)
        self.optimizer = BayesianOptimization(
            f=None,
            acquisition_function=acq,
            pbounds={'wheel_friction_sliding': (0, 10), 'wheel_friction_torsional': (0, 10), 'wheel_friction_rolling': (0, 10)},
            verbose=2,
            random_state=1,
        )
        
        self.start_opt_cycle()

    def change_friction(self):
        req = SetParameters.Request()
        for i in ['wheel_friction_sliding', 'wheel_friction_torsional', 'wheel_friction_rolling']:
            param = Parameter()
            param.name = i
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = float(self.friction[i])
            req.parameters.append(param)

        self.future = self.param_client.call_async(req)
        return self.future.result()
    
    def reset_position(self):
        req = SetParameters.Request()
        for i in ['start_x', 'start_y', 'start_z', 'start_yaw']:
            param = Parameter()
            param.name = i
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = float(self.starting_pos[i])
            req.parameters.append(param)

        self.future = self.param_client.call_async(req)
        return self.future.result()
        
    def timer_callback(self):
        self.itr += 1
        self.get_logger().info(f'timer: {self.itr}')
    
        # publish control values in here?
        
        msg = Vector3()
        msg.x = float(4.5)
        msg.y = float(0)
        self.ctrl_pub.publish(msg)     
        
        
        if(self.itr == 10**2):
            self.itr = 0
            self.timer.cancel()
            msg.x = float(0)
            msg.y = float(0)
            self.ctrl_pub.publish(msg)     
            
            
            self.control_end_callback()
        
            

            
        return
        
        
    def start_opt_cycle(self):
        self.get_logger().info(f'start cycle')
        
        self.friction = self.optimizer.suggest()
        self.get_logger().info(str(self.friction))
        self.change_friction()
        self.reset_position()
        
        time.sleep(5)
        
        self.timer.reset()
        
        self.error_total = 0
        self.error_num = 0
        
        
        return
        
    def reward_function(self): # returns negative mean squared error
        if self.error_num == 0:
            return None
        self.get_logger().info(f"Mean error squared: {self.error_total / self.error_num}")
        return - (self.error_total / self.error_num) # maximize negative mean squared error -> minimize error
    
    
    # TODO this is terrible. make it not terrible
    def pose_callback(self, msg):
        # subscribe to and record pose topic
        # compute slip from wheel velocities, append to dict with timestamps?
        
        
        
        vel = msg.velocity
        pos = msg.position
        
        r = R.from_quat(pos[-4:])
        heading = r.as_euler('xyz')[0]
        
        
        tangential_vel = (np.sin(heading) * float(vel[4]) + np.cos(heading) * float(vel[5]))
        transverse_vel = np.cos(heading) * float(vel[4]) - np.sin(heading) * float(vel[5])
        angular_vel = float(vel[-1])
        
        # compute "ideal" velocity from the slip thing I did before
        real_tangential_vel = radius * (float(vel[0]) + float(vel[1]) + float(vel[2]) + float(vel[3])) / 4
        real_transverse_vel = 0
        real_angular_vel = radius * (float(vel[0]) + float(vel[1]) - float(vel[2]) - float(vel[3])) / (2 * width)
        
        # euclidian norm
        error = (tangential_vel - real_tangential_vel) ** 2 + (transverse_vel - real_transverse_vel) ** 2 + (angular_vel - real_angular_vel) ** 2
        
        self.error_num += 1
        self.error_total += error
        

        
        
        
        return
    
    def control_end_callback(self):
        self.optimizer.register(
            params=self.friction,
            target=self.reward_function(),
        )

        self.start_opt_cycle()
        return
        


def main(args=None):
    rclpy.init(args=args)

    optimize = Optimize()

    rclpy.spin(optimize)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    optimize.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()