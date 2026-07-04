import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState

from scipy.spatial.transform import Rotation as R
import numpy as np

from bayes_opt import acquisition, BayesianOptimization

import rosbag2_py

import time


width = 0.21779 * 2
radius = .05


class Optimize(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.ctrl_pub = self.create_publisher(Vector3, 'control', 10)
        
        self.mj_node_name = "mujoco_node"
        
        self.starting_pos = {'start_x': 3, 'start_y': 3, 'start_z': 0.5, 'start_yaw': 0}
        
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer.cancel()
        self.itr = 0
        
        self.param_client = self.create_client(SetParameters, f'/{self.mj_node_name}/set_parameters')

                
        self.subscription = self.create_subscription(JointState, 'wheel_joint_states', self.pose_callback, 10)
        
        #bayesian optimization stuff
        acq = acquisition.UpperConfidenceBound(kappa=2.5)
        self.optimizer = BayesianOptimization(
            f=None,
            acquisition_function=acq,
            pbounds={'wheel_friction_sliding': (0, 5), 'wheel_friction_torsional': (0, .5), 'wheel_friction_rolling': (0, .5)},
            verbose=2,
            random_state=1,
        )
        
        #ros2 bag reader stuff
        self.reader = rosbag2_py.SequentialReader()


        self.last_ctrl = None
        
        self.pose_buffer = []
        self.pose_times = []
        self.ctrl_buffer = None
        self.ctrl_time = None
        
        self.bag_time_offset = None
        self.sim_time_offset = None
        
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
       
        msg = self.ctrl_buffer
        if msg:
            self.ctrl_pub.publish(msg)
            self.ctrl_buffer = None
            return
        
        while self.reader.has_next():
            msg = self.reader.read_next()
            if self.bag_time_offset == None:
                self.bag_time_offset = msg[2]
                self.sim_time_offset = self.get_clock().now()
            if msg[0] == "/wheel_joint_states":
                self.pose_buffer.append(deserialize_message(msg[1], JointState))
                self.pose_times.append(msg[2])
                continue

        
            elif msg[0] == "/control":
                msg = deserialize_message(msg[1], Vector3)
                
                self.ctrl_pub.publish(msg)
                return
        
            
        self.timer.cancel()
        self.control_end_callback()
            
        return
        
        
    def start_opt_cycle(self):
        self.friction = self.optimizer.suggest()
        self.change_friction()
        self.reset_position()
        
        storage_options = rosbag2_py.StorageOptions(
            uri='data/test_run',
            storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        
        self.reader.open(storage_options, converter_options)
        
        time.sleep(5)
        
        self.timer.reset()
        
        self.error_total = 0
        self.error_num = 0
        
        
        return
        
    def reward_function(self): # returns negative mean squared error
        if self.error_num == 0:
            return -1
        return - (self.error_total / self.error_num) # maximize negative mean squared error -> minimize error
    
    
    # TODO this is terrible. make it not terrible
    def pose_callback(self, msg):
        # subscribe to and record pose topic
        # compute slip from wheel velocities, append to dict with timestamps?
        
        #TODO: if the rover isn't moving don't calculate error
        
        vel = msg.velocity
        pos = msg.position
        
        r = R.from_quat(pos[-4:])
        heading = r.as_euler('xyz')[0]
        
        tangential_vel = (np.sin(heading) * float(vel[4]) + np.cos(heading) * float(vel[5]))
        transverse_vel = np.cos(heading) * float(vel[4]) - np.sin(heading) * float(vel[5])
        angular_vel = float(vel[-1])
        
        # real velocities at given time
        msg = None
        
        
        msg_found = False
        for i in range(len(self.pose_times)): #TODO make this so it gets the closest thing, not the first one to happen next
            if self.pose_times[i] - self.bag_time_offset > (self.get_clock().now() - self.sim_time_offset).nanoseconds: # TODO figure out if this is right or not
                msg = self.pose_buffer[i]
                self.pose_buffer = self.pose_buffer[i + 1 :]
                self.pose_times = self.pose_times[i + 1 :]
                msg_found = True
                break
        
        if not msg_found:
            
            self.pose_buffer = []
            self.pose_times = []
            while self.reader.has_next(): 
                msg = self.reader.read_next()
                if self.bag_time_offset == None:
                    self.bag_time_offset = msg[2]
                    self.sim_time_offset = self.get_clock().now()
                
                
                if msg[0] == "/wheel_joint_states":
                    msg = deserialize_message(msg[1], JointState)
                    msg_found = True
                    
                    break

            
                elif msg[0] == "/control":
                    self.ctrl_buffer = deserialize_message(msg[1], Vector3)
                    self.ctrl_time = msg[2]
                    continue
        if not msg_found:
            self.timer.cancel()
            self.control_end_callback()
            return
        
        vel = msg.velocity
        pos = msg.position
        
        r = R.from_quat(pos[-4:])
        heading = r.as_euler('xyz')[0]
        
        real_tangential_vel = (np.sin(heading) * float(vel[4]) + np.cos(heading) * float(vel[5]))
        real_transverse_vel = np.cos(heading) * float(vel[4]) - np.sin(heading) * float(vel[5])
        real_angular_vel = float(vel[-1])
        
        # euclidian norm
        error = (tangential_vel - real_tangential_vel) ** 2 + (transverse_vel - real_transverse_vel) ** 2 + (angular_vel - real_angular_vel) ** 2
        
        self.error_num += 1
        self.error_total += error
        
        return
    
    def control_end_callback(self):
        self.pose_buffer = []
        self.pose_times = []
        self.ctrl_buffer = None
        self.ctrl_time = None
        
        self.bag_time_offset = None
        self.sim_time_offset = None
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