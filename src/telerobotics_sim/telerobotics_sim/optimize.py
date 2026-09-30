import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType

from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
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
        
        self.last_ctrl = None
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer.cancel()
        self.itr = 0
        
        self.param_client = self.create_client(SetParameters, f'/{self.mj_node_name}/set_parameters')

                
        self.subscription = self.create_subscription(JointState, 'wheel_joint_states', self.pose_callback, 10)
        
        self.error_pub = self.create_publisher(Float64, 'error', 10)
        self.error_avg_pub = self.create_publisher(Float64, 'error_avg', 10)
        
        self.sim_avg = self.create_publisher(Vector3, 'sim_avg', 10)
        self.real_avg = self.create_publisher(Vector3, 'real_avg', 10)
        
        
        
        #bayesian optimization stuff
        acq = acquisition.UpperConfidenceBound(kappa=2.5)
        self.optimizer = BayesianOptimization(
            f=None,
            acquisition_function=acq,
            pbounds={'wheel_friction_sliding': (0, 3), 'wheel_friction_torsional': (0, .1), 'wheel_friction_rolling': (0, .05)},
            verbose=2,
            random_state=1,
        )
        
        #ros2 bag reader stuff
        self.reader = rosbag2_py.SequentialReader()


        self.ctl_iterations = 100

        
        self.pose_buffer = []
        self.pose_times = []
        self.ctrl_buffer = None
        self.ctrl_time = None
        
        self.bag_time_offset = None
        self.sim_time_offset = None
        
        self.ctls = [[4.5,4.5], [3.375, 4.5], [1.125, 4.5], [2.25, 4.5], [3.375, 4.5]]
        self.tan_target = [0.06662833536826238, 0.06494687030304573, 0.02827034501284375, 0.005250257609663304, 0.004006641722145742]
        self.tran_target = [0.0010356243687817087, -0.00275681761491831, -0.003036964496852522, -0.0028726530164218935, -0.0015745129517468937]
        self.ang_target = [-0.0022234652848633326,  -0.0040671347580258635, -0.009502468912110972, -0.01757577157573499, 0.015356837980413042]
        
        # "input_filename", "average", "neg_average", "average_tran", "neg_average_tran", "average_rot", "neg_average_rot", "cmd_neg_l", "cmd_neg_r", "cmd_pos_l", "cmd_pos_r"
        # "test1.txt", 0.06662833536826238, -0.000729351557597434, 0.0010356243687817087, -3.7155207739479975e-05, -0.0022234652848633326, 0.01198887318094532, 0, 0, 1.0, 1.0
        # "test2.txt", 0.06494687030304573, -0.0660283037780729, -0.00275681761491831, -0.001048086809973268, -0.0040671347580258635, -0.03606246850093788, -0.75, -1.0, 0.75, 1.0
        # "test4.txt", 0.02827034501284375, -0.01971268069716876, -0.003036964496852522, 0.0021353250146064363, -0.009502468912110972, 0.009715795986086365, -0.25, -1.0, 0.25, 1.0
        # "test7.txt", 0.005250257609663304, -0.008890301258607743, -0.0028726530164218935, 0.0029511287150970865, -0.01757577157573499, 0.016827316043262236, -0.5, -1.0, 0.5, 1.0
        # "test8.txt", 0.004006641722145742, -0.0020498900895139136, -0.0015745129517468937, 0.0047734360132171, 0.015356837980413042, -0.058630637606532844, -0.75, -1.0, 0.75, 1.0
        # # "test9.txt", 0.00098546081595037, 0.0016379128717829664, 0.0009813014442907639, 0.000756858128900289, -0.043626041489224214, 0.030148537277467156, -1.0, -1.0, 1.0, 1.0

        
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

        self.control_times = []
        self.control_vals = []
        
        self.vel_times = []
        self.vels = []
        
        self.wheel_vel_times = []
        self.wheel_vels = []
        
        self.future = self.param_client.call_async(req)
        return self.future.result()
        
    def timer_callback(self):
        
        if self.ctrl_num == len(self.ctls):
            self.get_logger().info(f"{(self.control_vals)}")
            
            self.timer.cancel()
            self.control_end_callback()
       
        elif self.itr < self.ctl_iterations:
            self.itr += 1
            msg = Vector3()
            msg.x = self.ctls[self.ctrl_num][0]
            msg.y = self.ctls[self.ctrl_num][1]
            self.ctrl_pub.publish(msg)

        
        elif self.itr == self.ctl_iterations:
            self.error_num += 1
            error = np.power((self.tan_av / (self.av_n + 0.001)) - self.tan_target[self.ctrl_num], 2) + np.power((self.tran_av / (self.av_n + 0.001)) - self.tran_target[self.ctrl_num], 2) + np.power((self.ang_av / (self.av_n + 0.001)) - self.ang_target[self.ctrl_num], 2)
            self.get_logger().info(f"{(self.tan_av / (self.av_n + 0.001))} + {(self.tran_av / (self.av_n + 0.001))} + {(self.ang_av / (self.av_n + 0.001))}")
            self.error_total += error
            
            self.itr = 0
            self.ctrl_num += 1
            
            self.tan_av = 0
            self.tran_av = 0
            self.ang_av = 0
            self.av_n = 0
            self.err = 0
            
            msg = Float64()
            msg.data = error
            
            self.error_pub.publish(msg)
            
     
        
        
    def start_opt_cycle(self):
        self.friction = self.optimizer.suggest()
        self.change_friction()
        self.reset_position()
        
        # storage_options = rosbag2_py.StorageOptions(
        #     uri='data/multi_input4',
        #     storage_id='sqlite3')
        # converter_options = rosbag2_py.ConverterOptions('', '')
        
        # self.reader.open(storage_options, converter_options)
        
        time.sleep(5)
        
        self.timer.reset()
        
        self.error_total = 0
        self.error_num = 0
        
        self.tangential_last = 0
        self.transverse_last = 0
        self.angular_last = 0
        self.real_tangential_last = 0
        self.real_transverse_last = 0
        self.real_angular_last = 0
        
        self.tan_av = 0
        self.tran_av = 0
        self.ang_av = 0
        self.av_n = 0
        self.err = 0
        
        self.ctrl_num = 0
        
        self.itr = 0
        
        
        return
        
    def reward_function(self): # returns negative mean squared error
        
        if self.error_num == 0:
            return -1.0
        return - (self.error_total / self.error_num) # maximize negative mean squared error -> minimize error
    
    
    # TODO this is terrible. make it not terrible
    def pose_callback(self, msg):
        # subscribe to and record pose topic
        # compute slip from wheel velocities, append to dict with timestamps?
        
        #TODO: if the rover isn't moving don't calculate error
        
        vel = msg.velocity
        pos = msg.position
        
        smoothing_factor = 1
        
        r = R.from_quat(pos[-4:])
        heading = r.as_euler('xyz')[0]
        
        tangential_vel = (np.sin(heading) * float(vel[4]) + np.cos(heading) * float(vel[5])) * smoothing_factor + (1 - smoothing_factor) * self.tangential_last
        transverse_vel = (np.cos(heading) * float(vel[4]) - np.sin(heading) * float(vel[5])) * smoothing_factor + (1 - smoothing_factor) * self.transverse_last
        angular_vel = float(vel[-1]) * smoothing_factor + (1 - smoothing_factor) * self.angular_last
        
        self.tangential_last = tangential_vel
        self.transverse_last = transverse_vel
        self.angular_last = angular_vel
        
        
        
        if(np.abs(tangential_vel) + np.abs(transverse_vel) + np.abs(angular_vel) > 0.001):
            self.get_logger().info(f"{tangential_vel} {transverse_vel} {angular_vel}")
            self.get_logger().info(f"{self.tan_av / (self.av_n + 1)} {self.tran_av / (self.av_n + 1)} {angular_vel}")
            self.tan_av += tangential_vel
            self.tran_av += transverse_vel
            self.ang_av += angular_vel
            self.av_n += 1
        
        msg = Vector3()
        msg.x = angular_vel
        msg.y = tangential_vel
        msg.z = transverse_vel
        self.sim_avg.publish(msg)
        
        # real velocities at given time
        msg = None

            
       
        
        # vel = msg.velocity
        # pos = msg.position
        
        # r = R.from_quat(pos[-4:])
        # heading = r.as_euler('xyz')[0]
        
        # real_tangential_vel = (np.sin(heading) * float(vel[4]) + np.cos(heading) * float(vel[5])) * smoothing_factor + (1 - smoothing_factor) * self.real_tangential_last
        # real_transverse_vel = (np.cos(heading) * float(vel[4]) - np.sin(heading) * float(vel[5])) * smoothing_factor + (1 - smoothing_factor) * self.real_transverse_last
        # real_angular_vel = float(vel[-1]) * smoothing_factor + (1 - smoothing_factor) * self.real_angular_last
        
        # self.real_tangential_last = real_tangential_vel
        # self.real_transverse_last = real_transverse_vel
        # self.real_angular_last = real_angular_vel
        
        # msg = Vector3()
        # msg.x = real_angular_vel
        # msg.y = real_tangential_vel
        # msg.z = real_transverse_vel
        # self.real_avg.publish(msg)
        
        # self.get_logger().info(f"{self.last_ctrl}")
        # if real_tangential_vel == 0 or real_transverse_vel == 0 or real_angular_vel == 0 or self.last_ctrl == None  or (self.last_ctrl.x == 0 and self.last_ctrl.y == 0):
        #     return
        # euclidian norm
        # error = np.sqrt(((tangential_vel - real_tangential_vel)) ** 2 + ((transverse_vel - real_transverse_vel)) ** 2 + ((angular_vel - real_angular_vel)) ** 2)
        
       
        
        return
    
    def control_end_callback(self):
        self.pose_buffer = []
        self.pose_times = []
        self.ctrl_buffer = None
        self.ctrl_time = None
        
        
        self.bag_time_offset = None
        self.sim_time_offset = None
        reward = self.reward_function()
        self.optimizer.register(
            params=self.friction,
            target=reward
        )

        self.start_opt_cycle()
        
        msg = Float64()
        msg.data = (-reward)
        
        self.error_avg_pub.publish(msg)
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