import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from bayes_opt import acquisition, BayesianOptimization


class Optimize(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        acq = acquisition.UpperConfidenceBound(kappa=2.5)
        self.optimizer = BayesianOptimization(
            f=None,
            acquisition_function=acq,
            pbounds={'x': (-2, 2), 'y': (-3, 3)},
            verbose=2,
            random_state=1,
        )


        
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        
        
    def start_bayesian_cycle(self):
        self.next_friction = self.optimizer.suggest()
        
        # TODO: set friction in sim, start control
        
        return
        
    def reward_function(self):
        # I don't quite know how to impliment this yet
        return 0
    
    def pose_callback(self):
        # subscribe to and record pose topic
        # compute slip from wheel velocities, append to dict with timestamps?
        
        return
    
    def control_end_callback(self):
        self.optimizer.register(
            params=self.next_friction,
            target=self.reward_function(),
        )

        self.start_control_sequence
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