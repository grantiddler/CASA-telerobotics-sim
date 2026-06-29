import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Optimize(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        
    def reward_function(self):
        # reset rover position
        # restart control routine
        # compare the rivers path to the correct path
        # wait for control routine to finish somehow?
        # this has to pause so the pose_callback can be run, I don't know exactly how to accomplish that in python yet.
        return
    
    def pose_callback(self):
        # subscribe to and record pose topic
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