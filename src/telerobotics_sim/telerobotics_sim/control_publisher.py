import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Vector3


torques = [[0.0, 0.0],
           [4.5, 4.5],
           [4.5, 3.375],
           [4.5, 2.25],
           [4.5, 1.125],
           [4.5, 0.0],
           [4.5, -1.125],
           [4.5, -2.25],
           [4.5, -3.375],
           [4.5, -4.5]]
times = [1500, 30000,30000,30000,30000,30000,30000,30000,30000,30000]

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Vector3, 'control', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.n = 0

    def timer_callback(self):
        # read through a bag with motor efforts, publish those motor efforts
        return
            
    def optimize_callback(self): # use a service for this, not a publisher?
        # reset everything 
        # start timer callback running
        # wait for timer callback to finish somehow? maybe publisher and this is a different function?
        return


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()