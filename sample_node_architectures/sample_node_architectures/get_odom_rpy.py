import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from .angle_helpers import euler_from_quaternion
from math import pi

class OdomRPYNode(Node):
    def __init__(self):
        super().__init__('odom_rpy')
        self.create_subscription(Odometry, 'odom', self.process_odom, 10)

    def process_odom(self, msg):
        print(euler_from_quaternion(msg.pose.pose.orientation.x,
                                    msg.pose.pose.orientation.y,
                                    msg.pose.pose.orientation.z,
                                    msg.pose.pose.orientation.w))


def main(args=None):
    rclpy.init(args=args)
    node = OdomRPYNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
