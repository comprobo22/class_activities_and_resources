import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist
from os import path

class CmdVelMultiplexer(Node):
    def __init__(self):
        super().__init__('cmd_vel_multiplixer')
        (robots,) = self.declare_parameters(
            namespace='',
            parameters=[
                ('robots', [''])
            ]
        )
        self.vel_pubs = []
        for r in robots.value:
            topic = path.join(r, 'cmd_vel')
            self.vel_pubs.append(self.create_publisher(Twist, topic, 10))

        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_received, 10)

    def cmd_vel_received(self, msg):
        for vel_pub in self.vel_pubs:
            vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMultiplexer()
    rclpy.spin(node)
    rclpy.shutdown(node)

if __name__ == '__main__':
    main()
