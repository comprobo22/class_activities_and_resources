import rclpy
from rclpy.node import Node
from neato2_interfaces.msg import Bump
from geometry_msgs.msg import Twist

class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('emergency_stop')
        self.create_timer(0.1, self.run_loop)
        self.create_subscription(Bump, 'bump', self.process_bump, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.bumper_active = False
    
    def run_loop(self):
        msg = Twist()
        msg.linear.x = 0.1 if not self.bumper_active else 0.0
        self.vel_pub.publish(msg)
    
    def process_bump(self, msg):
        self.bumper_active = (msg.left_front == 1 or \
                              msg.left_side == 1 or \
                              msg.right_front ==1 or \
                              msg.right_side == 1)
        print(self.bumper_active)


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
