import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class DistanceEmergencyStopNode(Node):
    def __init__(self):
        super().__init__('distance_emergency_stop')
        self.create_timer(0.1, self.run_loop)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.distance_to_obstacle = None
        self.Kp = 0.4
        self.target_distance = 0.5
    
    def run_loop(self):
        msg = Twist()
        if self.distance_to_obstacle is None:
            msg.linear.x = 0.1
        else:
            msg.linear.x = self.Kp*(self.distance_to_obstacle - self.target_distance)
        self.vel_pub.publish(msg)

    def process_scan(self, msg):
        if msg.ranges[0] != 0.0:
            self.distance_to_obstacle = msg.ranges[0]


def main(args=None):
    rclpy.init(args=args)
    node = DistanceEmergencyStopNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
