#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from simple_filter.msg import LaserSimple, OdometrySimple
from nav_msgs.msg import Odometry

class NeatoBridge(Node):
    def __init__(self):
        super().__init__('neato_bridge')
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.pub = self.create_publisher(LaserSimple, 'simple_scan', 10)
        self.create_subscription(Odometry, 'odom', self.process_odom, 10)
        self.pub_odom = self.create_publisher(OdometrySimple, 'simple_odom', 10)
        self.last_odom = None

    def process_scan(self, msg):
        self.pub.publish(LaserSimple(north_laser=msg.ranges[0],
                                     south_laser=msg.ranges[180],
                                     east_laser=msg.ranges[270],
                                     west_laser=msg.ranges[90]))

    def process_odom(self,msg):
        if (self.last_odom == None or
            abs(self.last_odom[0] - msg.pose.pose.position.x)>0.1 or
            abs(self.last_odom[1] - msg.pose.pose.position.y)>0.1):
            self.pub_odom.publish(OdometrySimple(south_to_north_position=msg.pose.pose.position.x,
                                                 west_to_east_position=msg.pose.pose.position.y))
            self.last_odom = (msg.pose.pose.position.x, msg.pose.pose.position.y)

def main(args=None):
    rclpy.init(args=args)      # Initialize communication with ROS
    node = NeatoBridge()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()
