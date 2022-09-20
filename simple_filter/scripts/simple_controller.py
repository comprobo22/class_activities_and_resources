#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from simple_filter.msg import VelocitySimple

class Teleop(Node):
    def __init__(self):
        super().__init__('simple_teleop')
        self.pub = self.create_publisher(VelocitySimple, 'simple_vel', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        cv2.namedWindow('main_window')

    def run_loop(self):
        key = cv2.waitKey(10) & 0xFF
        if key != 0xFF:
            print(key)
        if key == 100:          # 'd'
            self.pub.publish(VelocitySimple(south_to_north_velocity=1.0))
        elif key == 97:         # 'a'
            self.pub.publish(VelocitySimple(south_to_north_velocity=-1.0))

def main(args=None):
    rclpy.init(args=args)      # Initialize communication with ROS
    node = Teleop()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()
