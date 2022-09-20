#!/usr/bin/env python3

""" A very simple simulator for a robot in 1d with forward
    and rear facing range sensor """

import rclpy
from rclpy.node import Node
from numpy.random import randn
from simple_filter.msg import LaserSimple, VelocitySimple , OdometrySimple
import numpy as np
from std_msgs.msg import Float64

class SimpleWorldNode(Node):
    def __init__(self):
        super().__init__('simple_world')
        
        self.declare_parameter('walls', [0.0, 2.0])
        walls = self.get_parameter('walls').value
        self.world = WorldModel(walls=walls)
        self.pub = self.create_publisher(LaserSimple, 'simple_scan', 10)
        self.pub_pos = self.create_publisher(Float64, 'true_position', 10)
        self.pub_odom = self.create_publisher(OdometrySimple, 'simple_odom', 10)
        self.next_velocity = None
        self.create_subscription(VelocitySimple, 'simple_vel', self.process_simple_vel, 10)
        self.create_timer(self.world.dt, self.run_loop)

    def run_loop(self):
        if self.next_velocity == None:
            sensation = self.world.get_sensation()
        else:
            sensation = self.world.do_action(self.next_velocity)
            self.next_velocity = None
        self.pub.publish(sensation)
        self.pub_pos.publish(Float64(data=self.world.position))
        self.pub_odom.publish(OdometrySimple(south_to_north_position=self.world.odom_position))

    def process_simple_vel(self, msg):
        self.next_velocity = msg.south_to_north_velocity

class WorldModel(object):
    def __init__(self, noise_rate=.05, odom_noise_rate=.1, walls=None):
        if walls == None:
            self.walls =[]
        else:
            self.walls = walls
        self.position = randn()*0.2+1.5
        self.odom_position = 0.0
        self.odom_noise_rate = odom_noise_rate
        self.noise_rate = noise_rate
        self.dt = .2

    def add_wall(self, wall_position):
        self.walls.append(wall_position)

    def get_closest_obstacle(self, position, direction):
        if direction == -1:
            positions = [(position - w, idx) for idx, w in enumerate(self.walls) if position - w >= 0]
            if len(positions) == 0:
                return None
            min_idx = np.argmin([p[0] for p in positions])
            return self.walls[positions[min_idx][1]]
        else:
            positions = [(w - position, idx) for idx, w in enumerate(self.walls) if w - position >= 0]
            if len(positions) == 0:
                return None
            min_idx = np.argmin([p[0] for p in positions])
            return self.walls[positions[min_idx][1]]

    def get_sensation(self):
        closest_north = self.get_closest_obstacle(self.position, 1)
        closest_south = self.get_closest_obstacle(self.position, -1)
        if closest_north == None:
            north_laser_reading = 0.0
        else:
            north_laser_reading = (closest_north - self.position) + self.noise_rate*randn()
        if closest_south == None:
            south_laser_reading = 0.0
        else:
            south_laser_reading = (self.position - closest_south) + self.noise_rate*randn()

        return LaserSimple(south_laser= south_laser_reading,
                           north_laser= north_laser_reading)

    def do_action(self, velocity):
        self.position += velocity*self.dt
        self.odom_position += velocity*self.dt + self.odom_noise_rate*randn()
        return self.get_sensation()

def main(args=None):
    rclpy.init(args=args)      # Initialize communication with ROS
    node = SimpleWorldNode()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()