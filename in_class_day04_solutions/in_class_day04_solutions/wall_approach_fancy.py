import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data

class WallApproachNode(Node):
    def __init__(self):
        super().__init__('wall_approach')
        self.create_timer(0.1, self.run_loop)
        self.create_subscription(LaserScan, 'scan', self.process_scan, qos_profile=qos_profile_sensor_data)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.distance_to_obstacle = None
        self.declare_parameters(namespace='',
                parameters=[('Kp', 0.5), ('target_distance', 0.5)])
        self.Kp = self.get_parameter('Kp').value
        self.target_distance = self.get_parameter('target_distance').value
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'Kp' and param.type_ == Parameter.Type.DOUBLE:
                self.Kp = param.value
            elif param.name == 'target_distance' and param.type_ == Parameter.Type.DOUBLE:
                self.target_distance = param.value
        print(self.Kp, self.target_distance)
        return SetParametersResult(successful=True)


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
    node = WallApproachNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
