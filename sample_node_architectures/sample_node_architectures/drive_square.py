import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi

class DriveSquareSample1(Node):
    def __init__(self):
        super().__init__('drive_square_sample_1')
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.1, self.run_loop)
        self.turns_executed = 0
        self.executing_turn = False
        self.side_length = 0.5      # the length in meters of a square side
        self.time_per_side = 5.0    # duration in seconds to drive the square side
        self.time_per_turn = 2.0    # duration in seconds to turn 90 degrees
        # start_time_of_segment indicates when a particular part of the square was
        # started (e.g., a straight segment or a turn)
        self.start_time_of_segment = None   

    def run_loop(self):
        """ In the run_loop we are essentially implementing what's known as a finite-state
            machine.  That is, our robot code is in a particular state (in this case defined
            by whether or not we are turning and how many sides we've traversed thus far.
            
            Our run loop does the following things:
              1. if we haven't yet marked the start time of the segment, we do so by grabbing the current time
              2. we compute the desired time for the particular move we are executing (this will be our criteria to change state)
              3. we check to see if we are done with our current segment and should move onto the next state
                 -if we are done-
                    - Transition to the next state by switching from a turn to a straight segment (or vice versa)
                    - Reset the start time variable of the segment
                    - set the desired velocities to 0 (so we stop in between each segment)
                 -if we are not done-
                    - Compute the appropriate velocity command based on the state
                4. publish the velocity command 
            """
        if self.start_time_of_segment is None:
            self.start_time_of_segment = self.get_clock().now()
        msg = Twist()
        if self.executing_turn:
            segment_duration = self.time_per_turn
        else:
            segment_duration = self.time_per_side

        # check to see if we are done with the segment
        # here, I use self.get_clock().now() which is better than using time.time() since it works
        # equally well with simulator or wall clock time (e.g., the simulator might not run
        # at real-time).  You are totally fine using time.time(), but I wanted to show this.
        if self.get_clock().now() - self.start_time_of_segment > rclpy.time.Duration(seconds=segment_duration):
            if self.executing_turn:
                self.turns_executed += 1
            # toggle the executing_turn Boolean (turn to not turn or vice versa)
            self.executing_turn = not self.executing_turn
            self.start_time_of_segment = None
            print(self.executing_turn, self.turns_executed)
            # transition to next segment, don't change msg so we execute a stop
        else:
            if self.executing_turn:
                # we are trying to turn pi/2 radians in a particular amount of time
                # from this we can get the angular velocity
                msg.angular.z = (pi / 2) / segment_duration
            else:
                msg.linear.x = self.side_length / segment_duration
        self.vel_pub.publish(msg) 

def main(args=None):
    rclpy.init(args=args)
    node = DriveSquareSample1()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
