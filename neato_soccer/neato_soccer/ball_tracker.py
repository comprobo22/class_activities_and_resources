import rclpy
from threading import Thread
from rclpy.node import Node
import time
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3

class BallTracker(Node):
    """ The BallTracker is a Python object that encompasses a ROS node 
        that can process images from the camera and search for a ball within.
        The node will issue motor commands to move forward while keeping
        the ball in the center of the camera's field of view. """

    def __init__(self, image_topic):
        """ Initialize the ball tracker """
        super().__init__('ball_tracker')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        self.create_subscription(Image, image_topic, self.process_image, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        thread = Thread(target=self.loop_wrapper)
        thread.start()

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        cv2.namedWindow('video_window')
        cv2.namedWindow('binary_window')
        cv2.namedWindow('image_info')
        cv2.setMouseCallback('video_window', self.process_mouse_event)
        while True:
            self.run_loop()
            time.sleep(0.1)

    def process_mouse_event(self, event, x,y,flags,param):
        """ Process mouse events so that you can see the color values
            associated with a particular pixel in the camera images """
        self.image_info_window = 255*np.ones((500,500,3))
        cv2.putText(self.image_info_window,
                    'Color (b=%d,g=%d,r=%d)' % (self.cv_image[y,x,0], self.cv_image[y,x,1], self.cv_image[y,x,2]),
                    (5,50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0,0,0))

    def run_loop(self):
        # NOTE: only do cv2.imshow and cv2.waitKey in this function 
        if not self.cv_image is None:
            self.binary_image = cv2.inRange(self.cv_image, (128,128,128), (255,255,255))
            print(self.cv_image.shape)
            cv2.imshow('video_window', self.cv_image)
            cv2.imshow('binary_window', self.binary_image)
            if hasattr(self, 'image_info_window'):
                cv2.imshow('image_info', self.image_info_window)
            cv2.waitKey(5)

if __name__ == '__main__':
    node = BallTracker("/camera/image_raw")
    node.run()


def main(args=None):
    rclpy.init()
    n = BallTracker("camera/image_raw")
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()