#!/usr/bin/env python3

"""
    This script implements a Kalman filter for the system:

    x_0 ~ N(0, sigma_sq)
    x_t = x_{t-1} + w_t, w_t ~ N(0, sigma_m_sq)
    z_t = x_t + v_t, v_t ~ N(0, sigma_z_sq)
"""

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from numpy import arange
from numpy.random import randn
from math import e, sqrt, pi
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

class SimpleWorld(object):
    """ A simple system with dynamics:
        x_0 ~ N(0, sigma_sq)
        x_t = x_{t-1} + w_t, w_t ~ N(0, sigma_m_sq)
        z_t = x_t + v_t, v_t ~ N(0, sigma_z_sq)
    """

    def __init__(self, mu_0, sigma_0, sigma_m_sq, sigma_z_sq):
        """ the initial state is sampled from N(mu_0, sigma_0).
            the movement noise is sigma_m_sq and the measurement noise is sigma_z_sq
        """
        self.x_true = mu_0 + sqrt(sigma_0)*randn()
        self.sigma_m_sq = sigma_m_sq
        self.sigma_z_sq = sigma_z_sq

    def get_z_t(self):
        """ Sample an observation centered at x_true plus Gaussian noise
            with variance sigma_sq_z and mean 0 """
        return self.x_true + sqrt(self.sigma_z_sq)*randn()

    def get_x_t(self):
        """ Sample next system state as the current system state plus Gaussian
            noise with variance sigma_sq_m and mean 0 """
        self.x_true = self.x_true + sqrt(self.sigma_m_sq)*randn()
        return self.x_true

class SimpleKalmanFilter(Node):
    """ A Kalman filter node that estimates a single state x_t using noisy position measurements """

    def __init__(self):
        """ Sets up the world model and loads initial parameters """
        super().__init__('simple_kalman')
        plt.ion()

        # initial beliefs
        self.mu = 0
        self.sigma_sq = 1
        (sigma_m_sq, sigma_z_sq, pause_time) = self.declare_parameters(
            namespace='',
            parameters=[
                ('sigma_m_sq',  0.01),
                ('sigma_z_sq', 0.1),
                ('pause_time', 0.5)
            ]
        )
        # time to pause between plots
        self.pause_time = pause_time.value

        self.graphs = None
        self.world = SimpleWorld(self.mu, self.sigma_sq, sigma_m_sq.value, sigma_z_sq.value)
        self.create_timer(0.01, self.run_loop)
        self.last_update = self.get_clock().now()
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        """ This callback allows the parameters of the node to be adjusted
            dynamically by other ROS nodes (e.g., dynamic_reconfigure).
            This function is only needed to support dynamic_reconfigure. """
        for param in params:
            if param.name == 'pause_time' and param.type_ == Parameter.Type.DOUBLE:
                self.pause_time = param.value
            elif param.name == 'sigma_m_sq' and param.type_ == Parameter.Type.DOUBLE:
                self.world.sigma_m_sq = param.value
            elif param.name == 'sigma_z_sq' and param.type_ == Parameter.Type.DOUBLE:
                self.world.sigma_z_sq = param.value
        print(self.pause_time, self.world.sigma_m_sq, self.world.sigma_z_sq)
        return SetParametersResult(successful=True)

    def run_loop(self):
        if self.get_clock().now() - self.last_update > rclpy.time.Duration(seconds=self.pause_time):
            self.last_update = self.get_clock().now()
            # Graph new observation from the system
            z_t = self.world.get_z_t()
            self.graphs = self.plot_pdf(z_t)

            # Do Kalman updates
            K_t = (self.sigma_sq + self.world.sigma_m_sq)/(self.sigma_sq + self.world.sigma_m_sq + self.world.sigma_z_sq)
            self.mu = self.mu + K_t*(z_t - self.mu)
            self.sigma_sq = (1-K_t)*(self.sigma_sq+self.world.sigma_m_sq)
            self.graphs = self.plot_pdf(z_t)

            # sample next state
            self.world.get_x_t()
            plt.pause(0.01)

    def plot_pdf(self, z):
        """ Plot the Gaussian PDF with the specified mean (mu) and variance (sigma_sq)
            x_true is the true system state which will be plotted in blue
            z is the current observation which will be plotted in red """
        xs = arange(min(-5,z-2,self.world.x_true-2), max(5,z+2,self.world.x_true+2), .005)
        p_of_x = [1./sqrt(2*pi*self.sigma_sq)*e**(-(x - self.mu)**2/(2*self.sigma_sq)) for x in xs]
        plt.xlim([min(xs), max(xs)])
        if self.graphs:
            self.graphs[0].set_xdata(xs)
            self.graphs[0].set_ydata(p_of_x)
            self.graphs[1].set_xdata(self.world.x_true)
            self.graphs[2].set_xdata(z)
        else:
            self.graphs = []
            self.graphs.append(plt.plot(xs, p_of_x)[0])
            self.graphs.append(plt.plot(self.world.x_true, 0,'b.')[0])
            self.graphs.append(plt.plot(z, 0,'r.')[0])
            self.graphs[1].set_markersize(20)
            self.graphs[2].set_markersize(20)
            plt.ylim([0, 5])
            plt.legend(('probability density','true position','measured position'))
        plt.show(block=False)
        return self.graphs

def main(args=None):
    rclpy.init(args=args)      # Initialize communication with ROS
    node = SimpleKalmanFilter()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()