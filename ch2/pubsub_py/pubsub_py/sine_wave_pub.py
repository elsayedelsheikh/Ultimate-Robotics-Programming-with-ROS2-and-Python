#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# Import the datatype to publish towards the ROS2 network
from std_msgs.msg import Float64

# Math libraries to perform mathematical calculus 
import math

class SinusoidalPublisher(Node):

    def __init__(self):
        super().__init__('sinusoidal_publisher')

        # Create publisher
        self.publisher = self.create_publisher(Float64, 'sinusoidal_signal', 10)

        # Create timer to publish sinusoidal signal
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.i = 0

    def timer_callback(self):       
        
        # Set the parameters characterizing the sine wave signal
        amplitude = 2.0
        frequency = 0.1
        msg = Float64()

        # Calculate the sine wave signal at time i
        msg.data = amplitude * math.sin(2 * math.pi * frequency * self.i )
        # Publish the data
        self.publisher.publish(msg)
            
        # Increment counter
        self.i += 0.1

def main(args=None):

	# Init and spin the node
	rclpy.init(args=args)
	sinusoidal_publisher = SinusoidalPublisher()
	rclpy.spin(sinusoidal_publisher)
	sinusoidal_publisher.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

