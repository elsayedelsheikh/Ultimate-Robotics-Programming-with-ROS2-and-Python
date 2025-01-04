#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class SinusoidalPublisher(Node):

    def __init__(self):
        super().__init__('sinusoidal_publisher')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('frequency', 1.0),  # Default frequency: 1 Hz
                ('amplitude', 1.0)   # Default amplitude: 1.0
            ]
        )

        # Get parameters
        self.frequency = self.get_parameter('frequency').value
        self.amplitude = self.get_parameter('amplitude').value

        # Create publisher
        self.publisher = self.create_publisher(Float64, 'sinusoidal_signal', 10)

        # Create timer to publish sinusoidal signal
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # Calculate the value of the sinusoidal signal
        value = self.amplitude * math.sin(2 * math.pi * self.frequency * self.i)
        
        # Publish the value
        msg = Float64()
        msg.data = value
        self.publisher.publish(msg)
        
        # Log the published value
        self.get_logger().info('Publishing: %f' % msg.data)
        
        # Increment counter
        self.i += 0.1

def main(args=None):
    rclpy.init(args=args)
    sinusoidal_publisher = SinusoidalPublisher()
    rclpy.spin(sinusoidal_publisher)
    sinusoidal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

