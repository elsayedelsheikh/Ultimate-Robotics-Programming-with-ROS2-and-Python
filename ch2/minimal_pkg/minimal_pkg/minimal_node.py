# Importing ROS2 client libraries
import rclpy
from rclpy.node import Node

# Setup a new Node by defining a new Object inheriting from Node class of ROS2
class MinimalNode(Node):

    def __init__(self):
        super().__init__('minimal_node')
        
        # Use the logger function to print out an hello message
        self.get_logger().info('Hello, ROS2!')

def main(args=None):
   
    # Initialize th ROS2 middleware from this node
    rclpy.init(args=args)
    minimal_node = MinimalNode()
    minimal_node.destroy_node()
    
if __name__ == '__main__':
    main()
