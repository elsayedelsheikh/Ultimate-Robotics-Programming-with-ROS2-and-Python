import sys
import rclpy
from rclpy.node import Node
# Import the service message type used to fill the request
from srv_pkg.srv import JointConversion


class JointConversionClient(Node):

    def __init__(self):
        super().__init__('joint_conversion_client')

	# Create the service client object and wait that it appears online
        self.cli = self.create_client(JointConversion, 'joint_conversion')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')


    def send_request(self, j_values):
    
        # Fill the request part, considering the 7 joint values    
        req = JointConversion.Request()
        for i in range( len(j_values) ):
           req.joint_input[i].data = j_values[i]

        # Call the server, using the client object
        # The result is a future object that can be used to monitor its status
        self.future = self.cli.call_async(req)
        
        # The future object is used in this function to wait that the server replies  with a given resultt
        rclpy.spin_until_future_complete(self, self.future)
        
        # The result is returned to the main function
        return self.future.result()


def main(args=None):

    rclpy.init(args=args)

    conversion_client = JointConversionClient()
    
    # Set random values for the joint vector
    joint_values = [1.0, 0.78, 1.6, 0.0, 1.57, 0.3, 1.2]
  
    # Call the client
    response = conversion_client.send_request(joint_values)
    print("Response: ", response)

    conversion_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
