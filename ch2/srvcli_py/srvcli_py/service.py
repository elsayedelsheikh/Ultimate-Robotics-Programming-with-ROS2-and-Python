import rclpy
import time
from srv_pkg.srv import JointConversion
from rclpy.node import Node


class JointConversionService(Node):

    def __init__(self):
        super().__init__('joint_conversion_service')
	
	# Create the service object. Here we specify the function that is invoked when the service is called
        self.srv = self.create_service(JointConversion, 'joint_conversion', self.joint_conversion_callback)
        
        # A random set of values representing the offset to add to the joint vector
        self.offset = [0.2, 0.2, 0.2, 0.5, 0.5, 0.5, 0.0]

    def joint_conversion_callback(self, request, response):
        
        # Save the input vector of joints into a list
        j_values = []
        for i in range( len(request.joint_input) ):
           j_values.append(request.joint_input[i].data)
                   
        # Apply the offset
        result = [x + y for x, y in zip(j_values, self.offset)]
        
        # Transform from rad to degree
        result = [(x * 180.0) / 3.1415 for x in result]

	# Retro transofrm the list into the result field of the service
        for i in range( len(result) ):
           response.joint_output[i].data = result[i]

        # Just wait a bit, to check the output on the terminal
        time.sleep(2)
        print("Conversion performed")
        return response


def main(args=None):
    rclpy.init(args=args)
    conversion = JointConversionService()
    rclpy.spin(conversion)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

