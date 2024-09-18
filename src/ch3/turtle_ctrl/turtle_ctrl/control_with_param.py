import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class TurtleControl(Node):
    def __init__(self):
    
        super().__init__('turtle_control_param')
  
  			# Define parameters list      
        self.declare_parameter('linear_velocity_scale',  1.0)
        self.declare_parameter('angular_velocity_scale', 1.0)
        self.declare_parameter('max_linear_velocity',    2.0) 
        self.declare_parameter('max_angular_velocity',   2.5)
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
       
        self.linear_velocity_scale = self.get_parameter('linear_velocity_scale').get_parameter_value().double_value 
        self.angular_velocity_scale = self.get_parameter('angular_velocity_scale').get_parameter_value().double_value 
        self.max_linear_velocity  = self.get_parameter('max_linear_velocity').get_parameter_value().double_value 
        self.max_angular_velocity  = self.get_parameter('max_angular_velocity').get_parameter_value().double_value 
        
        print("Parameters value: ")
        print("linear_velocity_scale: ", self.linear_velocity_scale)
        print("angular_velocity_scale: ", self.angular_velocity_scale)
        print("max_linear_velocity: ", self.max_linear_velocity)
        print("max_angular_velocity: ", self.max_angular_velocity)
        
        self.vel_x = 1.0*self.linear_velocity_scale
        self.ang_vel_z = 1.0*self.angular_velocity_scale
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def timer_callback(self):       
  
        v = Twist()   
        v.linear.x = self.vel_x
        v.angular.z = self.ang_vel_z
      
        self.cmd_vel_publisher.publish(v)

    # Param callback
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'linear_velocity_scale':
                self.linear_velocity_scale = param.value
            elif param.name == 'angular_velocity_scale':
                self.angular_velocity_scale = param.value
            elif param.name == 'max_linear_velocity':
                self.max_linear_velocity = param.value
            elif param.name == 'max_angular_velocity':
                self.max_angular_velocity = param.value
            
        des_linear_vel = self.vel_x*self.linear_velocity_scale
        if des_linear_vel < self.max_linear_velocity:
           self.vel_x = des_linear_vel      
        else:
           self.get_logger().warn('Desired linear velocity too high') 
           
        des_angular_vel = self.ang_vel_z*self.angular_velocity_scale
        if des_angular_vel < self.max_angular_velocity:
           self.ang_vel_z = des_angular_vel
        else:
           self.get_logger().warn('Desired angular velocity too high') 


        
        return SetParametersResult(successful=True)

def main(args=None):
    
    rclpy.init(args=args)
    node = TurtleControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

