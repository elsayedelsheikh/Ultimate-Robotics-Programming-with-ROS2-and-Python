import rclpy
import math 
import time
from rclpy.node import Node

# Import the modules to implement the server part of the client server
from rclpy.action import ActionServer
from action_pkg.action import LinearControl

class LinearControlServer(Node):

    # Create the action server
    def __init__(self):
        super().__init__('linear_control_action_server')
        self._action_server = ActionServer(
            self,
            LinearControl,
            'linear_control',
            self.execute_callback)
        
    # In the execution callback we calculate the feedback and the result of the action
    def execute_callback(self, goal_handle):
        
        curr_pos = goal_handle.request.initial_position
        goal_pos = goal_handle.request.goal_position
        vel = goal_handle.request.linear_velocity
        
        dist = math.fabs( curr_pos - goal_pos ) 

        rate = self.create_rate(50)
        step_t = 1.0/50.0
        
        feedback_msg = LinearControl.Feedback()        
        
        # While the goal has not been reached (should be while dist > 0, but 0 is very difficult to catch)
        while dist > 1e-2:
        
            # Update the motor position value
            if( curr_pos < goal_pos ) > 0:
               curr_pos = curr_pos + vel*step_t
            else:
               curr_pos = curr_pos - vel*step_t
            
            # Calcualte the distance to fill also the feedback
            dist = math.fabs( curr_pos - goal_pos ) 
            feedback_msg.distance = dist
            
            # Publish the feedback
            goal_handle.publish_feedback( feedback_msg )
            time.sleep( 0.02 )
            
        goal_handle.succeed()
        result = LinearControl.Result()
        result.motion_done = True
        return result
        

def main(args=None):
    rclpy.init(args=args)
    linear_control_action_server = LinearControlServer()
    rclpy.spin(linear_control_action_server)
    
if __name__ == '__main__':
    main()
    
    
    
    
    
    

