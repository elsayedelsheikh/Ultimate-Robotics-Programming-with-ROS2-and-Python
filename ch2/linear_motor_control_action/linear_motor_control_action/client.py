import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

# Import the action client module to implement the client
from rclpy.action import ActionClient

# Import the action 
from action_pkg.action import LinearControl


class LinearControlClient(Node):

    # Define the action client specifyng the name of the action and the action type
    def __init__(self):
        super().__init__('linear_control_action_client')
        self._action_client = ActionClient(self, LinearControl, 'linear_control')


    # Prepare the action request 
    def send_goal(self, initial_position, goal_position, linear_velocity):

        goal_msg = LinearControl.Goal()
        goal_msg.initial_position = initial_position
        goal_msg.goal_position = goal_position
        goal_msg.linear_velocity = linear_velocity

        # Wait the server to be discovered
        self._action_client.wait_for_server()

        # Send the goal and wait for its acceptance and feedback
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # It the goal has been accepted, we can define a new callback to listen its result    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.motion_done))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.distance))
        

def main(args=None):

    rclpy.init(args=args)
    action_client = LinearControlClient()

    initial_position = 0.0
    goal_position = 1.7
    linear_velocity = 0.2

    future = action_client.send_goal( initial_position, goal_position, linear_velocity)
 
 
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()

