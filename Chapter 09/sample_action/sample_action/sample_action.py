import rclpy
import py_trees
import time
from rclpy.action import ActionClient
from action_pkg.action import LinearControl
from std_msgs.msg import Float32
from threading import Thread

class wait_for_linear_control_client(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(wait_for_linear_control_client, self).__init__(name)
        self.node = node
        self._action_client = ActionClient(self.node, LinearControl, 'linear_control')

    def update(self):       
        if ( self._action_client.wait_for_server(timeout_sec=0.5) == False):
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.SUCCESS

class linear_control_client(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(linear_control_client, self).__init__(name)
        self.node = node          
        self._action_client = ActionClient(self.node, LinearControl, 'linear_control')
        self.feedback_publisher = self.node.create_publisher(Float32, '/linear_control/feedback', 10)
        self.current_fb = 0.0
        self.action_called = False
        self.action_done = False
    def send_goal(self, initial_position, goal_position, linear_velocity):

        goal_msg = LinearControl.Goal()
        goal_msg.initial_position = initial_position
        goal_msg.goal_position = goal_position
        goal_msg.linear_velocity = linear_velocity

        # Send the goal and wait for its acceptance and feedback
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def update(self):
        
        if( self.action_called == False ):
            self.action_called = True
            initial_position = 0.0
            goal_position = 1.7
            linear_velocity = 0.2
            self.send_goal( initial_position, goal_position, linear_velocity)
            
        if( self.action_done == True ):
            return py_trees.common.Status.SUCCESS
        else:   
            print("Distance from target: ", self.current_fb)
            return py_trees.common.Status.RUNNING

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:            
            return
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        result = future.result().result
        if( result.motion_done == True ): 
            self.action_done = True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        feedback_date = Float32()
        feedback_date.data = feedback.distance
        self.current_fb = feedback.distance
        self.feedback_publisher.publish(feedback_date)

def spin( node ):
    rclpy.spin(node)


def main(args=None):

    rclpy.init(args=args)   
    node = rclpy.create_node("linear_control")

    root = py_trees.composites.Sequence(name="Sequence", memory=True)
    wait_for_linear_control_client_node = wait_for_linear_control_client(name="wait_for_linear_control_client", node=node)
    linear_control_client_node = linear_control_client(name="linear_control_client", node=node)
    root.add_child(wait_for_linear_control_client_node)
    root.add_child(linear_control_client_node)
    root.setup_with_descendants()

    spin_thread = Thread(target=spin, args=(node,))
    spin_thread.start()
    
    while True:
        try:
            root.tick_once()
            print(py_trees.display.unicode_tree(root=root, show_status=True))
            time.sleep(1.0)
            #print(root.status)
            if root.status == py_trees.common.Status.SUCCESS:
                print("All nodes succeeded. Terminating execution.")
                break
        except KeyboardInterrupt:
            break

    rclpy.shutdown()

if __name__ == '__main__':
    main()
