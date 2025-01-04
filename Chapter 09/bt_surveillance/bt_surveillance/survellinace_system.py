import rclpy
import sys
import py_trees
import time
import ast
import random 
from threading import Thread

from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist


class wait_for_nav_client(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(wait_for_nav_client, self).__init__(name)
        self.node = node
        self._action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def update(self):       
        if ( self._action_client.wait_for_server(timeout_sec=0.5) == False):
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.SUCCESS

class rotate_in_place(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(rotate_in_place, self).__init__(name)
        self.node = node
        self.elapsed_time = 0.0
        self.publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)

        self.des_vel = Twist()
        self.des_vel.angular.z = 1.5
        
    def initialise(self):
        self.elapsed_time = 0.0


    def update(self):

        if( self.elapsed_time < (6.28 / self.des_vel.angular.z) ):
            self.publisher.publish(self.des_vel)
            self.elapsed_time = self.elapsed_time + 0.5
            sys.stdout.flush()

            return py_trees.common.Status.RUNNING
        else:
            
            self.des_vel.angular.z = 0.0
            self.publisher.publish(self.des_vel)
            time.sleep(1)
            return py_trees.common.Status.SUCCESS


class find_intruder(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(find_intruder, self).__init__(name)

    def update(self):       
        print("Checking intruder presence")
        time.sleep(2)
        random_integer = random.randrange(1, 10)
        if( random_integer > 8 ):
            print("Intruder found here")
            sys.stdout.flush()

            return py_trees.common.Status.SUCCESS
        else:
            print("No intruders here")
            sys.stdout.flush()
            return py_trees.common.Status.FAILURE

class navigation_client(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, id):
        super(navigation_client, self).__init__(name)
        self.node = node          
        self.id = id
        self._action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

        self.action_called = False
        self.action_done = False
        self.blackboard = self.attach_blackboard_client()

        self.blackboard.register_key(key="wps", access=py_trees.common.Access.READ)

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def initialise(self):
        self.action_done = False
        self.action_called = False

    def update(self):
        if( self.action_called == False ):
            self.action_called = True
            goal_x = 0.0
            goal_y = 0.0
            index = int( self.id )
            print( self.blackboard.wps[ index ] )
            goal_x = self.blackboard.wps[index][0]
            goal_y = self.blackboard.wps[index][1]
         
            target_pose = PoseStamped()
            target_pose.header.frame_id = "map"
            target_pose.pose.position.x = goal_x
            target_pose.pose.position.y = goal_y
            target_pose.pose.position.z = 0.0
            target_pose.pose.orientation.x = 0.0
            target_pose.pose.orientation.y = 0.0
            target_pose.pose.orientation.z = 0.0
            target_pose.pose.orientation.w = 1.0
            self.send_goal(target_pose)
        
        if( self.action_done == True ):
            return py_trees.common.Status.SUCCESS
        else:   
            return py_trees.common.Status.RUNNING

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:            
            return
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        self.action_done = True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        #print("Feedback: ", feedback)

def spin( node ):
    rclpy.spin(node)


def main(args=None) -> bool:

    rclpy.init(args=args)   
    node = rclpy.create_node("surveillance_system")


    node.declare_parameter('wps', "")
    wps = node.get_parameter('wps').value 
    if( wps == ""):
        print("No waypoints found")
        return False
        
    list_of_lists = ast.literal_eval(wps)
    list_of_points = [tuple(point) for point in list_of_lists]
    
    py_trees.blackboard.Blackboard.enable_activity_stream(maximum_size=100)
    blackboard = py_trees.blackboard.Client()
    
    blackboard.register_key(key='wps', access=py_trees.common.Access.WRITE)
    blackboard.wps = []
    for i in range(0, len(list_of_points)):
        blackboard.wps.append( [ list_of_points[i][0], list_of_points[i][1]] )


    
    sys.stdout.flush()

    root = py_trees.composites.Sequence(name="MainSequence", memory=True)
    #load_navigation_parameters_node = load_navigation_parameters(name="load_navigation_parameters", node=node)
    wait_for_nav_client_node = wait_for_nav_client(name="wait_for_nav_client", node=node)
    root.add_child(wait_for_nav_client_node)
    task_sequence = py_trees.composites.Selector(name="TaskSelector", memory=True)

    sequence_nodes = []
    for i in range(0, len(list_of_points)):
        seq = py_trees.composites.Sequence(name="Sequence_"+str(i), memory=True)
        navigation_client_node = navigation_client(name="navigation_client_" + str(i), node=node, id=str(i))
        rotate_in_place_node = rotate_in_place(name="rotate_in_place_" + str(i), node=node)
        find_intruder_node = find_intruder(name="find_intruder_" + str(i))
        
        seq.add_children([navigation_client_node, rotate_in_place_node, find_intruder_node])
        sequence_nodes.append(seq)


    for s in sequence_nodes:
        task_sequence.add_child(s)

    root.add_child(task_sequence)
    root.setup_with_descendants()

    spin_thread = Thread(target=spin, args=(node,))
    spin_thread.start()


    while True:
        try:
            root.tick_once()
            print(py_trees.display.unicode_tree(root=root, show_status=True))
            time.sleep(0.5)
            #print(root.status)
            sys.stdout.flush()

            if root.status == py_trees.common.Status.SUCCESS:
                print("All nodes succeeded. Terminating execution.")
                break
        except KeyboardInterrupt:
            break
    
    rclpy.shutdown()
    
    

if __name__ == '__main__':
    main()
