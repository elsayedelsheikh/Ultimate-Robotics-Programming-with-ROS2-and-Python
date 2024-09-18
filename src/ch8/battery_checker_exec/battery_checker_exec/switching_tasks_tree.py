import rclpy
import py_trees
import time


#Parallel: battery reader - selector
 # Selector: battery ok: goto target - goto recharge station


from rclpy.node import Node
from std_msgs.msg import Float32

from threading import Thread

class Battery_Topic2BB(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(Battery_Topic2BB, self).__init__(name)
        self.node = node
        self.blackboard = self.attach_blackboard_client()
        
        self.blackboard.register_key(key='battery_level', access=py_trees.common.Access.WRITE)
        self.subscription = self.node.create_subscription(
            Float32,
            'battery_level_topic',
            self.battery_callback,
            10
        )
        print("Blackboard: ", self.blackboard)

        self.battery_level = None

    def battery_callback(self, msg):
        self.battery_level = msg.data

    def update(self):
        if self.battery_level is not None:
            self.blackboard.battery_level = self.battery_level
            self.logger.info(f"Updated Battery Level from topic: {self.blackboard.battery_level}%")
            #return py_trees.common.Status.SUCCESS
        #else:
        return py_trees.common.Status.SUCCESS

class ReadBattery(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(ReadBattery, self).__init__(name)
        print("name: ", name)
        #self.blackboard = self.attach_blackboard_client(name)
        #self.blackboard.register_key(key='battery_level', access=py_trees.common.Access.READ)
    
    def setup(self, **kwargs: int) -> None:
        #self.blackboard = self.attach_blackboard_client("ReadBattery")
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(key='battery_level', access=py_trees.common.Access.READ)

    def update(self):
        print("update")
        #return py_trees.common.Status.SUCCESS
        if self.blackboard.battery_level == -1:
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.SUCCESS        


class GoToTargetNode(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(GoToTargetNode, self).__init__(name)
        #print("name: ", name)
        #self.blackboard = self.attach_blackboard_client(name)
        #self.blackboard.register_key(key='battery_level', access=py_trees.common.Access.READ)
    
    def setup(self, **kwargs: int) -> None:
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(key='battery_level', access=py_trees.common.Access.READ)

    def update(self):
        
        #return py_trees.common.Status.SUCCESS
        if self.blackboard.battery_level < 20:
            print("Battery level too low, going to the recharing station")
            return py_trees.common.Status.FAILURE
        else:
            print("Going to the target")
            return py_trees.common.Status.SUCCESS        
class GoToChargingStation(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(GoToChargingStation, self).__init__(name)
        #print("name: ", name)
        #self.blackboard = self.attach_blackboard_client(name)
        #self.blackboard.register_key(key='battery_level', access=py_trees.common.Access.READ)
    
    #def setup(self, **kwargs: int) -> None:
    #    self.blackboard = self.attach_blackboard_client()
    #    self.blackboard.register_key(key='battery_level', access=py_trees.common.Access.READ)

    def update(self):
        
        #return py_trees.common.Status.SUCCESS
        #if self.blackboard.battery_level < 20:
        #    print("Battery level too low, going to the recharing station")
        #    return py_trees.common.Status.FAILURE
        #else:
        #    print("Going to the target")
        print("Going to the charging station")
        return py_trees.common.Status.SUCCESS        
    


def spin( node ):
    rclpy.spin(node)

def main(args=None):

    rclpy.init(args=args)
    #node = rclpy.create_node('battery_behavior_tree_node')
    node = rclpy.create_node("battery_behavior_tree_node")
    py_trees.blackboard.Blackboard.enable_activity_stream(maximum_size=100)
    blackboard = py_trees.blackboard.Client()

    blackboard.register_key(key='battery_level', access=py_trees.common.Access.WRITE)
    blackboard.battery_level = -1
    print("Blackboard: ", blackboard)
    
    root = py_trees.composites.Parallel(name="NavigationTask", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    update_battery = Battery_Topic2BB(name="Battery_Topic2BB", node=node)

    #task_selection = py_trees.composites.Selector(name="TaskSelection", memory=False)
    task_sequence = py_trees.composites.Sequence(name="TaskSequence", memory=False)
    task_selection= py_trees.composites.Selector(name="TaskSelection", memory=False)
    
    read_battery = ReadBattery(name="ReadBattery")
    goto_target = GoToTargetNode(name="GoToTarget")
    goto_charging_station = GoToChargingStation(name="GoToChargingStation")

    
        
    task_selection.add_children([goto_target, goto_charging_station,])
    task_sequence.add_children([read_battery,task_selection])
    root.add_children([update_battery,task_sequence,])

    root.setup_with_descendants()

    
    
    spin_thread = Thread(target=spin, args=(node,))
    spin_thread.start()
    
    while True:
        try:
            root.tick_once()
            print(py_trees.display.unicode_tree(root=root, show_status=True))
            time.sleep(1.0)
        except KeyboardInterrupt:
            break
    

if __name__ == "__main__":
    main()
