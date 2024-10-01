import rclpy
from yolo_msgs.msg import DetectedObject, DetectedObjectList
from rclpy.node import Node


from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.duration import Duration
from threading import Thread

from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
#import tf_transformations
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
import numpy as np
#import tf_transformations 

class SeekAndGo(Node):  
    def __init__(self):
        super().__init__('seek_and_go')

        self.obj_input_sub = self.create_subscription(String, '/object_to_seek', self.object_to_seek_input, 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Periodically check for the transform between map and base_link
        self.timer = self.create_timer(1.0, self.get_transform)

        self.object_to_seek = None
        self.new_object_to_seek = False
        self.reahing_obj = False

        self.timer = self.create_timer(0.2, self.get_transform)

    def get_transform(self):
        try:
            # Try to get the transform between map and base_link
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform('map', 'base_link', now, Duration(seconds=1.0))

            # Extract the yaw from the transform's rotation
            yaw = self.get_yaw_from_transform(transform)
            self.get_logger().info(f"Yaw (in radians): {yaw}")

        except Exception as e:          
            self.get_logger().warn(f"Could not get transform: {e}")

    def get_yaw_from_transform(self, transform: TransformStamped):
        rotation = transform.transform.rotation
        quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)
        return self.quaternion_to_euler(quaternion)[2]

    def quaternion_to_euler(self, quat):
   
        x, y, z, w = quat

        # Quaternion to Euler conversion (roll, pitch, yaw)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)

        return roll_x, pitch_y, yaw_z
    

    def seek_object( self ):
        print("Seeking object: ", self.object_to_seek)
        return True
    
    def goto_object( self ):
        print("Going to object: ", self.object_to_seek)



    def main_loop(self):
        rate = self.create_rate(2)
        obj_found = False
        obj_reached = False


        while rclpy.ok():

            if ( self.reahing_obj == False):

                if self.new_object_to_seek:
                    self.reahing_obj = True
            
                    obj_found = self.seek_object()             
                    self.new_object_to_seek = False

                if( obj_found == True):
                    self.goto_object()


            rate.sleep()
    def object_to_seek_input(self, msg):        
        self.object_to_seek = msg.data
        self.new_object_to_seek = True


    def run( self ):
        main_loop_thread = Thread(target = self.main_loop, args = ())
        main_loop_thread.start()
        
        rclpy.spin(self)
def main(args=None):
    rclpy.init(args=args)
    node = SeekAndGo()
    node.run()


if __name__ == '__main__':
    main()