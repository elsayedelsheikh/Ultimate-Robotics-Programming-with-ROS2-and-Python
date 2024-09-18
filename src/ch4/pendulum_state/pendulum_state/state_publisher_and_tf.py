import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformListener, Buffer
import math
import time

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_state)
        self.joint_state = JointState()

        # Initialize joint names
        self.joint_state.name = ['revolute_joint']
        self.joint_state.position = [0.0]
        self.joint_state.velocity = [0.0]
        self.joint_state.effort = [0.0]

        # Initialize TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def publish_joint_state(self):
        # Update the joint state
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        
        # Example of updating joint positions with a sine wave
        self.joint_state.position[0] = math.sin(time.time())

        self.publisher_.publish(self.joint_state)
        #self.get_logger().info('Publishing: "%s"' % str(self.joint_state.position))

        # Lookup the transform from 'base_link' to 'end_effector'
        try:
            trans = self.tf_buffer.lookup_transform('base_link', 'pole_link', rclpy.time.Time())
            self.get_logger().info('Transform: "%s"' % str(trans.transform))
        except Exception as e:
            self.get_logger().warn('Could not transform: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()