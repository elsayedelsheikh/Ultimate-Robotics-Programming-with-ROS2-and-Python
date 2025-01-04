import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=100
        )

        self.publisher_ = self.create_publisher(String, 'data', qos_profile)
        self.timer = self.create_timer(0.1, self.publish_sensor_data)  # Publish at 10 Hz
        self.counter = 0

    def publish_sensor_data(self):
        msg = String()
        msg.data = f'Sensor data {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()
    rclpy.spin(sensor_publisher)
    sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

