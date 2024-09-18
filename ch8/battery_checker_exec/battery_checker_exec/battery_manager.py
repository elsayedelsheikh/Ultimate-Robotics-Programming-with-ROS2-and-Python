import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random
import time

class BatteryPublisher(Node):

    def __init__(self):
        super().__init__('battery_publisher')
        self.publisher_ = self.create_publisher(Float32, 'battery_level_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.battery_level = random.uniform(50.0, 100.0)
        self.discharge_rate = random.uniform(1.0, 4.0)

        self.get_logger().info(f'Initial Battery Level: {self.battery_level}%')
        self.get_logger().info(f'Discharge Rate: {self.discharge_rate} Hz')
    def timer_callback(self):
        self.battery_level -= self.discharge_rate
        if self.battery_level < 0:
            self.battery_level = 0
        msg = Float32()
        msg.data = self.battery_level
        self.publisher_.publish(msg)
        self.get_logger().info(f'Battery Level: {self.battery_level}%')
        if self.battery_level == 0:
            self.get_logger().info('Battery is fully discharged.')

def main(args=None):
    rclpy.init(args=args)
    battery_publisher = BatteryPublisher()
    rclpy.spin(battery_publisher)
    battery_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()