import rclpy
from rclpy.node import Node
from calc_msgs.msg import CalcMsg
from std_msgs.msg import Int32

class CalcServ(Node):

    def __init__(self):
        super().__init__('calc_server')
        self.subscription = self.create_subscription(
            CalcMsg,
            'calculator',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Int32, 'result_topic', 10)

    def listener_callback(self, msg):
        print( msg )
        result = Int32()
        if msg.operation == 'add':
            result.data = msg.a + msg.b
        elif msg.operation == 'subtract':
            result.data = msg.a - msg.b
        elif msg.operation == 'multiply':
            result.data = msg.a * msg.b
        elif msg.operation == 'divide' and msg.b != 0:
            result.data = int(msg.a / msg.b)
        else:
            result.data = 0  # Error case
        self.publisher_.publish(result)
        self.get_logger().info(f'Published result: {result.data}')

def main(args=None):
    rclpy.init(args=args)
    node = CalcServ()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
