import rclpy
from rclpy.node import Node
from calc_msgs.msg import CalcMsg

class CalcRequest(Node):

    def __init__(self):
        super().__init__('calc_requester')
        self.publisher_ = self.create_publisher(CalcMsg, 'calculator', 10)
        #self.timer = self.create_timer(1.0, self.publish_message)
        self.publish_message(5, 3, 'add')
        
    def publish_message(self, a, b, operation):
        msg = CalcMsg()
        msg.a = a
        msg.b = b
        msg.operation = operation
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: a={msg.a}, b={msg.b}, operation={msg.operation}')
        

def main(args=None):
    rclpy.init(args=args)
    node = CalcRequest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
