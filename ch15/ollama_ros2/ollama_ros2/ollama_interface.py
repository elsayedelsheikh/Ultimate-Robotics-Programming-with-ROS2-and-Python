import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ollama 


class OllamaInterface(Node):
    def __init__(self):
        super().__init__('ollama_interface')
        self.req_sub = self.create_subscription(String, 'input_request', self.request_cb, 10)
        self.resp_pub = self.create_publisher(String, 'response', 10)

    def request_cb(self, msg):
        self.get_logger().info(f'Received request: "{msg.data}"')
        response = ollama.chat( model='llama3.2', messages=[{'role': 'user', 'content':  msg.data}])
        output = String()
        output.data = response['message']['content']
        self.resp_pub.publish( output )

def main(args=None):
    rclpy.init(args=args)
    node = OllamaInterface()
    rclpy.spin(node)
    

if __name__ == '__main__':
    main()
