import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ollama 
import requests

tools = [
    {
      'type': 'function',
      'function': {
        'name': 'get_current_weather',
        'description': 'Get the current weather for a city',
        'parameters': {
          'type': 'object',
          'properties': {
            'city': {
              'type': 'string',
              'description': 'The name of the city',
            },
          },
          'required': ['city'],
        },
      },
    },
    {
        "type": "function",
        "function": {
            "name": "do_math",
            "description": "Do basic math operations",
            "parameters": {
                "type": "object",
                "properties": {
                    "a": {
                        "type": "int",
                        "description": "The first operand"
                    },
                    "op": {
                        "type": "str",
                        "description": "The operation to perform (one of '+', '-', '*', '/')"
                    },
                    "b": {
                        "type": "int",
                        "description": "The second operand"
                    }
                },
                "required": [
                    "a",
                    "op",
                    "b"
                ]
            }
        }
    },
    {
      'type': 'function',
      'function': {
        'name': 'generic_chat',
        'description': 'chat',
        'parameters': {},
      },
    },
]

class OllamaInterface(Node):
    def __init__(self):
        super().__init__('ollama_interface')

        self.req_sub = self.create_subscription(String, 'input_request', self.request_cb, 10)
        self.resp_pub = self.create_publisher(String, 'response', 10)

    def get_current_weather(self, city):
        base_url = f"http://wttr.in/{city}?format=j1"
        response = requests.get(base_url)
        data = response.json()
        return f"{data['current_condition'][0]['temp_C']}°C"

    def do_math(self, x:int, op:str, y:int):
        res = "0"
        if op == "+":
            res = str(int(x) + int(y))
        elif op == "-":
            res = str(int(x) - int(y))
        elif op == "*":
            res = str(int(x) * int(y))
        elif op == "/":
            if int(y) != 0: 
                res = str(int(x) / int(y))
        return res

    def generic_chat(self):
        return "Sorry, I can not serve or understand your request. Can be more specific?"




    def request_cb(self, msg):
        # Log the received message
        self.get_logger().info(f'Received request: "{msg.data}"')
        response = ollama.chat( model='llama3.2', messages=[{'role': 'user', 'content':  msg.data}],
            tools=tools,
        )
        
        res = ""

        print("response: ", response)

        if ('message' in response and
            'tool_calls' in response['message'] and
            len(response['message']['tool_calls']) > 0 and
            'function' in response['message']['tool_calls'][0] and
            'name' in response['message']['tool_calls'][0]['function'] and
            'arguments' in response['message']['tool_calls'][0]['function']):
            
            # Parse tool name and arguments
            tools_calls = response['message']['tool_calls']
            tool_name = tools_calls[0]['function']['name']
            arguments = tools_calls[0]['function']['arguments']
            
                
            if( tool_name == "get_current_weather"):
                temperature = self.get_current_weather(arguments["city"])
                res = "The temperature in " + arguments["city"] + " is about " + temperature
            elif( tool_name == "do_math"):
                result = self.do_math( int(arguments['x']), arguments['op'], int(arguments['y']) )
                res = "The result of the requested operation is: " + result

        else:
            res = self.generic_chat()

        output = String()
        output.data = res
        self.resp_pub.publish( output )



def main(args=None):
    rclpy.init(args=args)
    node = OllamaInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
