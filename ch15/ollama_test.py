import ollama 
import requests
from rich import print

def get_current_weather(city):
    base_url = f"http://wttr.in/{city}?format=j1"
    response = requests.get(base_url)
    data = response.json()
    return f"{data['current_condition'][0]['temp_C']}°C"

def do_math(x:int, op:str, y:int):
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

def generic_chat():
    return "Sorry, I can not serve or understand your request. Can be more specific?"

Question = "What is the weather in Naples?"
#Question = "How much is 13 plus 3?"


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
response = ollama.chat(
    model='llama3.2',
    messages=[{'role': 'user', 'content': 
        Question}],

    tools=tools,
)


print("Response: ", response)
tools_calls = response['message']['tool_calls']

# Parse tool name and arguments
tool_name = tools_calls[0]['function']['name']
arguments = tools_calls[0]['function']['arguments']



res = ""
if( tool_name == "get_current_weather"):
    temperature = get_current_weather(arguments["city"])
    res = "The temperature in " + arguments["city"] + " is about " + temperature
elif( tool_name == "do_math"):
    result = do_math( int(arguments['a']), arguments['op'], int(arguments['b']) )
    res = "The result of the requested operation is: " + result

print(res)
