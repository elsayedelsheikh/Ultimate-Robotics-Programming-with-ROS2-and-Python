import rclpy
from rclpy.node import Node

class Calculator(Node):

    def __init__(self):
        super().__init__('calculator')

    def add(self, a, b):
        return a + b
    def subtract(self, a, b):
        return a - b    
    def multiply(self, a, b):
        return a * b
    def divide(self, a, b):
        return a % b

def main(args=None):
    rclpy.init(args=args)
    calculator = Calculator()
    rclpy.spin(calculator)
    calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()