import pytest
import rclpy
from first_pytest.calculator import Calculator

@pytest.fixture(scope='module')
def init_ros():
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def calculator_node(init_ros):
    node = Calculator()
    yield node
    node.destroy_node()

def test_add(calculator_node):
    result = calculator_node.add(2, 3)
    assert result == 5

def test_subtract(calculator_node):
    result = calculator_node.subtract(2, 3)
    assert result == -1

def test_multiply(calculator_node):
    result = calculator_node.multiply(2, 3)
    assert result == 6

def test_divide(calculator_node):

    result = calculator_node.divide(6, 3)
    assert result == 2     