import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from topic_calc.publisher import CalcRequest
from topic_calc.subscriber import CalcServ
from calc_msgs.msg import CalcMsg
from std_msgs.msg import Int32
import time

@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def pub_node(ros_context):    
    node = CalcRequest()
    yield node
    node.destroy_node()
    
@pytest.fixture
def sub_node(ros_context):
    node = CalcServ()
    yield node
    node.destroy_node()

def test_complex_pub_sub(pub_node, sub_node):

    received_messages = []
    def result_callback(msg):
        received_messages.append(msg.data)

    sub_node.create_subscription(Int32, 'result_topic', result_callback, 10)

    executor = MultiThreadedExecutor()
    executor.add_node(pub_node)
    executor.add_node(sub_node)
    
    pub_node.publish_message(5, 3, 'add')
    msg_len = 1
    while( len(received_messages) != msg_len ):
        executor.spin_once(timeout_sec=0.5)
    assert len(received_messages) > 0
    assert received_messages[0] == 8 
    
    pub_node.publish_message(7, 2, 'subtract')
    msg_len = msg_len+1
    while( len(received_messages) != msg_len ):
        executor.spin_once(timeout_sec=1.0)
    assert received_messages[-1] == 5  
    
    pub_node.publish_message(4, 3, 'multiply')
    msg_len = msg_len+1
    while( len(received_messages) != msg_len ):
        executor.spin_once(timeout_sec=1.0)
    
    assert received_messages[-1] == 12  
    
    pub_node.publish_message(8, 0, 'divide')
    msg_len = msg_len+1
    while( len(received_messages) != msg_len ):
        executor.spin_once(timeout_sec=1.0)
    
    assert received_messages[-1] == 0
    
