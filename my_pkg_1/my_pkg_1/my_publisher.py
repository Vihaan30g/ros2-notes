import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import random
import time

class random_num(Node):
    def __init__(self):
        super().__init__('rand_num_gen')
        self.publisher = self.create_publisher(Int32,'my_topic_1',5)
        

    
def main(args=None):
    rclpy.init(args=args)
    node = random_num()
    n = random.randint(1, 100)
    msg = Int32()
    msg.data = n
    time.sleep(1)
    node.publisher.publish(msg)
    node.get_logger().info(f'Random Number generated : {n}')
    node.destroy_node()
    rclpy.shutdown()