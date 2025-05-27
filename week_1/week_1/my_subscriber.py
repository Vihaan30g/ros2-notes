
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class square_calc(Node):
    def __init__(self):
        super().__init__('square_calc')
        self.subscription = self.create_subscription(Int32, 'my_topic_1', self.callback, 5)
    
    def callback(self, msg):
        n = msg.data
        self.get_logger().info(f'Number recieved : {n} , its square : {n*n}')

def main(args=None):
    rclpy.init(args=args)
    node = square_calc()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()