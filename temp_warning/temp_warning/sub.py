import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class TempMonitorNode(Node):
    def __init__(self):
        super().__init__('temp_monitor')
        self.declare_parameter('threshold', 75.0)
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value
        self.subscription = self.create_subscription(
            Float64, 'topic_1', self.callback, 10)
        self.get_logger().info(f'Monitoring started. Warning threshold: {self.threshold} deg')

    def callback(self, msg):
        temp = msg.data
        if temp > self.threshold:
            self.get_logger().warn(f'High temperature! {temp} deg exceeds threshold.')
        else:
            self.get_logger().info(f'Temperature OK: {temp} deg')

def main(args=None):
    rclpy.init(args=args)
    node = TempMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
