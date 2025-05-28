# week_qos/subscriber_node.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSLivelinessPolicy, Duration
from std_msgs.msg import Int32

class IntegerSubscriber(Node):
    def __init__(self):
        super().__init__('integer_subscriber')

        qos_profile = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            deadline=Duration(seconds=1.5),
            liveliness=QoSLivelinessPolicy.AUTOMATIC
        )

        self.subscription = self.create_subscription(
            Int32,
            'numbers',
            self.listener_callback,
            qos_profile
        )

    def listener_callback(self, msg):
        squared = msg.data ** 2
        self.get_logger().info(f'Received: {msg.data}, Squared: {squared}')

def main(args=None):
    rclpy.init(args=args)
    node = IntegerSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
