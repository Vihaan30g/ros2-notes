
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSLivelinessPolicy, Duration
from std_msgs.msg import Int32
import time

class IntegerPublisher(Node):
    def __init__(self):
        super().__init__('integer_publisher')

        qos_profile = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            deadline=Duration(seconds=1.5),
            liveliness=QoSLivelinessPolicy.AUTOMATIC
        )

        self.publisher_ = self.create_publisher(Int32, 'numbers', qos_profile)
        self.get_logger().info("Publisher node started")

        self.publish_numbers()

    def publish_numbers(self):
        for i in range(1, 11):
            msg = Int32()
            msg.data = i
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: {msg.data}')
            time.sleep(1 if i <= 5 else 2)

        self.get_logger().info("Publishing complete.")

def main(args=None):
    rclpy.init(args=args)
    node = IntegerPublisher()
    rclpy.spin_once(node, timeout_sec=0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
