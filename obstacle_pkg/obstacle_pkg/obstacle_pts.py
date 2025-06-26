
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class ObstaclePointsSubscriber(Node):
    def __init__(self):
        super().__init__('obstacle_points_subscriber')

        qos2 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(Float32MultiArray, 'ObstaclePts', self.callback, qos2)

    def callback(self, msg: Float32MultiArray):
        data = msg.data

        # Format all floats with 3 decimal places and join with 4 spaces
        formatted_data = '    '.join(f"{val:.3f}" for val in data)

        # Display the whole line
        self.get_logger().info(formatted_data)
        self.get_logger().info("\n\n New Entry \n \n")



def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePointsSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class ObstaclePointsSubscriber(Node):

    def __init__(self):
        super().__init__('obstacle_points_subscriber')

        qos2 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(Float32MultiArray, 'ObstaclePts', self.callback, qos2)

    def callback(self, msg: Float32MultiArray):
        if len(msg.data) == 3:
            x, y, z = msg.data
            self.get_logger().info(f"x = {x:.3f}")
            self.get_logger().info(f"y = {y:.3f}")
            self.get_logger().info(f"z = {z:.3f}")
        else:
            self.get_logger().warn("Unexpected number of values in ObstaclePts")


def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePointsSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()