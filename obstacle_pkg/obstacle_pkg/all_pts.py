
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import os

class ObstaclePointsSubscriber(Node):

    def __init__(self):
        super().__init__('obstacle_points_subscriber')

        qos2 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'points_coord',
            self.callback,
            qos2
        )

        # Output file path
        self.file_path = os.path.join(os.path.expanduser("~"), "all_points_log.txt")

        # Clear the file initially
        with open(self.file_path, 'w') as f:
            f.write("# Logged obstacle coordinates (x, y, z)\n")

        self.get_logger().info(f"Logging obstacle points to: {self.file_path}")

    def callback(self, msg: Float32MultiArray):
        data = msg.data
        with open(self.file_path, 'a') as f:
            for i in range(0, len(data), 3):
                if i + 2 < len(data):
                    x, y, z = data[i], data[i+1], data[i+2]
                    f.write(f"x = {x:.3f}, y = {y:.3f}, z = {z:.3f}\n")
                else:
                    remaining = data[i:]
                    f.write(f"Incomplete coordinate triplet: {remaining}\n")

def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePointsSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()