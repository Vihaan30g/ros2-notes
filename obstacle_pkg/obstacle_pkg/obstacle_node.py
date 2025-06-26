# This node publishes 0 velocity persistently until the robot stops and is within 0.5 m of range of obstacle.

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleStopNode(Node):
    def __init__(self):
        super().__init__('obstacle_stop_node')
        
        self.t = 0.5  # Minimum safe distance
        self.moving = False
	

        qos1 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        qos2 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
	
	
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos1)

        self.create_subscription(Twist, '/cmd_vel', self.velocity_callback, qos2)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos2)


    def velocity_callback(self, msg):
        if msg.linear.x != 0.0 or msg.linear.y != 0.0 or msg.linear.z != 0.0:
            self.moving = True
        else:
            self.moving = False

    def lidar_callback(self, msg):
        if self.moving:                     # we send stop commands only if the robot is moving within 0.5 of obstacle.
            for dist in msg.ranges:
                if dist <= self.t:
                    self.get_logger().info('Obstacle too close! Stopping robot.')
                    self.stop_robot()
                    break

    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_pub.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

	
