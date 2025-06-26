import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import struct
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class ObstacleStopNode(Node):
    def __init__(self):
        super().__init__('obstacle_stop_node_2')

        self.t = 0.5  # Minimum safe distance
        self.moving = False
        self.lidar_z = 0.415   # w.r.t. base link


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
	

        self.subscription = self.create_subscription(PointCloud2,'/velodyne_points',self.reform_raw_data, qos1)

        self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.velocity_callback, qos2)
        
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', qos2)

        self.obstacle_pts_pub = self.create_publisher(Float32MultiArray, 'ObstaclePts', qos2)

        self.point_cloud_pub = self.create_publisher(Float32MultiArray, 'points_coord', qos2)
        


    def velocity_callback(self, msg):
        if msg.linear.x != 0.0 or msg.linear.y != 0.0 or msg.linear.z != 0.0:
            self.moving = True
        else:
            self.moving = False




    def reform_raw_data(self, msg):
        # Number of points in this message
        ttl_pts = msg.width * msg.height

        # Point structure:
        # Each point is 16 bytes: [x (4), y (4), z (4), intensity (4)]
        # We care about only the first 12 bytes of each point (x, y, z) , ignoring intensity field
        point_step = msg.point_step  # usually 16
        raw_data = msg.data  # raw bytes

        # Create empty numpy array to hold all (x, y, z) rows
        points = np.empty((ttl_pts, 3), dtype=np.float32)



        for i in range(ttl_pts):
            start = i * point_step

            # Extract bytes for x, y, z (4 bytes each)
            x = raw_data[start : start + 4]
            y = raw_data[start + 4 : start + 8]
            z = raw_data[start + 8 : start + 12]

            # Convert to float32 using struct.unpack
            x = struct.unpack('<f', x)[0]
            y = struct.unpack('<f', y)[0]
            z = struct.unpack('<f', z)[0]

            # Store in the array
            points[i] = [x, y, z]


        # To publish all points catched by lidar
        flat_points = points.flatten().tolist()
        msg_out = Float32MultiArray()
        msg_out.data = flat_points
        self.point_cloud_pub.publish(msg_out)
        
            

        # You now have a full 2D array: shape = (ttl_pts, 3)
        self.obstacle_check(points)








    def obstacle_check(self, points: np.ndarray ):

        if self.moving == True:
            for i in points:
                if i[2] > -(self.lidar_z+0.05) and abs(i[0]) <= 0.5:     #  lidar_z = 0.415, Lidar height = 0.515 from ground
                    
                    self.get_logger().info('Obstacle too close! Stopping robot.')
                    self.stop_robot()

                    # Obstacle Points

                    # publishing only y coordinate
                    '''
                    flat_points = points.flatten().tolist()
                    msg_out = Float32MultiArray()
                    msg_out.data = flat_points
                    self.obstacle_pts_pub.publish(msg_out)
                    '''

                    # Publishing only z coordinate
                    '''
                    msg = Float32MultiArray()
                    msg.data = [float(i[2])]
                    self.obstacle_pts_pub.publish(msg)
                    '''

                     # Publish all 3 coordinates: x, y, z
                    msg = Float32MultiArray()
                    msg.data = [float(i[0]), float(i[1]), float(i[2])]
                    self.obstacle_pts_pub.publish(msg)

                    break




    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.vel_pub.publish(stop_msg)




def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()