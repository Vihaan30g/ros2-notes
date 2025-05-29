import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random
from rclpy.qos import QoSProfile

class temp_generator(Node):
    def __init__(self):
        super().__init__('temp_generator')

        qos_profile = QoSProfile(depth=10)
        
        self.publisher = self.create_publisher(Float64, 'topic_1', qos_profile)
        
        self.get_logger().info("Starting to generate temperatures : \n")
        self.timer = self.create_timer(1,self.log_temps)


    def log_temps(self):
        t = float(random.randint(0,100))
        msg = Float64()
        msg.data = t
        self.publisher.publish(msg)
        self.get_logger().info(f'Generated temp : {t} deg')

def main(args = None):
    rclpy.init(args=args)
    my_node = temp_generator()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
