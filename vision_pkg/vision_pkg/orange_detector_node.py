import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2 as cv
from sensor_msgs.msg import Image
import numpy as np

bridge = CvBridge()


class OrangeDetectorNode(Node):

    def __init__(self):
        super().__init__('OrangeDetector')

        self.create_subscription(Image, '/camera_sensor/image_raw', self.image_callback, 10)



    def image_callback(self,msg):
        # Convert to OpenCV image
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')   ### rgb to bgr
        # Now cv_image is a NumPy array with shape (height, width, 3)

        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

        lower_orange = np.array([5, 50, 50])    ### 10, 100, 100
        upper_orange = np.array([30, 255, 255])    ### 25, 255, 255

        mask = cv.inRange(hsv, lower_orange, upper_orange)

        threshold = 500

        if cv.countNonZero(mask) > threshold:
            self.get_logger().info("Something is Orange dude!")


        self.img_show(mask)  ###


    def img_show(self, img):
        cv.imshow("trolley_camera", img)
        cv.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    node = OrangeDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
