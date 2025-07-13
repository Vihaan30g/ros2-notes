import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import cv2
import numpy as np

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            self.listener_callback,
            10)
        
        # Load YOLOv5 model
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/vihaan/Desktop/ws_trolley/src/yolov5_detector/models/best.pt')  # path to your trained model
        self.model.conf = 0.4  # confidence threshold

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        results = self.model(cv_image)
        detections = results.pandas().xyxy[0]  # x1, y1, x2, y2, conf, class, name

        for i, row in detections.iterrows():
            if row['name'] == 'orange_cone':  # Replace with your label name
                self.get_logger().info(f"Detected orange cone with confidence {row['confidence']:.2f}")

        # Optional: Show the image with boxes
        results.render()
        cv2.imshow("Detections", results.ims[0])
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
