import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import cv2
import numpy as np

import matplotlib
matplotlib.use('TkAgg')  # or 'Qt5Agg' if using PyQt5
import matplotlib.pyplot as plt
import threading
import time

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
        self.model = torch.hub.load(
            'ultralytics/yolov5',
            'custom',
            path='/home/vihaan/Desktop/ws_trolley/src/yolov5_detector/models/best.pt',
            force_reload=False, trust_repo=True)
        self.model.conf = 0.4

        # Shared image data for visualization
        self.original_image = None
        self.detected_image = None
        self.orange_mask = None
        self.lock = threading.Lock()

        # Start the matplotlib thread
        self.vis_thread = threading.Thread(target=self.visualize_loop, daemon=True)
        self.vis_thread.start()

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        original = cv_image.copy()
        results = self.model(cv_image)
        detections = results.pandas().xyxy[0]
        results.render()
        detected = results.ims[0]

        orange_mask = None
        for _, row in detections.iterrows():
            if row['name'] == 'cone':
                xmin = int(row['xmin'])
                ymin = int(row['ymin'])
                xmax = int(row['xmax'])
                ymax = int(row['ymax'])

                cone_crop = cv_image[ymin:ymax, xmin:xmax]
                hsv = cv2.cvtColor(cone_crop, cv2.COLOR_BGR2HSV)
                lower_orange = np.array([5, 50, 50])
                upper_orange = np.array([30, 255, 255])
                mask = cv2.inRange(hsv, lower_orange, upper_orange)

                if cv2.countNonZero(mask) > 500:
                    self.get_logger().info("Orange cone detected!")

                orange_mask = mask  # overwrite for latest cone

        # Convert for matplotlib
        with self.lock:
            self.original_image = cv2.cvtColor(original, cv2.COLOR_BGR2RGB)
            self.detected_image = cv2.cvtColor(detected, cv2.COLOR_BGR2RGB)
            self.orange_mask = orange_mask

    def visualize_loop(self):
        while True:
            with self.lock:
                orig = self.original_image
                det = self.detected_image
                mask = self.orange_mask

            if orig is not None and det is not None:
                orig_disp = cv2.cvtColor(orig, cv2.COLOR_RGB2BGR)
                det_disp = cv2.cvtColor(det, cv2.COLOR_RGB2BGR)

                if mask is not None:
                    mask_disp = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                else:
                    mask_disp = np.zeros_like(orig_disp)

                combined = np.hstack([
                    cv2.resize(orig_disp, (320, 240)),
                    cv2.resize(det_disp, (320, 240)),
                    cv2.resize(mask_disp, (320, 240))
                ])
                cv2.imshow("Cone Detection: Original | YOLO | Orange Mask", combined)
                cv2.waitKey(1)

            time.sleep(0.05)



def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    plt.close('all')
    cv2.destroyAllWindows()






'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import cv2
import numpy as np

import matplotlib
matplotlib.use('TkAgg')  # or 'Qt5Agg' if using PyQt5
import matplotlib.pyplot as plt
import threading
import time

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
        self.model = torch.hub.load(
            'ultralytics/yolov5',
            'custom',
            path='/home/vihaan/Desktop/ws_trolley/src/yolov5_detector/models/best.pt',
            force_reload=False, trust_repo=True)
        self.model.conf = 0.4

        # Shared image data for visualization
        self.original_image = None
        self.detected_image = None
        self.orange_mask = None
        self.lock = threading.Lock()

        # Start the matplotlib thread
        self.vis_thread = threading.Thread(target=self.visualize_loop, daemon=True)
        self.vis_thread.start()

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        original = cv_image.copy()
        results = self.model(cv_image)
        detections = results.pandas().xyxy[0]
        results.render()
        detected = results.ims[0]

        orange_mask = None
        for _, row in detections.iterrows():
            if row['name'] == 'cone':
                xmin = int(row['xmin'])
                ymin = int(row['ymin'])
                xmax = int(row['xmax'])
                ymax = int(row['ymax'])

                cone_crop = cv_image[ymin:ymax, xmin:xmax]
                hsv = cv2.cvtColor(cone_crop, cv2.COLOR_BGR2HSV)
                lower_orange = np.array([5, 50, 50])
                upper_orange = np.array([30, 255, 255])
                mask = cv2.inRange(hsv, lower_orange, upper_orange)

                if cv2.countNonZero(mask) > 500:
                    self.get_logger().info("Orange cone detected!")

                orange_mask = mask  # overwrite for latest cone

        # Convert for matplotlib
        with self.lock:
            self.original_image = cv2.cvtColor(original, cv2.COLOR_BGR2RGB)
            self.detected_image = cv2.cvtColor(detected, cv2.COLOR_BGR2RGB)
            self.orange_mask = orange_mask

    def visualize_loop(self):
        plt.ion()
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        fig.canvas.manager.set_window_title('Cone Detection Visualization')

        # Pre-create images to avoid flicker
        im0 = axes[0].imshow(np.zeros((100, 100, 3), dtype=np.uint8))
        axes[0].set_title("Original")
        axes[0].axis('off')

        im1 = axes[1].imshow(np.zeros((100, 100, 3), dtype=np.uint8))
        axes[1].set_title("YOLO Detection")
        axes[1].axis('off')

        im2 = axes[2].imshow(np.zeros((100, 100), dtype=np.uint8), cmap='gray')
        axes[2].set_title("Orange Mask")
        axes[2].axis('off')

        fig.tight_layout()
        fig.show()

        while True:
            with self.lock:
                orig = self.original_image
                det = self.detected_image
                mask = self.orange_mask

            if orig is not None and det is not None:
                im0.set_data(orig)
                im1.set_data(det)
                if mask is not None:
                    im2.set_data(mask)
                else:
                    im2.set_data(np.zeros_like(im2.get_array()))

                fig.canvas.draw_idle()
                fig.canvas.flush_events()
                plt.pause(0.001)

            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    plt.close('all')
    cv2.destroyAllWindows()

'''



'''
# WORKING BUT NO DISPLAY< TERMINAL MESSAGES VISIBLE


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import cv2
import numpy as np

import matplotlib
matplotlib.use('TkAgg')  # or 'Qt5Agg' if using PyQt5
import matplotlib.pyplot as plt
import threading
import time

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
        self.model = torch.hub.load(
            'ultralytics/yolov5',
            'custom',
            path='/home/vihaan/Desktop/ws_trolley/src/yolov5_detector/models/best.pt',
            force_reload=False, trust_repo=True)
        self.model.conf = 0.4

        # Shared image data for visualization
        self.original_image = None
        self.detected_image = None
        self.orange_mask = None
        self.lock = threading.Lock()

        # Start the matplotlib thread
        self.vis_thread = threading.Thread(target=self.visualize_loop, daemon=True)
        self.vis_thread.start()

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        original = cv_image.copy()
        results = self.model(cv_image)
        detections = results.pandas().xyxy[0]
        results.render()
        detected = results.ims[0]

        orange_mask = None
        for _, row in detections.iterrows():
            if row['name'] == 'cone':
                xmin = int(row['xmin'])
                ymin = int(row['ymin'])
                xmax = int(row['xmax'])
                ymax = int(row['ymax'])

                cone_crop = cv_image[ymin:ymax, xmin:xmax]
                hsv = cv2.cvtColor(cone_crop, cv2.COLOR_BGR2HSV)
                lower_orange = np.array([5, 50, 50])
                upper_orange = np.array([30, 255, 255])
                mask = cv2.inRange(hsv, lower_orange, upper_orange)

                if cv2.countNonZero(mask) > 500:
                    self.get_logger().info("Orange cone detected!")

                orange_mask = mask  # overwrite for latest cone

        # Convert for matplotlib
        with self.lock:
            self.original_image = cv2.cvtColor(original, cv2.COLOR_BGR2RGB)
            self.detected_image = cv2.cvtColor(detected, cv2.COLOR_BGR2RGB)
            self.orange_mask = orange_mask

    def visualize_loop(self):
        plt.ion()
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        fig.canvas.manager.set_window_title('Cone Detection Visualization')

        while True:
            with self.lock:
                orig = self.original_image
                det = self.detected_image
                mask = self.orange_mask

            if orig is not None and det is not None:
                axes[0].clear()
                axes[0].imshow(orig)
                axes[0].set_title("Original")
                axes[0].axis('off')

                axes[1].clear()
                axes[1].imshow(det)
                axes[1].set_title("YOLO Detection")
                axes[1].axis('off')

                axes[2].clear()
                if mask is not None:
                    axes[2].imshow(mask, cmap='gray')
                    axes[2].set_title("Orange Mask")
                else:
                    axes[2].text(0.5, 0.5, 'No Cone', ha='center', va='center')
                    axes[2].set_title("Orange Mask")
                axes[2].axis('off')

                plt.tight_layout()
                plt.pause(0.001)

            time.sleep(0.05)  # to avoid 100% CPU usage

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    plt.close('all')


'''







'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import cv2
import numpy as np

import matplotlib
matplotlib.use('TkAgg')  # or 'Qt5Agg' if using PyQt5
import matplotlib.pyplot as plt
import threading
import time

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
        self.model = torch.hub.load(
            'ultralytics/yolov5',
            'custom',
            path='/home/vihaan/Desktop/ws_trolley/src/yolov5_detector/models/best.pt',
            force_reload=False, trust_repo=True)
        self.model.conf = 0.4

        # Shared image data for visualization
        self.original_image = None
        self.detected_image = None
        self.orange_mask = None
        self.lock = threading.Lock()

        # Start the matplotlib thread
        self.vis_thread = threading.Thread(target=self.visualize_loop, daemon=True)
        self.vis_thread.start()

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        original = cv_image.copy()
                # Run YOLO detection
        results = self.model(cv_image)
        detections = results.pandas().xyxy[0]  # DataFrame

        if detections.empty:
            self.get_logger().info("No objects detected.")
        else:
            self.get_logger().info(f"Detected objects:\n{detections}")


    def visualize_loop(self):
        plt.ion()
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        fig.canvas.manager.set_window_title('Cone Detection Visualization')

        while True:
            with self.lock:
                orig = self.original_image
                det = self.detected_image
                mask = self.orange_mask

            if orig is not None and det is not None:
                axes[0].clear()
                axes[0].imshow(orig)
                axes[0].set_title("Original")
                axes[0].axis('off')

                axes[1].clear()
                axes[1].imshow(det)
                axes[1].set_title("YOLO Detection")
                axes[1].axis('off')

                axes[2].clear()
                if mask is not None:
                    axes[2].imshow(mask, cmap='gray')
                    axes[2].set_title("Orange Mask")
                else:
                    axes[2].text(0.5, 0.5, 'No Cone', ha='center', va='center')
                    axes[2].set_title("Orange Mask")
                axes[2].axis('off')

                plt.tight_layout()
                plt.pause(0.001)

            time.sleep(0.05)  # to avoid 100% CPU usage

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    plt.close('all')

'''




'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import cv2
import numpy as np
import matplotlib.pyplot as plt

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
        self.model = torch.hub.load(
            'ultralytics/yolov5', 
            'custom', 
            path='/home/vihaan/Desktop/ws_trolley/src/yolov5_detector/models/best.pt',
            force_reload=False, trust_repo=True)
        self.model.conf = 0.4  # confidence threshold

        # Set up matplotlib for interactive mode
        plt.ion()
        self.fig, self.axes = plt.subplots(1, 3, figsize=(15, 5))
        self.fig.canvas.manager.set_window_title('YOLO Cone Detection Visualization')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # Save the original image for subplot
        original_image = cv_image.copy()

        # Run YOLO detection
        results = self.model(cv_image)
        detections = results.pandas().xyxy[0]  # DataFrame
        results.render()
        detected_image = results.ims[0]  # Annotated image with boxes (in BGR)

        orange_mask = None
        cone_detected = False

        # Process each detected cone
        for _, row in detections.iterrows():
            if row['name'] == 'orange_cone':
                xmin, ymin = int(row['xmin']), int(row['ymin'])
                xmax, ymax = int(row['xmax']), int(row['ymax'])

                cone_crop = cv_image[ymin:ymax, xmin:xmax]
                hsv = cv2.cvtColor(cone_crop, cv2.COLOR_BGR2HSV)
                lower_orange = np.array([5, 50, 50])
                upper_orange = np.array([30, 255, 255])
                mask = cv2.inRange(hsv, lower_orange, upper_orange)

                if cv2.countNonZero(mask) > 500:
                    self.get_logger().info("Orange cone detected!")
                    cone_detected = True

                # Show only the last processed cone mask
                orange_mask = mask

        # Convert images from BGR to RGB for matplotlib
        original_rgb = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)
        detected_rgb = cv2.cvtColor(detected_image, cv2.COLOR_BGR2RGB)

        # Display the 3 images using matplotlib
        self.axes[0].clear()
        self.axes[0].imshow(original_rgb)
        self.axes[0].set_title("Original Image")
        self.axes[0].axis('off')

        self.axes[1].clear()
        self.axes[1].imshow(detected_rgb)
        self.axes[1].set_title("YOLO Detection")
        self.axes[1].axis('off')

        self.axes[2].clear()
        if orange_mask is not None:
            self.axes[2].imshow(orange_mask, cmap='gray')
            self.axes[2].set_title("Orange Mask")
        else:
            self.axes[2].text(0.5, 0.5, 'No Cone Detected', ha='center', va='center')
            self.axes[2].set_title("Orange Mask")
        self.axes[2].axis('off')

        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    plt.close('all')

'''

'''

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
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/vihaan/Desktop/ws_trolley/src/yolov5_detector/models/best.pt', force_reload=False, trust_repo=True)
        self.model.conf = 0.4  # confidence threshold

        # Create a window for live display
        cv2.namedWindow("Live Feed", cv2.WINDOW_NORMAL)


    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # Run detection
        results = self.model(cv_image)
        detections = results.pandas().xyxy[0]  # DataFrame with detection info

        cone_images = []  # List to store cropped images

        for index, row in detections.iterrows():
            if row['name'] == 'orange_cone':
                xmin = int(row['xmin'])
                ymin = int(row['ymin'])
                xmax = int(row['xmax'])
                ymax = int(row['ymax'])
                # Crop the cone
                cone_crop = cv_image[ymin:ymax, xmin:xmax]
                cone_images.append(cone_crop)

        # Render detection boxes on the original image
        results.render()
        # Display the image with detections
        cv2.imshow("Live Feed", results.ims[0])

        # Detect orange cones in cropped images
        self.orange_detection(cone_images)    


    def orange_detection(self, imgs):
        for img in imgs:
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            lower_orange = np.array([5, 50, 50])
            upper_orange = np.array([30, 255, 255])
            mask = cv2.inRange(hsv, lower_orange, upper_orange)

            threshold = 500
            if cv2.countNonZero(mask) > threshold:
                self.get_logger().info("Orange cone detected!")

            self.img_show(img)  # replaced "img" with "mask" . previously it was mask


    def img_show(self, img):
        cv2.imshow("Orange Mask", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

'''


'''
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
        detections = results.pandas().xyxy[0]  # DataFrame with detection info

        cone_images = []  # Initialize list to store cropped images
        
        for index, row in detections.iterrows():
            if row['name'] == 'orange_cone':  # Or use 'class' if class id is known
                xmin = int(row['xmin'])
                ymin = int(row['ymin'])
                xmax = int(row['xmax'])
                ymax = int(row['ymax'])
                # Crop the cone
                cone_crop = cv_image[ymin:ymax, xmin:xmax]
                cone_images.append(cone_crop)


        # Show the image with boxes
        results.render()
        cv2.imshow("Detections", results.ims[0])
        cv2.waitKey(1)

        self.orange_detection(self,cone_images)
    



    def orange_detection(self,img):

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_orange = np.array([5, 50, 50])    ### 10, 100, 100
        upper_orange = np.array([30, 255, 255])    ### 25, 255, 255

        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        threshold = 500

        if cv2.countNonZero(mask) > threshold:
            self.get_logger().info("Orange cone detected daa !!!")


        self.img_show(mask)  ###


    def img_show(self, img):
        cv2.imshow("trolley_camera", img)
        cv2.waitKey(1)
                


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
'''