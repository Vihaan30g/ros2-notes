This is documentation of how I installed and used a yolov5 model to detect cones. This file shows how I created the ros2 package. The model used here detects cones of different colors and designs.  

## Prerequisites
Before working on this, i had my robot and a camera plugin on it already.  
<br>

## Installing YOLOv5 and Dependencies
#### Clone YOLOv5
```
cd ~
git clone https://github.com/ultralytics/yolov5.git
cd yolov5
```
#### Install Dependencies
```
pip install -r requirements.txt
```
<br>

## Create a New Package
```
ros2 pkg create --build-type ament_python yolov5_detector --dependencies rclpy sensor_msgs cv_bridge std_msgs
```
<br>

## Create the node
Naming the node file as 'OrangeConeDetectorNode.py'  <br>
**Node content :**
```
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
```
Modify the required code according to need. Mostly you will need to just put the correct path for your training model.  
<br>

## Download the model
Here i have used a model trained for detecting cones from github repo : (https://github.com/jhan15/traffic_cones_detection/tree/master)  
model files are very large, so we use git-LFS.  
<br>
#### Install Git LFS
```
sudo apt update
sudo apt install git-lfs
```
#### Clone the repository with model
```
cd ~
git lfs install
git clone https://github.com/jhan15/traffic_cones_detection.git
cd traffic_cones_detection
```
#### Move the model into your ROS 2 package
Create a new folder 'models' inside the pkg.  
```
cp best.pt ~/ros2_ws/src/yolov5_detector/models/
```
<br>

## Add the node to entry_points in setup.py
```
'OrangeCone_node = yolov5_detector.OrangeConeDetectorNode:main',
```
<br>

## Building the workspace (Caution)
YOLOv5 uses a newer version of setuptools, on the other hand ros2 humble uses an older version. Now when you installed the dependencies for yolo, it automatically upgrades the stuptools. This will break ros. So don't build the workspace right away.
Here we got 2 options : 
**1. Downgrade setuptools before building the workspace.**
```
 pip uninstall setuptools -y
 pip install setuptools==58.1.0
```
And now run everything(gazebo, yolo node, etc).  
YOLOv5 needs setuptools at runtime, so when we run the yolo node, it upgrades. And after you are done with simulation and everything, again downgrade setuptools for future ros use.

**2.  Use a Python Virtual Environment for YOLO.**

 
