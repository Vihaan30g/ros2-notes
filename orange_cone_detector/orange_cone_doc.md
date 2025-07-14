This package detects specifically an orange colored cone, not just any other colored cone.  
___
<br>

We use a pretrained yolo model to detect for cones. I found this model on github (https://github.com/jhan15/traffic_cones_detection/tree/master). To install this model on your system you can follow the steps I have already mentioned in [yolov5 cone detection documentation](yolov5_detector/yolov5_coneDetect_documentation.md). But this model detects cone of any color and design.  
<br>

Now using OpenCV, we separate the regions that contain cones as detected by yolo and process it to detect orange color in it. For this we follow same steps as followed in [OpenCV orange color detection documentation](vision_pkg/opencv_orange_detect.md).  
<br>

Thats how we make sure that the detected cones are orange in color.  
<br>

