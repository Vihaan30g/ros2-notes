## Description
### You will find 3 new packages created for this task :  
1. **vision_pkg** - Detects orange colored objects using opencv.  
2. **yolov5_detector** - Detects a cone using yolov5 model.  
3. **orange_cone_detector** - Detects an orange colored cone.
<br>

### Other old packages are :   
1. **bringup** - This launches all the system(gazebo, rviz & other nodes)  
2. **trolley_description** - Contains urdf for the robot  
3. **obstacle_pkg** - Contains nodes to avoid obstacles
<br>

The launch file will run orange_cone_node and obstacle node made to work with 3d lidar. You can easily comment out other nodes in launch file to check them.  
<br>
<br>

## Difficulties faced with different methods of detection  
1. **Using OpenCV**  
    This wasn't really difficult overall, but still the hardest part was to decide the upper and lower ranges for masking the image.  

2. **Using yolo**  
    This had several issues :  
    <br>
    First of all yolov5 requires newer version of setup_tools to work, whereas ros2 uses older version. Now to make it work, before building the workspace, I had to make sure that the setup_tools is at older version. When I ran yolo model it automatically upgraded setup_tools to newer compatible version. So I had to make sure that after every run I downgrade the setup_tools, else ros2 won't be able to build any other pkg.  
    <br>
    Other problem was that we need data to train yolo model. Finding a pretained model for your specific purpose hard. I couldn't find a model that would detect orange cone specifically, so i had to use opencv on top of yolo to make sure that the color of detected cone by yolo is orange or has significant orange color.  
    <br>
    Yolo was quite heavy, it made the whole project slow in runtime. Especially gazebo went mad, it just couldn't respond.  
    <br>
    Visualizing the processed images at every stage was difficult. You have to properly terminate old windows of images whether it be using img_show or matplotlib, because we are processing many images in very short period. You may have to run this node in a separate terminal if it doesn't work.  
    <br>



