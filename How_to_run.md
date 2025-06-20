This file explains how to visualize the robot on gazebo classic on your system.  
<br><br>  

**1. Clone the repository**   
Create a workspace and an src folder within it on your system.  
In src/, run this command : `git clone https://github.com/Vihaan30g/ros2-notes.git .`
<br><br>  


**2. Install any missing dependency**  
run this command : `rosdep install --from-paths src --ignore-src -r -y`
<br><br>  


**3. Build the workspace and source it**  
run following commands within workspace directory :  
`colcon build --symlink-install`
`source install/setup.bash`   
<br><br>


**4. Run the launch file**  
run : `ros2 launch bringup trolley_gazebo.launch.xml`  
<br><br>  


**5. Run teleop pkg to control the robot**  
Run this command in a separate terminal : `ros2 run teleop_twist_keyboard teleop_twist_keyboard`  
If not having teleop key pkg, install it : `sudo apt install ros-humble-teleop-twist-keyboard`  
<br><br>  

___


