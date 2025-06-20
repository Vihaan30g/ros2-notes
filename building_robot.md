
# Building the robot (brief overview)  
*(This is a very brief overview of the process i followed to build the robot.)*
<br>
<br>

**Create a workspace and src folder in it.**
<br><br>

**Create an ament_cmake pkg, inside src/. Name it robot_description.**
<br><br>

**Inside the pkg, create a folder named urdf and store all the .urdf/.xacro files in here.**
<br><br>

**Launch file for robot_description pkg.**  
Create another folder named launch inside this pkg and write the launch file in this folder. This launch file will be to run robot_state_publisher and joint_state_publisher_gui and rviz2.  So that while building robot structure, we can visualize it in rviz.
<br><br>

**In CMakeLists.txt, install urdf and launch folders. i.e. add below code:**  
```
install(
	DIRECTORY urdf launch
	DESTINATION share/${PROJECT_NAME}/
)  
```
<br>

**Build the workspace.**   
Use symlink flag, this will create symbolic links instead of copying files into the install/ directory. This results in faster builds and instant updates.  
`colcon build --symlink-install`  
___
<br><Br>


**Creating pkg to launch gazebo and spawn the robot into it.**  
Create a ament_cmake pkg in same workspace, name it robot_bringup. Create a launch folder in it. This launch file would run robot_state_publisher, launch gazebo, spawn our robot and may also run rviz.
<br>
Save rviz configuration and gazebo world in 2 separate folders and install them in CMakeLists.txt. Add necessary flags in launch file to run rviz and gazebo with the saved configurations.
<br><br>

**Add necessary dependencies in package.xml:**  
```
<exec_depend>trolley_description</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>gazebo_ros</exec_depend>
```
<br>

**Build the workspace.**  
___
<br><br>

**Creating an Obstacle-stop node.**   
This node will stop the robot if an obstacle comes closer than 0.5 m to the robot from front.
<br>

**Create a new pkg named obstacle_pkg with ament_python build.**
write a node that subscribes to /scan, to get distance b/w robot and obstacles, and to /cmd_vel, to know the velocity of the robot. It should publish 0 velocity to /cmd_vel when required.
<br><br>


**Add dependencies to package.xml :**  
```
<depend>rclpy</depend>
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>
```
<br>

**Enter the node in setup.py**
<br><br>

**Build the workspace.**  
___










