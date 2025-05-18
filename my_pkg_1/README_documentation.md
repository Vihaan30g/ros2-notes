
# **What is ROS?**
    • Robot Operating System
    • it’s a middleware framework that helps you develop software for robots.
ROS provides: Tools, Libraries, Communication infrastructure.
    • All of this helps different robot components (like sensors, motors, cameras, etc.) talk to each other in a structured way.

# **Key Components of ROS:**
## **1. Nodes**
    • A node is a process that performs some computation (e.g., reading sensor data, controlling motors).
    • Nodes are the smallest executable units.
    • You can think of a node as a "program" or "module" running in your robot system.
## **2. Topics**
    • Nodes communicate using topics.
    • Topics are communication channels used to publish and subscribe messages.
        ◦ A publisher sends messages to a topic.
        ◦ A subscriber listens for messages on a topic.
    • Example: A camera_node might publish images to a topic called /camera/image_raw, and another node can subscribe to it to process images.
## **3. Messages**
    • Messages are the data format sent over topics.
    • Examples of message types:
        ◦ std_msgs/String – just a string
        ◦ geometry_msgs/Twist – velocity commands (used in mobile robots)
        ◦ sensor_msgs/LaserScan – data from LIDAR
## **4. Services**
    • Unlike topics, services are synchronous communication.
    • A service has a request and a response.
    • Example: "Take a photo and return it" — the client asks and the server replies.
    • Used when you need a one-time interaction (e.g., change a setting, request a sensor reading).
## **5. Actions  **
Actions in ROS 2 are used for long-running tasks where the client can send a goal, receive ongoing feedback, and get a final result.
### **ROS 2  Action Components:**
Part
Description
Goal
Request to start a task
Feedback
Updates while the task is in progress
Result
Final output when the task completes
Cancel
Option to cancel the task
Action Server
The part that performs the task
Action Client
The part that requests the task and listens for updates
## **6. Parameters**
    • Nodes can have configurable parameters (e.g., speed limit, sensor threshold).
    • These can be loaded at runtime.
## **7. Packages**
    • A package is a self-contained unit of code.
    • It typically contains:
        ◦ Source code (nodes, libraries)
        ◦ Launch files
        ◦ Configuration files
        ◦ Dependencies
## **8. Launch Files**
    • Launch files are scripts that tell ROS which nodes to start and how to configure them.
    • Used to start multiple nodes with specific parameters, in specific namespaces.

## **Example Use-Case**
Imagine you’re building a robot that avoids obstacles:
    1. Laser scanner publishes data to /scan.
    2. Obstacle detection node subscribes to /scan, calculates danger.
    3. It publishes movement commands to /cmd_vel.
    4. Motor controller node subscribes to /cmd_vel and drives the wheels.



## **Sourcing**
We do this in order to add packages, nodes, etc to the environment.
Its telling the terminal that, where to find the source file, so that we can use the related commands and stuff from that file.

### **Commands: **

### 1. Will have to source ros everytime you open new terminal. 
	`source /opt/ros2/humble/setup.bash`
 
### 2. Will automatically source ros everytime you open new terminal.
	`echo "source /opt/ros2/humble/setup.bash" >> ~/.bashrc`

here we are adding the sourcing command to .bashrc file. .bashrc file runs everytime you open terminal. So, it automatically runs that sourcing command too.





## **Executables**
    • Executables in ROS are the compiled or runnable programs (nodes) that perform specific tasks in your robot system.
    • They are the actual files you run (like a robot driver or sensor reader) that do the work inside ROS.

### **TO LIST ALL THE PACKAGES OF ROS2:**
`ros2 pkg list`

### **TO LIST ALL THE EXECUTABLES OF ROS2:**
`ros2 pkg executables`

### **TO LIST EXECUTABLES OF A SPECIFIC PKG:**
`ros2 pkg executables _name_of_pkg_` 


### **TURTLESIM EXAMPLE:**
Turtlesim is an inbuilt pkg in ros2.

1st list down the executables within this pkg : 
`ros2 pkg executables turtlesim`

now run these executables : 
`ros2 run turtlesim turtlesim_node`
`ros2 run turtlesim turtle_teleop_key`


### **TO LIST THE NODES RUNNING :**
`ros2 node list`

### **TO GET INFO ABOUT RUNNING NODE : **
`ros2 node info /turtlesim`

### **TO LIST TOPICS :**
`ros2 topic list`

### **TO LIST THE TYPES WITH THESE TOPICS : **
`ros2 topic list -t`
Eg: 
```vihaan@vihaan-Inspiron-14-5430:~/Desktop$ ros2 topic list -t
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
/turtle1/color_sensor [turtlesim/msg/Color]
/turtle1/pose [turtlesim/msg/Pose]```

for eg here,
/turtle1/cmd_vel                         → this is a topic
geometry_msgs/msg/Twist        → this the types to messages that flow in this topic 


### **TO GET DATA IN SPECIFIC TOPIC : **
`ros2 topic echo  _topic_name_`



The subscriber count and publisher count tell you how many nodes are currently connected to that topic.



### **TO PUBLISH DATA TO THE TOPIC (only once) : **
eg:
`ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist “{linear:  {x:2.0, y:0.0,z:0.0}},  {angular: { x:0.0, y:0.0, z:1.8}}”`

### **TO PUBLISH DATA TO THE TOPIC AT A SPECIFIC RATE :**
eg:
`ros2 topic pub --rate 1  /turtle1/cmd_vel geometry_msgs/msg/Twist “{linear:  {x:2.0, y:0.0,z:0.0}},  {angular: { x:0.0, y:0.0, z:1.8}}”`

### **TO DISPLAY FREQENCY AT WHICH DATA IS BEING PUBLISHED ON THE TOPIC:**
`ros2 topic hz /turtle1/pose`


### **TO DISPLAY ALL SERVICES:**
`ros2 service list`

### **TO FIND A SERVICE WITH SPECIFIED TYPE:**
`ros2 service find  _type_`


### **TO SEE INTERFACE OF A SERVICE :**
`ros2 interface show turtlesim/srv/Spawn`

#### you will get output something like this : 
```
n
float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name
```

#### **Interpretation of the output:**
**Request (before the --- line):**
These are the inputs you give when calling the service:
Field
Type
Description
x
float32
X position to spawn the turtle
y
float32
Y position to spawn the turtle
theta
float32
Orientation (angle in radians)
name
string
Optional: name of the turtle (or leave blank to auto-generate)

**Response (after the --- line):**
This is the output you get back from the service:
Field
Type
Description
name
string
The actual name of the turtle that was spawned



### **TO CALL A SERVICE:**
`ros2 service call <service_name> <service_type> "<request_data>"`

eg: 
`ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.0, y: 5.0, theta: 1.57, name: 'leo'}"`


### **TO LIST THE PARAMETERS: **
`ros2 param list`

### **TO GET SPECIFIC PARAMETER’S VALUE:**
`ros2 param get <parameter_name>`
eg : 
`ros2 param get /turtlesim background_g`


### **TO SET PARAMETER TO SOME SPECIFIC VALUE:**
`ros2 param set /turtlesim background_g 50`
(this code sets value to 50)


### **TO DUMP THE PARAMETERS WITHOUT SPECIFYING FILE:**
`ros2 param dump /turtlesim`


### **TO DUMP THE PARAMETERS IN A SPECIFIC FILE:**
`ros2 param dump /turtlesim --output-file file_name.yaml`

if you change some parameters and dump it again (using $ ros2 param dump /turtlesim), system will overwrite the previous file.
System stores only one set of parameter for one specific node at a time. We can store parameter values for various nodes in same file.


### **TO LOAD PARAMETERS:**
`ros2 param load <node_name> <file.yaml>`

On Loading the parameter file (.yaml), the parameter values of current running node will change to these loaded values.


### **ACTION COMMANDS:**
`ros2 action list`
`ros2 action list -t`
`ros2 action info turtle1/rotate_absolute`
`ros2 interface show turtlesim/action/RotateAbsolute`
`ros2 action send_goal <action_name> <action_type> "<goal>"`


## **COLCON:**
Command line tool used to build our own packages.


## **ROS 2 Workspace Structure :**

workstation/              <-- Your ROS 2 workspace
├── src/                  <-- Source directory for all packages
│   ├── my_cpp_package/   <-- A C++ package
│   │   ├── src/
│   │   │   └── node1.cpp  <-- C++ node code
│   │   └── CMakeLists.txt
│   └── my_python_package/  <-- A Python package
│       ├── my_python_package/
│       │   └── node2.py   <-- Python node code
│       └── setup.py
├── build/                <-- Generated after `colcon build`
├── install/              <-- Generated after build
└── log/                  <-- Build/run logs




## **BAG FILE (very basics):**
    • Used to record data being published in topics
 
### **To record data being published in a topic, into a bag file :** 
    1. Create a folder to save the recordings, let it be bag_dir.
       
    2. Now in one of the terminals, go inside this directory.
       
    3. Now run : $ ros2 bag record <topic_name>
       
    4. The file name in which this data is stored is auto-generated, so to change it use -o : 
       ros2 bag record -o <new_name> <topic_name>

#### **To record all the topics, use -a**

### **To see details about our bag file :**
`ros2 bag info <bag_file_name>`

### **To replay whatever was stored in bag file :**
`ros2 bag play <bag_file_name>`
 









## **Do not get confused with name of node, file and executable :**
Concept
What It Refers To
Where It's Defined
File name
The name of your .py or .cpp source file
In your file system (e.g., talker.py, talker.cpp)
Node name
The internal name of the node in the ROS 2 graph
In the code itself — e.g., Node('talker')
Executable name
The name used with ros2 run to start the node
Declared in setup.py (Python) or CMakeLists.txt (C++)


# **ROS 2 Workflow Overview**

## **How to create a simple publisher node**
Here's a simplified graphical overview of the process we'll follow:
[1] Set up workspace
      ↓
[2] Create a package
      ↓
[3] Write code (e.g., nodes)
      ↓
[4] Build the workspace
      ↓
[5] Source the setup file
      ↓
[6] Run the nodes

**1. Set Up Your ROS 2 Workstation**
A “workspace” is where you develop and build ROS 2 packages.
**Step-by-Step:**
 1. Open a terminal and choose a location where you want to create your workspace.
 2. Create the workspace and source folders:
```
mkdir -p ~/ros2_ws/src 
cd ~/ros2_ws
```
 3. Source the ROS 2 environment if not sourced already:
`source /opt/ros/humble/setup.bash`  
 4. You now have a workspace set up at:
~/ros2_ws/  
└── src/  
That’s it — your workstation is ready!

**2. Create a ROS 2 Package**
Now we’ll create a simple Python-based package.
**Package Creation Command:**
```
cd ~/ros2_ws/src  
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy  
```
This creates:
my_py_pkg/  
├── package.xml  
├── setup.py  
├── setup.cfg  
├── resource/  
│   └── my_py_pkg  
└── my_py_pkg/  
    └── __init__.py  

**3. Create Node**
Inside the newly created package folder:
 1. Create a Python file:
```
cd ~/ros2_ws/src/my_py_pkg/my_py_pkg  
touch simple_node.py  
chmod +x simple_node.py
```  
 2. Edit simple_node.py:
```
import rclpy  
from rclpy.node import Node  

class HelloNode(Node):  
    def __init__(self):  
        super().__init__('hello_node')  
        self.get_logger().info('Hello from ROS 2!')  

def main(args=None):  
    rclpy.init(args=args)  
    node = HelloNode()  
    rclpy.spin(node)  
    node.destroy_node()  
    rclpy.shutdown()  

if __name__ == '__main__':  
    main()  
```
**3.5. Modify setup.py to Register the Executable**
Edit the setup.py file located in ~/ros2_ws/src/my_py_pkg/setup.py.
Find the entry_points section and add this:
```
entry_points={  
    'console_scripts': [  
        'simple_node = my_py_pkg.simple_node:main',  
    ],  
},
```  
This tells ROS 2 that ros2 run my_py_pkg simple_node should execute the main() function in simple_node.py.

**4. Build the Workspace**
Go back to the root of your workspace and build it:
```
cd ~/ros2_ws  
colcon build  
```
**5. Source the Setup File**
After building, you must source the local setup script:
`source install/setup.bash`  

**6. Run Your Node**
Now that everything is built and sourced, run your node:
`ros2 run my_py_pkg simple_node`  
You should see:
`[INFO] [hello_node]: Hello from ROS 2!`  




## ROS 2 Python Package Structure
When you run something like:
`ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy std_msgs`
ROS 2 generates a structure like this:
my_py_pkg/
├── my_py_pkg/
│   └── __init__.py
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── my_py_pkg
├── test/
│   └── test_my_py_pkg.py
Let’s go over each part:

### Top-Level: Project Root (my_py_pkg/)
This is the root of your ROS 2 package. When you build your workspace with colcon build, ROS 2 looks here.

### 1. package.xml
Purpose: Describes the package to the ROS build system.
Includes:
    • Package name, version, description
    • Maintainer & license
    • Dependencies (like rclpy, std_msgs)
    • Build type (ament_python, ament_cmake, etc.)
Required by ROS to resolve dependencies and build your package.

### 2. setup.py
Purpose: Standard Python packaging file used to build/install the package.
Includes:
    • Package name
    • Install instructions
    • Entry points (what node to run with ros2 run)
    • Maintainer, version, license
Think of this as the Python equivalent of CMakeLists.txt.

### 3. setup.cfg
Purpose: Provides metadata and build options in a simpler format (used with setup.py).
Often looks like:
```
[develop]
script_dir=$base/lib/my_py_pkg
[install]
install_scripts=$base/lib/my_py_pkg
```
ROS uses this to install your scripts to the correct location.

### 4. my_py_pkg/ (Python Package Folder)
This is the actual Python module.
    • It must have the same name as the package (ROS uses this to find your code).
    • It contains your Python nodes and logic.
### __init__.py
    • Required for Python to treat this folder as a module.
    • Can be empty.
    • You can also define reusable code here (like shared functions).

### 5. resource/my_py_pkg
Purpose: Used internally by ament_index.
    • Just an empty file named after your package
    • Used so that ROS 2 tools like ros2 pkg list can discover your package at runtime
Leave it alone. It’s auto-managed.

### 6. test/
Purpose: Contains unit tests for your package (optional but recommended).
test_my_py_pkg.py
    • You can write tests using unit test or pytest
    • colcon test will run them automatically if defined

### Optional Extras We May Add
File/Folder
Purpose
launch/
Launch files to start multiple nodes
msg/
Custom message definitions
srv/
Custom service definitions
config/
YAML files for parameters
README.md
Documentation






## Explaination of code of workstation_1 my_node_1
**Code: **
```
    1. 
    2. import rclpy
    3. from rclpy.node import Node
    4. 
    5. class HelloNode(Node):
    6.     def __init__(self):
    7.         super().__init__('hello_node')
    8.         self.get_logger().info('Hello from ROS 2!')
    9. 
    10. def main(args=None):
    11.     rclpy.init(args=args)
    12.     node = HelloNode()
    13.     rclpy.spin(node)
    14.     node.destroy_node()
    15.     rclpy.shutdown()
    16. 
    17. if __name__ == '__main__':
    18. main()
```
**Explaination:**
**rclpy :** rclpy (ROS 2 Python client library)
**Node :** Node is a fundamental class that acts as a container for all ROS 2 communications, such as publishers, subscribers, services, actions, parameters, timers, and logging.
**Line 6:** here we initialize the parent Node class and register our node with the name 'hello_node' in the ROS 2 system.
**get_logger():** its a method that returns an object, and with this object you can use following methods : .debug(), .info(), .warn(), .error(), .fatal(). These methods format your message and send it to the ROS 2 logging pipeline.
**rclpy.init(args=args):** Initializes the ROS 2 Python client library. Prepares the system so you can create nodes, use publishers, subscribers, timers, etc.
**rclpy.spin(node):**
 • Keeps the node alive so it can do its work (e.g., respond to callbacks).
 • In this example, there are no timers, subscriptions, or publishers, so it just stays alive doing nothing until you shut it down.
**node.destroy_node():**
 • Destroys the node.
 • Frees any resources allocated by the node.
**rclpy.shutdown():**
 • Shuts down the ROS 2 Python client library.
 • Must be called after all nodes are done.
**Line 16:** Is this file being run directly (like python myfile.py) or is it being imported into another file?
Every .py has an inbuilt variable name.
If we run the file directly, name == ‘main’ is true,
if we import the file, then its false.
This makes sure that the file doesn't accidentally auto-run when someone imports your node for testing or reuse.






