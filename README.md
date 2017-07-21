# DifferentialDriveRobot

### Brief Explanation

* **myrobot_description** specifies the entire robot structure as links and joints and can launch the model in rviz.
* **myrobot_gazebo** launches the model in the gazebo environment and contains different simulation worlds.
* **myrobot_control** launches the model in the gazebo environment where the robot motion can be commanded by the keyboard.

### Create the Robot Model (URDF)
In ~/catkin_ws/src/myrobot_description/urdf, there are four files:

* myrobot.xacro: primary file that loads the other three files and contains only URDF items like joints and links
* myrobot.gazebo: contains gazebo-specific labels that are wrapped within gaz
* materials.xacro: maps strings to colors
* macros.xacro: macros to help simplify

### Run the Models
Load the Gazebo simulator and rviz in separate terminals using the following commands:
```
roslaunch myrobot_gazebo myrobot_world.launch
roslaunch myrobot_description myrobot_rviz.launch
```
#### Circle mode:
On a new terminal use the following command to make the robot drive incessantly along a circle of user-defined diameter. 
(Here diameter = 4 m. So, radius = 2m. Thus linear and angular velocities are set accordingly) 
```
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.1"
```

#### Keyboard teleop mode:
The ~/catkin_ws/src/myrobot_control/scripts folder contains the *myrobot_key* node, which is the teleop node.

* Launch the gazebo simulator with complete simulation settings using the following command:
```
roslaunch myrobot_gazebo myrobot_gazebo_full.launch
```

* Start the teleop node:
```
roslaunch myrobot_control keyboard_teleop.launch
```

* Start RViz to visualize the robot state:
```
rosrun rviz rviz
``




