# Differential Drive Robot

### package dependencies
* rospy
* roscpp
* geometry_msgs


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
![simulation_in_gazebo_and_rviz](https://user-images.githubusercontent.com/5114945/29138625-5ee60c4a-7d12-11e7-8a74-f363cd7b2a8b.png)

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
Can also run using script. The ~/catkin_ws/src/myrobot_gazebo/scripts contains the circle_mode.py node.

* Launch the gazebo simulator using the following command:
```
roslaunch myrobot_gazebo myrobot_world.launch
```
* Start the circle_mode node:
```
rosrun myrobot_gazebo circle_mode.py
```

![circle_mode](https://user-images.githubusercontent.com/5114945/29154671-10523ce6-7d64-11e7-94c5-bfdad3751dd6.png)

#### Square mode:
The ~/catkin_ws/src/myrobot_gazebo/scripts contains the square_mode.py node. It draws a square of 0.4m side.

* Launch the gazebo simulator using the following command:
```
roslaunch myrobot_gazebo myrobot_world.launch
```
* Start the square_mode node:
```
rosrun myrobot_gazebo square_mode.py
```
This method is not perfect as the robot is way off from the point it started, after completing the square. Using motion planning the action can be made accurate.

#### Keyboard teleop mode:
The ~/catkin_ws/src/myrobot_control/scripts folder contains the *myrobot_key* node, which is the teleop node. There is already a standard teleop node implementation available (for the turtlebot), we simply reused the node. Then a remapping is done from the turtlebot_teleop_keyboard/cmd_vel to /cmd_vel of our robot in the *keyboard_teleop.launch* file.

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
```




