# DifferentialDriveRobot

### Brief Explanation:

* **myrobot_description** specifies the entire robot structure as links and joints and can launch the model in rviz.
* **myrobot_gazebo** launches the model in the gazebo environment and contains different simulation worlds.
* **myrobot_control** 

### Create the Robot Model (URDF)
In ~/myrobot_ws/src/myrobot_description/urdf, there are four files:

* myrobot.xacro: primary file that loads the other three files and contains only URDF items like joints and links
* myrobot.gazebo: contains gazebo-specific labels that are wrapped within gaz
* materials.xacro: maps strings to colors
* macros.xacro: macros to help simplify


