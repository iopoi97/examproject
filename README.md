# Paolino De Risi P38/062 FSR/RL Project 2
The following is a study about computer vision, kinematics, planning algorithms, control and odometry for a differential drive robot given an autonomous navigation capability in a Gazebo-ROS environment

## Getting Started

 1. Clone the Repository in the **/src** folder of your **/ros_ws** or **/catkin_ws** directory,
 2. Unzip the **aruco_ros.zip** package in the same position, then if you want cancel the **.zip** file,
 3. Open a terminal in the **/ros_ws** or **/catkin_ws** and type `catkin_make` command,
 >If necessary add also `-DCATKIN_WHITELIST_PACKAGES=examproject` command;
 
 
 **You are ready to start!!**

## How to start with a simulation
Open a new terminal tab and type:

 1. `roslaunch examproject spawn_my_robot.launch`
 >This will open **Gazebo** with the **my_world** simulation environment and the **my_robot** robot model
 2. `roslaunch examproject aruco.lunch`
 >To allow **AR-Markers** detection
 3. `rosrun examproject odom_new`
 >To start the **Odometry** node
 4. `rosrun examproject navigation`
 >To start the **APF planning** and the **I/O Linearization control** systems
 5. You will be asked to insert an ID to search among 8, 26, 582 
 >**Note**: 8 for full navigation
 6. The navigation will **start**

