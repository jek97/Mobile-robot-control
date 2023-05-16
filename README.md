# ROS bug0 controll of a robot #

this repository contain the implementation of an autonomous mobile robot able to move following the bug0 algorithm to the given target.
The implementation use the Gazebo simulator together with the Rviz visualizer to give a direct rappresentation of what's going on, in detail the differential robot is provided by a lidar, to perform an external measure of the environment, and encoders, to perform an internal measure and so the odometry, all these component are then all combined in a ROS architecture.
The starting code can be downloaded at the link: [GitHub repository](https://github.com/CarmineD8/assignment_2_2022)

## How to run the project ##
Note that to run the project it's necessary to have both ROS (Noetic) and Python3 installed on your machine.
1. Download the package from github in your local workspace: [GitHub repository](https://github.com/jek97/RT1.git)
2. instal xterm, by the command:
```
sudo apt-get install xterm
```
3. Move to the root folder of the work space with the command:
```
$ cd RT1_ros_ws/src/assignment_2_2022
```
4. Build the environment with the command:
```
$ catkin_make
```
5. Launch the program by the command:
```
$ roslaunch assignment_2_2022 assignment1.launch
```
## related documentation ##
[Documentation](https://jek97.github.io/Mobile-robot-control/)
