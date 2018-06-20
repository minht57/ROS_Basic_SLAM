
 
# BUILDING AN AUTOMATIC VEHICLE BASED ON STEREO CAMERA

<p align="center">
  <img width="460" height="300" src="https://github.com/minht57/ROS_Basic_SLAM/blob/master/img/robot.png">
</p>

## Overview
The goal of this project is to build a map of surrounding area and autonomosly control a robot to the pointed destination in the map. The robot is a Turtlebot model and base on ROS (Robot Operating System). This project focuses on using Sereo camera to reduce the cost. Robot can build a map, navigate and go to destinated position, which can be futher developed to go around a house doing diffrent tasks or exploring unknown enviroment. 

## Algorithm
![Algorithm](https://github.com/minht57/ROS_Basic_SLAM/blob/master/img/bot.png)

## Dependencies
In order to run the project, the following dependencies are required:

 - Ubuntu 16.04 [here](http://cdimage.ubuntu.com/netboot/16.04/?_ga=2.243318149.1855666904.1529366501-828848615.1529366501)
- ROS Kinetic Kame [here](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- Catkin tool [here](http://wiki.ros.org/catkin)
- ZED SDK (version 2.1) [here](https://www.stereolabs.com/developers/release/2.1/)
- Cuda Toolkit (version 8.0) [here](https://developer.nvidia.com/cuda-80-ga2-download-archive)
- ROS packages:
	- zed_ros_wrapper
	- depthimage_to_laserscan
	- laser_scan_matcher
	- gmapping
	- map_server
	- move_base
	- amcl
	- dwa_local_planner

Install ROS package dependencies:
```
sudo apt install ros-kinetic-<dependency_name>
```
## Build and run the robot

### Setup workspace:
```
git clone https://github.com/minht57/ROS_Basic_SLAM.git
cp -rf ROS_Basic_SLAM/src/* ~/catkin_ws/src/
sudo chmod 777 ~/catkin_ws/src/zed-ros-wapper/cfg/Zed.cfg
sudo chmod 777 ~/catkin_ws/src/depthimage_to_laserscan/cfg/Depth.cfg
cd ~/catkin_ws
catkin_make
```
### Mapping:
Run the mapping:
```
roslaunch depthimage_to_laserscan gmapping.launch
```
Use RF controller to move the robot around. The map will be automatically built.
To see the map, launch Rviz and create visualization by adding 'Map' to the Display console:
```
rviz
```
Save map:
```
rosrun map_server map_saver -f ~/catkin_ws/test_map
```
Raw map:
<p align="center">
  <img width="460" height="300" src="https://github.com/minht57/ROS_Basic_SLAM/blob/master/img/map.png">
</p>

Filter the map using GIMP:
<p align="center">
  <img width="460" height="300" src="https://github.com/minht57/ROS_Basic_SLAM/blob/master/img/editedmap.png">
</p>

### Navigation:

Run navigation:

```
roslaunch navigation slam_amcl.launch
```
Send velocity commands via serial port to move base:
```
sudo chmod 777 /dev/ttyUSB0
roslaunch serial serial.launch
```

Place a box in the area to check object avoidance algorithm.
<p align="center">
  <img width="460" height="300" src="https://github.com/minht57/ROS_Basic_SLAM/blob/master/img/navImage.jpg">
</p>

Open Rviz:
```rviz```

<p align="center">
  <img width="460" height="300" src="https://github.com/minht57/ROS_Basic_SLAM/blob/master/img/nav01.png">
</p>

<p align="center">
  <img width="460" height="300" src="https://github.com/minht57/ROS_Basic_SLAM/blob/master/img/nav02.png">
</p>

<p align="center">
  <img width="460" height="300" src="https://github.com/minht57/ROS_Basic_SLAM/blob/master/img/nav03.png">
</p>

<p align="center">
  <img width="460" height="300" src="https://github.com/minht57/ROS_Basic_SLAM/blob/master/img/nav04.png">
</p>

### Result:
Youtube:
 - Mapping:  [here
](https://www.youtube.com/watch?v=OGnmODjk6VQ)
 - Navigation: [here](https://www.youtube.com/watch?v=Besl-2XVk7A)
