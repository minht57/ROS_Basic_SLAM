
 
# BUILDING AN AUTOMATIC VEHICLE BASED ON STEREO CAMERA

<p align="center">
  <img width="240" height="360" src="https://github.com/minht57/ROS_Basic_SLAM/blob/master/img/robot.png">
</p>

## Overview
This project focuses on using stereo camera to build a vehicle which can map the surrounding area and autonomosly navigate a to the pointed destination in the built map. The vehicle is a Turtlebot model and based on ROS (Robot Operating System). For a further purpose, the vehicle can be developed to go around a house doing diffrent tasks or explore unknown enviroment. 

## Algorithm
![Algorithm](https://github.com/minht57/ROS_Basic_SLAM/blob/master/img/bot.png)

## Dependencies
In order to run the project, the following dependencies are required:

- Ubuntu 16.04 [here](http://cdimage.ubuntu.com/netboot/16.04/?_ga=2.243318149.1855666904.1529366501-828848615.1529366501)
- ROS Kinetic Kame [here](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- Catkin tool [here](http://wiki.ros.org/catkin)
- ZED SDK (version 2.1) [here](https://www.stereolabs.com/developers/release/2.1/)
- Cuda Toolkit (version 8.0) [here](https://developer.nvidia.com/cuda-80-ga2-download-archive)
- Firmware for Mobile base [here](https://github.com/minht57/STM32F4_BaseControl)
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

To see the map, launch Rviz and create visualization by adding 'Map' to the 'Display' console:
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

Edit the map with GIMP for better map:
<p align="center">
  <img width="460" height="300" src="https://github.com/minht57/ROS_Basic_SLAM/blob/master/img/editedmap.png">
</p>

### Navigation:

Run navigation:

```
roslaunch navigation slam_amcl.launch
```
Send velocity commands to move base via serial port:
```
sudo chmod 777 /dev/ttyUSB0
roslaunch serial serial.launch
```

<p align="center">
  <img width="460" height="300" src="https://github.com/minht57/ROS_Basic_SLAM/blob/master/img/navImage01.jpg">
</p>

Place a box in the area to check object avoidance algorithm.
<p align="center">
  <img width="460" height="300" src="https://github.com/minht57/ROS_Basic_SLAM/blob/master/img/navImage.jpg">
</p>

Open Rviz:

```
rviz
```

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

## Result:
Youtube:
 - Mapping:  [here
](https://www.youtube.com/watch?v=OGnmODjk6VQ)
 - Navigation: [here](https://www.youtube.com/watch?v=Besl-2XVk7A)
