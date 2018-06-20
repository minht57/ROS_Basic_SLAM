 
# BUILDING AN AUTOMATIC VEHICLE BASED ON STEREO CAMERA

## Overview

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
sudo apt install ros-kinetic-<dependencies_name>
```
## Build and run the robot

First, we need to change mode for the packages:

```
 sudo chmod 777 ~/catkin_ws/src/zed-ros-wapper/cfg/Zed.cfg
 sudo chmod 777 ~/catkin_ws/src/depthimage_to_laserscan/cfg/Depth.cfg
 ```

### Setup workspace:
```
git clone https://github.com/minht57/ROS_Basic_SLAM.git
cp -f ROS_Basic_SLAM/src/* ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
```
### Mapping:
```
roslaunch depthimage_to_laserscan gmapping.launch
rviz
```
[Picture]
Use RF controller to build your map and save your map using this command
```
rosrun map_server map_saver -f ~/catkin_ws/test_map
```
[Picture]

### Navigation:

Running navigation using this command:

```roslaunch navigation slam_amcl.launch```
[Picture]

Send velocity commands via Serial to move base:
```
sudo chmod 777 /dev/ttyUSB0
roslaunch serial serial.launch
```

Open Rviz:
```rviz```
[Picture]

### Result:
Youtube:
 - Mapping:  [here
](https://www.youtube.com/watch?v=OGnmODjk6VQ)
 - Navigation: [here](https://www.youtube.com/watch?v=Besl-2XVk7A)