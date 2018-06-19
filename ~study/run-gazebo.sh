#!/bin/bash

source devel/setup.bash

sudo killall gzserver
sudo killall gzclient
sudo killall rviz
sudo killall roscore
sudo killall rosmaster

roslaunch robot_gazebo mybot_world.launch
