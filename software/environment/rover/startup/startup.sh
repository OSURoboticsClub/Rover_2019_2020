#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/nvidia/catkin_workspace/devel/setup.bash
/home/nvidia/Github/Rover_2017_2018/software/environment/rover/auto_poweroff/auto_poweroff.py &
roslaunch rover_main rover.launch
exec bash
