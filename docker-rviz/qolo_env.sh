#!/bin/bash
export ROS_MASTER_URI=http://192.168.13.110:11311
# export ROS_MASTER_URI=http://128.179.163.192:11311

# Startup rviz
rviz -d rviz/qolo_rviz_all.rviz 
