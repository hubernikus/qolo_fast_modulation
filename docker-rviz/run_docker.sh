#!/bin/bash

# it: Do iterative or non-iterative terminal
docker run \
	   -it \
	   -e DISPLAY=$DISPLAY \
	   -h $HOSTNAME \
	   -v /tmp/.X11-unix:/tmp/.X11-unix \
	   -v $HOME/.Xauthority:/home/ros/.Xauthority \
	   -v  "$(pwd)"/qolo_env.sh:/home/ros/qolo_env.sh\
	   --mount type=bind,source="$(pwd)"/rviz,target=/home/ros/rviz \
	   ros_with_rviz 

