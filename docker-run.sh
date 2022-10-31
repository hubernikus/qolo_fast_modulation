#!/bin/bash

# it: Do iterative or non-iterative terminal
docker run \
	   -it \
	   -e DISPLAY=$DISPLAY \
	   -h $HOSTNAME \
	   -v /tmp/.X11-unix:/tmp/.X11-unix \
	   -v $HOME/.Xauthority:/home/ros/.Xauthority \
	   -v "$(pwd)"/qolo_env.sh:/home/ros/qolo_env.sh\
	   -v "$(pwd)"/scripts:/home/ros/catkin_ws/src/qolo_fast_modulation/scripts\
	   ros_qolo_fast_modulation

# --mount type=bind,source="$(pwd)"/visualization,target=/home/ros/rviz \
# -u root \

# Run with specific user
# -u root \


