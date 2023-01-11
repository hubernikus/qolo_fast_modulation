#!/bin/bash

# it: Do iterative or non-iterative terminal
docker run \
	   -it \
	   -e DISPLAY=$DISPLAY \
	   -h $HOSTNAME \
	   --net host \
	   -v /tmp/.X11-unix:/tmp/.X11-unix \
	   -v $HOME/.Xauthority:/home/ros/.Xauthority \
	   -v "$(pwd)"/scripts:/home/ros/catkin_ws/src/qolo_fast_modulation/scripts\
	   -v "$(pwd)"/src/dynamic_obstacle_avoidance/dynamic_obstacle_avoidance:/home/ros/python/dynamic_obstacle_avoidance/dynamic_obstacle_avoidance\
	   -v "$(pwd)"/src/various_tools/vartools:/home/ros/python/various_tools/vartools\
	   -v "$(pwd)"/src/fast_obstacle_avoidance/fast_obstacle_avoidance:/home/ros/python/fast_obstacle_avoidance/fast_obstacle_avoidance\
	   ros_qolo_fast_modulation

# Alternative mounting?!
# --mount type=bind,source="$(pwd)"/visualization,target=/home/ros/rviz \

# Change user to root
# -u root \

# Copy specify file
# -v "$(pwd)"/docker-rviz/qolo_env.sh:/home/ros/qolo_env.sh\


# Run with specific user
# -u root \


