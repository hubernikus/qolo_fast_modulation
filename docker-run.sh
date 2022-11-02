#!/bin/bash

# it: Do iterative or non-iterative terminal
docker run \
	   -it \
	   -e DISPLAY=$DISPLAY \
	   -h $HOSTNAME \
	   -v /tmp/.X11-unix:/tmp/.X11-unix \
	   -v $HOME/.Xauthority:/home/ros/.Xauthority \
	   -v "$(pwd)"/scripts:/home/ros/catkin_ws/src/qolo_fast_modulation/scripts\
	   -v "$(pwd)"/src/fast_obstacle_avoidance/fast_obstacle_avoidance:/python/fast_obstacle_avoidance/fast_obstacle_avoidance\
	   -v "$(pwd)"/src/fast_obstacle_avoidance/scripts:/python/fast_obstacle_avoidance/scripts\
	   ros_qolo_fast_modulation

# -v "$(pwd)"/src/various_tool:/python/fast_obstacle_avoidance/dynamic_obstacle_avoidance\
# -v "$(pwd)"/src/dynamic_obstacle_avoidance/dynamic_obstacle_avoidance:/python/fast_obstacle_avoidance/dynamic_obstacle_avoidance\

# Alternative mounting?!
# --mount type=bind,source="$(pwd)"/visualization,target=/home/ros/rviz \

# Change user to root
# -u root \

# Copy specify file
# -v "$(pwd)"/docker-rviz/qolo_env.sh:/home/ros/qolo_env.sh\


# Run with specific user
# -u root \

