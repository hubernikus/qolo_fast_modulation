FROM ros:noetic-ros-base

# INSTALL NECESSARY PACKAGES
RUN apt update \
	&& apt install -y \
	vim tmux wget \
	gnupg2 curl lsb-core \ 
	libpng16-16 libjpeg-turbo8 libtiff5 \
	ros-${ROS_DISTRO}-rviz \
	&& apt clean

# Create a user called ROS
RUN groupadd -g 1000 ros
RUN useradd -d /home/ros -s /bin/bash -m ros -u 1000 -g 1000

# Set ROS environment -> 
# ENV ROS_MASTER_URI=http://128.179.186.206:11311

# ROS environment for QOLO
# ENV ROS_MASTER_URI=http://128.179.186.206:11311

USER ros
ENV HOME /home/ros

WORKDIR /home/ros
CMD tmux

# CMD rviz -d rviz/qolo_rviz_all.rviz 

