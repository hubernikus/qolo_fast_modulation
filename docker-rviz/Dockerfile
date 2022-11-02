# FROM ros:noetic-ros-base
FROM ros:melodic-ros-base

# INSTALL NECESSARY PACKAGES
RUN apt update \
	&& apt install -y \
	vim tmux wget \
	gnupg2 curl lsb-core \ 
	libpng16-16 libjpeg-turbo8 libtiff5 \
	ros-${ROS_DISTRO}-rviz \
	&& apt clean

# How to install jsk-twist message ?!
# RUN apt install -y --fix-missing ros-${ROS_DISTRO}-jsk-rviz-plugins

# Create a user called ROS
RUN groupadd -g 1000 ros
RUN useradd -d /home/ros -s /bin/bash -m ros -u 1000 -g 1000

# Set ROS environment -> 
# ENV ROS_MASTER_URI=http://128.179.186.206:11311

# ROS environment for QOLO
# ENV ROS_MASTER_URI=http://128.179.186.206:11311

RUN apt install -y qtbase5-dev qtdeclarative5-dev

USER ros
ENV HOME /home/ros

RUN mkdir -p ${HOME}/catkin_ws/src

WORKDIR ${HOME}/catkin_ws/src
COPY spencer_tracking_rviz_plugin spencer_tracking_rviz_plugin
COPY messages messages

# Additional messages (nice twist) -> does not work..
# RUN git clone https://github.com/jsk-ros-pkg/jsk_visualization.git
# RUN git clone https://github.com/jsk-ros-pkg/jsk_common_msgs.git
# RUN /opt/ros/${ROS_DISTRO}/setup.sh

# RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'
WORKDIR ${HOME}/catkin_ws
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make'

WORKDIR ${HOME}
CMD tmux

# CMD rviz -d rviz/qolo_rviz_all.rviz 

