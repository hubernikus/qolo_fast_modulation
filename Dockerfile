FROM ros:noetic-ros-base
# FROM ros:melodic-ros-base

# INSTALL NECESSARY PACKAGES
RUN apt update \
	&& apt install -y \
	tmux \	
vim-python-jedi \
	# gnupg2 curl lsb-core \ 
	# libpng16-16 libjpeg-turbo8 libtiff5 \
	# ros-${ROS_DISTRO}-rviz \
	&& apt clean

# RUN apt install -y qtbase5-dev qtdeclarative5-dev
RUN apt-get install -y python3.9
RUN apt-get install -y python3-pip
# RUN apt-get install python3.9-venv

# TF in python
RUN apt-get install -y ros-${ROS_DISTRO}-tf

# Allow matplotlib-plotting
RUN apt-get install -y python3-tk

# Files are currently just copied -> direct access from github could be done (?)
# but this would require (stable) tags
# Create a user called ROS
RUN groupadd -g 1000 ros
RUN useradd -d /home/ros -s /bin/bash -m ros -u 1000 -g 1000


USER ros
ENV HOME /home/ros

# Install QOLO package
# USER root
# RUN apt install -y git  # TODO: move to general install
# USER ros
# RUN mkdir -p ${HOME}/catkin_ws/src
# WORKDIR ${HOME}/catkin_ws/src
# RUN git clone https://github.com/DrDiegoPaez/qolo_ros.git

RUN mkdir -p ${HOME}/catkin_ws/src/qolo_fast_modulation/scripts

WORKDIR ${HOME}/catkin_ws/src/qolo_fast_modulation
# COPY messages src/messages
COPY requirements.txt requirements.txt
COPY CMakeLists.txt CMakeLists.txt
COPY package.xml package.xml
COPY msg msg

# RUN python3 -m pip install -r requirements.txt

# Optional: source could be directly downloaded from git (but no local changes possible...)
# The source code is already copied to the python directory
# COPY src src

# Local environment to allow for installation
# RUN python3.9 -m venv env
# RUN source ./env/bin/activate

# COPY scripts scripts
# Set ROS environment -> 
# ENV ROS_MASTER_URI=http://128.179.186.206:11311

# ROS environment for QOLO
# ENV ROS_MASTER_URI=http://128.179.186.206:11311

USER ros

WORKDIR ${HOME}/catkin_ws
# RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Install python stuff alongside with ros
COPY src /home/ros/python

USER root
RUN bash /home/ros/.bashrc

WORKDIR /home/ros/python/various_tools
RUN sudo python3 -m pip install -e .
RUN sudo python3 -m pip install -r requirements.txt

WORKDIR /home/ros/python/dynamic_obstacle_avoidance
RUN python3 -m pip install -r requirements.txt
RUN python3 -m pip install -e .

WORKDIR /home/ros/python/fast_obstacle_avoidance
RUN python3 -m pip install -r requirements.txt
RUN python3 -m pip install -e .

# Resolve few conflicts
RUN python3 -m pip install numpy --upgrade
RUN python3 -m pip install --upgrade scikit-image

# RUN export ROS_MASTER_URI=http://localhost:11311
USER root
ENV ROS_MASTER_URI http://192.168.13.110:11311
ENV ROS_IP 192.168.13.120

# WORKDIR ${HOME}
WORKDIR ${HOME}/catkin_ws/src/qolo_fast_modulation/scripts

ENTRYPOINT tmux
# ENTRYPOINT echo "Welcome to Docker"
