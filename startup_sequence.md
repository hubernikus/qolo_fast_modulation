# ########### Launch Sequence for experiments in Qolo robot ######### #
ssh -X qolo@192.168.13.110
main UpBoard 110
Frontal Lidar 120
Hasler 130
Nvidia board 200
	pass: ....

# ============== Experiments for OA recordings - Including Collisions ========================== #

#### Establish 3 connetcions to main UpBoard 110
#### Establish 2 connetcion to Frontal Upboard 120
#### Establish 2 connetcion to Frontal Upboard 200

## Tmux
Use Tmux to launch the different windows; basic commands:  

Start a new session (one per computer) `tmux`  
  
Split window vertically: `Ctrl-b "`  
Split window horizontally: `Ctrl-b "`  
Maximize/Minimize a sub-window `Ctrl-b z`  
Move to specific window: ``Ctrl-b [Arrow Key]`  
Toggle window: ``Ctrl-b o`  

Since it's fully terminal, you can not use your mouse. To move up the history type: `Ctrl-b [` (and `q` to cancel again)  
Close a window: `Ctrl-d` or type `exit`  

### Check that all dates and times are the same on all PCs ###
``` bash
date
```


### If a PC is out of sync it might be that the internal NTP server went down ###
	Manual setup:
``` bash
sudo /etc/init.d/ntp restart
sudo ntpd -q 192.168.13.110
```


1. 110 terminal:
``` bash
rosclean purge -y
roscore
```

**2. 110 terminal:rear LIDAR**
``` bash
cd ~/catkin_ws/
. devel/setup.bash
rosrun qolo rear_lidar2lrf.sh
```

**3. 120 terminal: Launching Front Lidar and Low-level avoidance (RDS)**
``` bash
rosclean purge -y
cd ~/autonomy_ws/
. devel/setup.bash
rosrun rds_ros lidar2lrf.sh
```

**4. 200: Localization from T265 camera**
``` bash
rosclean purge -y
cd /ssd_nvidia/autonomy_ws
. devel/setup.bash 
roslaunch realsense2_camera qolo_localization_t265.launch
```

**6. 110 terminal: Starting main Qolo Control Node**
``` bash
echo "qoloLASA2020" | sudo -S echo "" && sudo -s
cd ~/catkin_ws/
. devel/setup.bash
rosrun qolo compliant_mds_shared_qolo.sh
```

Run the controller with hands-free-flag `-H`:
``` bash
rosrun qolo compliant_mds_shared_qolo.sh -H
```

9.-z) Run the main controller (in a docker container)
``` bash
cd ~/autonomy_ws/src/qolo_fast_modulation
bash docker-run.sh
python3.9 controller_laserscan.py -p # with debug publishing option
```

## Run tracker + MDS
%% --> WAIT 15seconds  BEFORE THE NEXT CAMERA LAUNCH
**5. 200: Yolo People Detection from D435 camera**
``` bash
cd /ssd_nvidia/autonomy_ws
. devel/setup.bash
roslaunch realsense2_camera qolo_left_camera.launch
```

*200* Nvidia-200 terminal: Start People tracker
``` bash
cd ~/tracker_ws
. /ssd_nvidia/venv_sensing/bin/activate
. devel/setup.bash
roslaunch rwth_crowdbot_launch qolo_onboard.launch trt:=true
```

*120* - Run MDS algorithm
``` bash
cd ~/autonomy_ws/
. devel/setup.bash
rosrun qolo_modulation qolo_modulation_ros_controller.py
```

## Recording
** 200 Light recording **
``` bash
cd /ssd_nvidia/data/fast_avoidance
rosbag record /chatter /tf /tf_static qolo/odom /qolo/pose2D /qolo/twist /qolo/remote_commands /qolo/user_commands /front_lidar/scan /rear_lidar/scan /rwth_tracker/pedestrian_array /rwth_tracker/tracked_persons
```



# 10. Visualization- RUN LOCALLY**
# Change to the local workspace with the qolo_ros package
``` bash
cd ~/autonomy_ws/src/qolo_fast_modulation/docker-rviz
bash docker-run.sh
./qolo_env
```


### Old launches
**2. Alternatevely:**
``` bash
roslaunch qolo rear_lidar-cloud.launch
```

**3. Alternatively** launch additionally with RDS
``` bash
cd ~/autonomy_ws/
. devel/setup.bash
rosrun rds_ros rds_lidar2lrf.sh
```




**8. 200: Rosbag Recording**
``` bash
cd /ssd_nvidia/data/irl_obstacles/
rosbag record /tf /tf_static /diagnostics /front_lidar/scan /front_lidar/scan_all /front_lidar/velodyne_points /rear_lidar/velodyne_points /rear_lidar/scan /rear_lidar/scan_all /joint_states /qolo/compliance/svr /qolo/user_commands /qolo/emergency /qolo/odom /qolo/pose2D /qolo/remote_commands /qolo/twist /rds_to_gui /rokubi_node_front/ft_sensor_measurements /rosout /rosout_agg /t265/accel/imu_info /t265/accel/sample /t265/gyro/imu_info /t265/gyro/sample /t265/odom/sample

/t265/fisheye1/camera_info /t265/fisheye1/image_raw /t265/fisheye2/camera_info /t265/fisheye2/image_raw
```

### Current ERROR DEGUB ####
