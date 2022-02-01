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
**Alternatevely:**
``` bash
roslaunch qolo rear_lidar-cloud.launch
```

**3. 120 terminal: Launching Front Lidar and Low-level avoidance (RDS)**
``` bash
cd ~/autonomy_ws/
. devel/setup.bash
rosrun rds_ros lidar2lrf.sh
```

**Alternatively** launch additionally with RDS
``` bash
cd ~/autonomy_ws/
. devel/setup.bash
rosrun rds_ros rds_lidar2lrf.sh
```
(launches 

**4. 200: Localization from T265 camera**
``` bash
rosclean purge -y
cd /ssd_nvidia/autonomy_ws
. devel/setup.bash 
roslaunch realsense2_camera qolo_localization_t265.launch
```
%% --> WAIT 15seconds  BEFORE THE NEXT CAMERA LAUNCH

**5. 200: Yolo People Detection from D435 camera**
``` bash
cd /ssd_nvidia/autonomy_ws
. devel/setup.bash 
roslaunch realsense2_camera qolo_left_camera.launch
```

**6. 110 terminal: Starting main Qolo Control Node**
``` bash
echo "qoloLASA2020" | sudo -S echo "" && sudo -s
cd ~/catkin_ws/
. devel/setup.bash
rosrun qolo compliant_mds_shared_qolo.sh
```
run the controller with hands-free-flag `-H`:
``` bash
rosrun qolo compliant_mds_shared_qolo.sh -H
```

**7. Nvidia-200 terminal:  Start People tracker**
``` bash
cd ~/tracker_ws
. /ssd_nvidia/venv_sensing/bin/activate
. devel/setup.bash
roslaunch rwth_crowdbot_launch qolo_onboard.launch trt:=true
```

**8. 200: Rosbag Recording**
``` bash
cd /ssd_nvidia/data/irl_obstacles/
rosbag record /tf /tf_static /diagnostics /front_lidar/scan /front_lidar/scan_all /front_lidar/velodyne_points /rear_lidar/velodyne_points /rear_lidar/scan /rear_lidar/scan_all /joint_states /qolo/compliance/svr /qolo/user_commands /qolo/emergency /qolo/odom /qolo/pose2D /qolo/remote_commands /qolo/twist /rds_to_gui /rokubi_node_front/ft_sensor_measurements /rosout /rosout_agg /t265/accel/imu_info /t265/accel/sample /t265/gyro/imu_info /t265/gyro/sample /t265/odom/sample

/t265/fisheye1/camera_info /t265/fisheye1/image_raw /t265/fisheye2/camera_info /t265/fisheye2/image_raw
```

**9. 120 terminal: MDS Modulation with Underlying linear DS**
``` bash
source ~/autonomy_ws/src/qolo_fast_modulation/.venv/bin/activate
python ~/autonomy_ws/src/qolo_fast_modulation/scripts/controller_laserscan.py
```

9.b) Alternatively run:
eg. for the tracker with scaling do
``` bash
source ~/autonomy_ws/src/qolo_fast_modulation/.venv/bin/activate
python ~/autonomy_ws/src/qolo_fast_modulation/scripts/controller_laserscan.py -s 1.5 -t
```


<!-- Alternative run: -->
<!-- ``` bash -->
<!-- ``` -->

**10. Visualization- RUN LOCALLY**
# Change to the local workspace with the qolo_ros package
``` bash
cd ~/qolo_ws/
. devel/setup.bash
roslaunch qolo rviz.launch
```
### Current ERROR DEGUB ####
