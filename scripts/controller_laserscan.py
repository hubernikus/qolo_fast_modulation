#!/usr/bin/env ipython3
"""
QOLO Pedestrian collision free navigation using modulation-algorithm and python.
"""
# Author: Lukas Huber
# Created: 2021-12-15
# Email: lukas.huber@epfl.ch

import os
import sys

import warnings
import signal

import copy
import time

from timeit import default_timer as timer

import numpy as np
from numpy import linalg as LA

from threading import Lock
lock = Lock()

if (sys.version_info < (3, 0)):
    from itertools import izip as zip
    
print("Run python {}".format(sys.version_info))

import rospy

try:
    import rospkg
except:
    print("Could not import critical rospackages.")
    raise

from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import LaserScan

from vartools.states import ObjectPose
    
try:
    # Check if module is installed
    from fast_obstacle_avoidance.control_robot import QoloRobot
    
except ModuleNotFoundError:
    # rospack = rospkg.RosPack()
    # Add obstacle avoidance without 'setting' up
    # directory_path = rospack.get_path('qolo_fast_modulation')
    directory_path =  '/home/qolo/autonomy_ws/src/qolo_fast_modulation'

    path_avoidance = os.path.join(
        directory_path, "scripts", "fast_obstacle_avoidance")
    if not path_avoidance in sys.path:
        sys.path.append(path_avoidance)

    from fast_obstacle_avoidance.control_robot import QoloRobot

# Custom libraries
from fast_obstacle_avoidance.obstacle_avoider import FastLidarAvoider
from fast_obstacle_avoidance.utils import laserscan_to_numpy


DEBUG_FLAG = True

class ControllerQOLO:
    # MAX_ANGULAR_SPEED = 0.6      # rad/s
    # MAX_SPEED = 0.65    # m/s
    MAX_ANGULAR_SPEED = 0.3      # rad/s
    MAX_SPEED = 0.3    # m/s

    dimension = 2
    
    def __init__(self):
        rospy.init_node('qolo_controller')

        # Create ctrl-c handler
        signal.signal(signal.SIGINT, self.control_c_handler)

        # Angular velocity difference
        self.diff_angular = 0

        # Shutdown variable
        self.shutdown_finished = False

    def control_c_handler(self, sig, frame):
        """ User defined handling of ctrl-c"""
        print('\nCaught ctrl-c by user. Shutdown is initiated ...')
        self.shutdown()
        rospy.signal_shutdown('Caught ctrl-c by user. Shutdown is initiated ...')

    def shutdown(self):
        """ User defined shutdown command."""

        print("\nDoing shutdown.")
        if self.shutdown_finished:
            return

        # Published repeated zero velocity to ensure stand-still
        for ii in range(10):
            self.publish_command(0, 0)
            rospy.sleep(0.01) # ? why error...

        self.shutdown_finished = True
        print("\nShutdown successful.")
        
    def controller_robot(self, vel_desired):
        # Inverse kinematics with decomposed jacobian
        command_linear, command_angular = LA.inv(self.Jacobian) @ vel_desired
        return command_linear, command_angular

    def controller_robot_old(self, vel_desired):
        """ Convert dynamical system into robot command. 
            P-D controller in angular direction. """
        command_linear = np.linalg.norm(vel_desired)

        diff_angular_old = copy.deepcopy(self.diff_angular)
        self.diff_angular = np.arctan2(vel_desired[1], vel_desired[0])

        if self.diff_angular > max_delta_angle:
            command_linear = -command_linear

            if self.diff_angular>0:
                self.diff_angular = self.diff_angular - np.pi
            else:
                self.diff_angular = np.pi + self.diff_angular

        # P-controller
        p_angular = 1./self.loop_dt * p_angular
        command_angular = self.diff_angular*p_angular

        if abs(command_angular) > (2*self.MAX_ANGULAR_SPEED):
            # Only rotate in this scenario
            command_angular = np.copysign(self.MAX_ANGULAR_SPEED, command_angular)
            command_linear = 0

        return command_linear, command_angular

        
    def publish_command(self, command_linear, command_angular):
        """  Command to QOLO motor [Real Implementation]. 
        Includes MASTER CHECK if linear/angular velocity reached limit
        """
        if np.abs(command_linear) > self.MAX_SPEED:
            # warnings.warn("Max linear velocity exceeded.")
            # Rospy.Logwarn("Max linear velocity exceeded.")
            command_linear = np.copysign(self.MAX_SPEED, command_linear)

        if np.abs(command_angular) > self.MAX_ANGULAR_SPEED:
            # warnings.warn("Max angular velocity exceeded.")
            # rospy.logwarn("Max angular velocity exceeded.")
            command_angular = np.copysign(self.MAX_ANGULAR_SPEED, command_angular)

        # Use 'time' since 'rospy.time' is a too large float
        msg_time = round(time.perf_counter(), 4)
        
        data_remote = Float32MultiArray()
        data_remote.layout.dim.append(MultiArrayDimension())
        data_remote.layout.dim[0].label = 'Trajectory Commands [Time, V, W]'
        data_remote.layout.dim[0].size = 3
        data_remote.data = [msg_time, command_linear, command_angular]
        
        self.pub_qolo_command.publish(data_remote)


class ControllerSharedLaserscan(ControllerQOLO):
    def callback_laserscan(self, msg, topic_name):
        with lock:
            self.qolo.set_laserscan(msg, topic_name=topic_name)
            
    def callback_remote(self, msg, tranform_to_global_frame=False):
        """ Get remote message and return velocity in global frame. """
        # Immediate republish
        with lock:
            (msg_time, command_linear, command_angular) = msg.data
            # print(f"Got linear={command_linear} | angular={command_angular}")

            if self.qolo.control_points.shape[1] > 1:
                raise NotImplementedError()
            
            ii = 0
            # velocity = np.array([
                # command_linear, command_angular*self.qolo.control_points[0, ii]
                # ])
            velocity = self.Jacobian @ np.array([command_linear, command_angular])

            if tranform_to_global_frame:
                breakpoint()
                velocity = self.agent.transform_relative2global_dir(velocity)
                
            self.remote_velocity_local = velocity

    def __init__(self, loop_rate: float = 10):
        """ Setup the laserscan controller."""
        super().__init__()
        
        self.loop_rate = loop_rate
        self.loop_dt = 1./self.loop_rate
        self.rate = rospy.Rate(self.loop_rate) # Hz

        self.msg_pose = None
        self.msg_laserscan_front = None
        self.msg_laserscan_rear = None

        self.remote_velocity = None

        self.remote_velocity_local = np.zeros(self.dimension)

        # QOLO State and geometry definition
        self.qolo = QoloRobot(
            pose=ObjectPose(position=[0.0, 0.0], orientation=00*np.pi/180)
        )

        self.Jacobian = np.diag([1,  self.qolo.control_points[0, 0]])

        ##### Subscriber #####
        # Since everthing is in the local frame. This is not needed
        # self.sub_qolo_pose2D = rospy.Subscriber(
            # '/qolo/pose2D', Pose2D, self.callback_qolo_pose2D)

        # Jostick input
        self.sub_remote = rospy.Subscriber('qolo/user_commands',
                                           Float32MultiArray, self.callback_remote)

        topic_rear_scan = '/rear_lidar/scan'
        self.sub_laserscan_rear = rospy.Subscriber(
            topic_rear_scan, LaserScan, self.callback_laserscan, topic_rear_scan)
            
        topic_front_scan = '/front_lidar/scan'
        self.sub_laserscan_front = rospy.Subscriber(
            topic_front_scan, LaserScan, self.callback_laserscan, topic_front_scan)
            

        ##### Publisher #####
        self.pub_qolo_command = rospy.Publisher(
            'qolo/remote_commands', Float32MultiArray, queue_size=1)

        # Define avoider object
        self.fast_avoider = FastLidarAvoider(robot=self.qolo)

        if DEBUG_FLAG:
            from debug_visualization_animator import DebugVisualizer
            self.visualizer = DebugVisualizer(main_controller=self, robot=self.qolo)
        
    def run(self):
        while (len(self.qolo.laser_data) != len(self.qolo.laser_poses)
               and not rospy.is_shutdown()
               ):
            print("Awaiting first messages...")

            if len(self.qolo.laser_data) != len(self.qolo.laser_poses):
                print("Waiting for first scans.")

                self.rate.sleep()
        
        self.it_count = 0

        # Starting main loop
        print("\nStarting looping")
        while not rospy.is_shutdown():
            # Start with sleep for initialization & error in case of shutdown
            self.rate.sleep()
            
            with lock:
                # TODO: check if laserscan has been updated
                if self.qolo.has_newscan:
                    self.fast_avoider.update_laserscan(self.qolo.get_allscan())

                modulated_velocity = self.fast_avoider.avoid(self.remote_velocity_local)
                # modulated_velocity = self.remote_velocity_local
                
                command_linear, command_angular = self.controller_robot(modulated_velocity)
                
                print('lin= {},   ang= {}'.format(command_linear, command_angular))
                self.publish_command(command_linear, command_angular)

                if DEBUG_FLAG:
                    self.visualizer.update_step(
                        ii=self.it_count,
                        initial_velocity=self.remote_velocity_local,
                        modulated_velocity=modulated_velocity)
                
            self.it_count += 1


if (__name__)=="__main__":
    print("Trying.")
    print("Setting up controller.")
    
    main_controller = ControllerSharedLaserscan()

    print("Starting controller.")
    main_controller.run()


    print("\nLet's call it a day and go home.\n")
