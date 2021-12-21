#!/usr/bin/env python2
"""
QOLO Pedestrian collision free navigation using modulation-algorithm and python.
"""
# Author: Lukas Huber
# Created: 2021-12-15
# Email: lukas.huber@epfl.ch

import os
import sys

import logging
import warnings
import signal

import copy
import time

from timeit import default_timer as timer

import numpy as np

from threading import Lock
lock = Lock()

if (sys.version_info < (3, 0)):
    from itertools import izip as zip
    
print("Run python {}".format(sys.version_info))

import rospy

try:
    import rospkg
    # import tf
except:
    print("Could not import critical rospackages.")
    raise

from std_msgs.msg import Float32MultiArray, MultiArrayDimension
# from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan

from vartools.states import ObjectPose
    
    
# from std_msgs.msg import MultiArrayLayout, MultiArrayDimension
# from geometry_msgs.msg import Twist, TwistStamped, Pose2D


try:
    from fast_obstacle_avoidance.control_robot import ControlRobot
    
except:
    # rospack = rospkg.RosPack()
    # Add obstacle avoidance without 'setting' up
    # directory_path = rospack.get_path('qolo_fast_modulation')
    directory_path =  '/home/qolo/autonomy_ws/src/qolo_fast_modulation'

    path_avoidance = os.path.join(
        directory_path, "scripts", "fast_obstacle_avoidance")
    if not path_avoidance in sys.path:
        sys.path.append(path_avoidance)

    from fast_obstacle_avoidance.control_robot import ControlRobot

# Custom libraries
from fast_obstacle_avoidance.obstacle_avoider import FastObstacleAvoider
from fast_obstacle_avoidance.utils import laserscan_to_numpy


class ControllerQOLO:
    # MAX_ANGULAR_SPEED = 0.6      # rad/s
    # MAX_SPEED = 0.65    # m/s
    MAX_ANGULAR_SPEED = 0.3      # rad/s
    MAX_SPEED = 0.3    # m/s

    delta_angle_rear = np.pi
    delta_position_rear = np.array([0.75, 0.0])

    dimension = 2
    
    def __init__(self):
        rospy.init_node('qolo_controller')

        # Create ctrl-c handler
        signal.signal(signal.SIGINT, self.control_c_handler)

        # Angular velocity difference
        self.diff_angular = 0

        # Shutdown variable
        self.shutdown_finished = False

    
    def callback_laserscan_front(self, msg):
        with lock:
            self.msg_laserscan_front = laserscan_to_numpy(msg)

    def callback_laserscan_rear(self, msg):
        with lock:
            self.msg_laserscan_rear = laserscan_to_numpy(
                msg, delta_angle=self.delta_angle_rear,
                delta_position=self.delta_position_rear
            )

    def control_c_handler(self, sig, frame):
        """ User defined handling of ctrl-c"""
        logging.warning('\nCaught ctrl-c by user. Shutdown is initiated ...')
        self.shutdown()
        rospy.signal_shutdown('Caught ctrl-c by user. Shutdown is initiated ...')

    def shutdown(self):
        """ User defined shutdown command."""

        logging.info("\nDoing shutdown.")
        if self.shutdown_finished:
            return

        # Published repeated zero velocity to ensure stand-still
        for ii in range(10):
            self.publish_command(0, 0)
            rospy.sleep(0.01) # ? why error...

        self.shutdown_finished = True
        logging.info("\nShutdown successful.")
        
    def controller_robot(self, vel_desired,
                         max_delta_angle=180./180*np.pi, p_angular=0.5, ):
                         
        ''' Convert dynamical system into robot command. 
            P-D controller in angular direction'''
        
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
            # rospy.logwarn("Max linear velocity exceeded.")
            command_linear = np.copysign(self.MAX_SPEED, command_linear)

        if np.abs(command_angular) > self.MAX_ANGULAR_SPEED:
            # warnings.warn("Max angular velocity exceeded.")
            # rospy.logwarn("Max angular velocity exceeded.")
            command_linear = np.copysign(self.MAX_ANGULAR_SPEED, command_angular)
        
        data_remote = Float32MultiArray()
        data_remote.layout.dim.append(MultiArrayDimension())
        data_remote.layout.dim[0].label = 'Trajectory Commands [V, W]'
        data_remote.layout.dim[0].size = 3
        data_remote.data = [0]*3

        # Use 'time' since 'rospy.time' is a too large float
        msg_time = round(time.perf_counter(), 4)

        data_remote.data = [msg_time, command_linear, command_angular]
        # warnings.warn("Not publishing here though..")
        self.pub_qolo_command.publish(data_remote)


class ControllerSharedLaserscan(ControllerQOLO):
    def callback_remote(self, msg, tranform_to_global_frame=False):
        """ Get remote message and return velocity in global frame. """
        # Immediate republish
        with lock:
            (msg_time, command_linear, command_angular) = msg.data

            if self.qolo.control_points.shape[1] > 1:
                raise NotImplementedError()
            ii = 0
            velocity = np.array([
                command_linear, command_angular*self.qolo.control_points[0, ii]
                ])

            if tranform_to_global_frame:
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
        self.qolo = ControlRobot(
            control_points=np.array([[0.035, 0],
                                     # [0.5, 0],
                                     ]).T,
            control_radiuses=np.array([0.5,
                                       # 0.4,
                                       ]),
            pose=ObjectPose(position=[0.0, 0.0], orientation=00*np.pi/180)
        )

        ##### Subscriber #####
        # Since everthing is in the local frame. This is not needed
        # self.sub_qolo_pose2D = rospy.Subscriber(
            # '/qolo/pose2D', Pose2D, self.callback_qolo_pose2D)

        # Jostick input
        self.sub_remote = rospy.Subscriber('qolo/user_commands',
                                           Float32MultiArray, self.callback_remote)

        self.sub_laserscan_rear = rospy.Subscriber('/rear_lidar/scan',
                                              LaserScan, self.callback_laserscan_rear)

        self.sub_laserscan_front = rospy.Subscriber('/front_lidar/scan',
                                                   LaserScan, self.callback_laserscan_front)

        ##### Publisher #####
        self.pub_qolo_command = rospy.Publisher(
            'qolo/remote_commands', Float32MultiArray, queue_size=1)

        # Define avoider object
        self.fast_avoider = FastObstacleAvoider(robot=self.qolo)
        

    def run(self):
        while ((self.msg_laserscan_front is None
                or self.msg_laserscan_rear is None)
               and not rospy.is_shutdown()
               ):
            logging.info("Awaiting first messages...")
            
            if self.msg_laserscan_front is None:
                logging.info("Waiting for front_scan.")

            if self.msg_laserscan_rear is None:
                logging.info("Waiting for rear_scan.")

            self.rate.sleep()
        
        self.it_count = 0

        # Starting main loop
        logging.info("\nStarting looping")
        while not rospy.is_shutdown():
            # Start with sleep for initialization & error in case of shutdown
            self.rate.sleep()
            
            with lock:
                # TODO: check if laserscan has been updated
                self.fast_avoider.update_laserscan(
                    np.hstack((self.msg_laserscan_rear, self.msg_laserscan_front)))

                modulated_velocity = self.fast_avoider.evaluate(self.remote_velocity_local)

                print()
                print('mod vel', modulated_velocity)
                command_linear, command_angular = self.controller_robot(modulated_velocity)
                print('lin= {},   ang= {}'.format(command_linear, command_angular))
                self.publish_command(command_linear, command_angular)
                
            self.it_count += 1


if (__name__)=="__main__":
    logging.basicConfig(
        level=logging.INFO,
        # filename='example.log',
        handlers=[logging.StreamHandler()]
        )

    logging.info("Setting up controller.")
    main_controller = ControllerSharedLaserscan()

    logging.info("Starting controller.")
    main_controller.run()


    print("\nLet's call it a day and go home.\n")
