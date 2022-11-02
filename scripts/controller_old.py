#!/usr/bin/env python3
"""
QOLO Pedestrian collision free navigation using modulation-algorithm and python.
"""
# Author: Lukas Huber
# Created: 2021-12-15
# Email: lukas.huber@epfl.ch

import os
import sys
from enum import Enum, auto
import warnings
import signal
import copy
import time

from timeit import default_timer as timer

import argparse

import numpy as np
from numpy import linalg as LA

from threading import Lock

lock = Lock()

if sys.version_info < (3, 0):
    from itertools import izip as zip

import rospy

try:
    import rospkg
except:
    print("Cannot import critical rospackages.")
    raise

from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import LaserScan

from vartools.states import ObjectPose


class AlgorithmType(Enum):
    SAMPLED = 0
    MIXED = 1
    VFH = 2
    OBSTACLE = auto()
    MODULATED = auto()
    CLUSTERSAMPLED = auto()


try:
    # Check if module is installed
    from fast_obstacle_avoidance.control_robot import QoloRobot

except ModuleNotFoundError:
    # rospack = rospkg.RosPack()
    # Add obstacle avoidance without 'setting' up
    # directory_path = rospack.get_path('qolo_fast_modulation')
    directory_path = "/home/qolo/autonomy_ws/src/qolo_fast_modulation"

    path_avoidance = os.path.join(directory_path, "scripts", "fast_obstacle_avoidance")
    if not path_avoidance in sys.path:
        sys.path.append(path_avoidance)

    from fast_obstacle_avoidance.control_robot import QoloRobot

# Custom libraries
from fast_obstacle_avoidance.obstacle_avoider import FastLidarAvoider
from fast_obstacle_avoidance.comparison.vfh_avoider import VFH_Avoider
from fast_obstacle_avoidance.utils import laserscan_to_numpy

from _base_controller import ControllerQOLO


class ControllerQOLO_INITIAL:
    # MAX_ANGULAR_SPEED = 0.6      # rad/s
    # MAX_SPEED = 0.65    # m/s
    # MAX_ANGULAR_SPEED = 0.3      # rad/s
    # MAX_SPEED = 0.3    # m/s

    dimension = 2

    def __init__(self):
        rospy.init_node("qolo_controller")

        # Create ctrl-c handler
        signal.signal(signal.SIGINT, self.control_c_handler)

        # Angular velocity difference
        self.diff_angular = 0

        # Shutdown variable
        self.shutdown_finished = False

        ##### Publisher #####
        self.pub_qolo_command = rospy.Publisher(
            "qolo/remote_commands", Float32MultiArray, queue_size=1
        )

    def control_c_handler(self, sig, frame):
        """User defined handling of ctrl-c"""
        print("\nCaught ctrl-c by user. Shutdown is initiated ...")
        self.shutdown()

    def shutdown(self):
        """User defined shutdown command."""

        print("\nInitiating shutdown.")
        if self.shutdown_finished:
            return

        # Published repeated zero velocity to ensure stand-still
        for ii in range(10):
            self.publish_command(0, 0)
            rospy.sleep(0.01)  # ? why error...

        self.shutdown_finished = True

        rospy.signal_shutdown("Caught ctrl-c by user. Shutdown is initiated ...")
        print("\nShutdown successful.")

    def controller_robot(self, vel_desired):
        # Inverse kinematics with decomposed jacobian
        command_linear, command_angular = LA.inv(self.Jacobian) @ vel_desired
        return command_linear, command_angular

    def controller_robot_old(self, vel_desired):
        """Convert dynamical system into robot command.
        P-D controller in angular direction."""
        command_linear = np.linalg.norm(vel_desired)

        diff_angular_old = copy.deepcopy(self.diff_angular)

        self.diff_angular = np.arctan2(vel_desired[1], vel_desired[0])

        if self.diff_angular > max_delta_angle:
            command_linear = -command_linear

            if self.diff_angular > 0:
                self.diff_angular = self.diff_angular - np.pi
            else:
                self.diff_angular = np.pi + self.diff_angular

        # P-controller
        p_angular = 1.0 / self.loop_dt * p_angular
        command_angular = self.diff_angular * p_angular

        if abs(command_angular) > (2 * self.MAX_ANGULAR_SPEED):
            # Only rotate in this scenario
            command_angular = np.copysign(self.MAX_ANGULAR_SPEED, command_angular)
            command_linear = 0

        return command_linear, command_angular

    def publish_command(self, command_linear, command_angular):
        """Command to QOLO motor [Real Implementation].
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
        data_remote.layout.dim[0].label = "FastMod Command [Time, V, W]"
        data_remote.layout.dim[0].size = 3
        data_remote.data = [msg_time, command_linear, command_angular]

        breakpoint()
        self.pub_qolo_command.publish(data_remote)
