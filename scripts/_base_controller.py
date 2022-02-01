#!/usr/bin/env python3
"""
QOLO Pedestrian collision free navigation using modulation-algorithm and python.
"""
# Author: Lukas Huber
# Created: 2021-12-15
# Email: lukas.huber@epfl.ch

import signal

from timeit import default_timer as timer
import time

import numpy as np
from numpy import linalg as LA

from scipy.spatial.transform import Rotation

from threading import Lock

import rospy

from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from geometry_msgs.msg import Pose2D

import tf2_ros


class ControllerQOLO:
    # Base variables
    MAX_ANGULAR_SPEED = 0.3      # rad/s
    MAX_SPEED = 0.3    # m/s

    dimension = 2
    
    def __init__(self, loop_rate: float = 100):
        self.lock = Lock()
        
        rospy.init_node('qolo_controller')

        # Create ctrl-c handler
        signal.signal(signal.SIGINT, self.control_c_handler)

        # Angular velocity difference
        self.diff_angular = 0

        # Shutdown variable
        self.shutdown_finished = False

        self.loop_rate = loop_rate
        self.loop_dt = 1./self.loop_rate
        self.rate = rospy.Rate(self.loop_rate) # Hz

        self.Jacobian = np.diag([1,  0.0625])
        # Increased influence of angular velocity
        self.RemoteJacobian = np.diag([1, 0.15])

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        ##### Publisher #####
        self.pub_qolo_command = rospy.Publisher(
            'qolo/remote_commands', Float32MultiArray, queue_size=1)

    def control_c_handler(self, sig, frame):
        """ User defined handling of ctrl-c"""
        print('\nCaught ctrl-c by user. Shutdown is initiated ...')
        self.shutdown()

    def shutdown(self):
        """ User defined shutdown command."""
        print("\nInitiating shutdown.")
        if self.shutdown_finished:
            return

        # Published repeated zero velocity to ensure stand-still
        for ii in range(10):
            self.publish_command(0, 0)
            rospy.sleep(0.01) # ? why error...

        self.shutdown_finished = True
        
        rospy.signal_shutdown('Caught ctrl-c by user. Shutdown is initiated ...')
        print("\nShutdown successful.")
        
    def controller_robot(self, vel_desired):
        # Inverse kinematics with decomposed jacobian
        command_linear, command_angular = LA.inv(self.Jacobian) @ vel_desired
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
        data_remote.layout.dim[0].label = 'FastMod Command [Time, V, W]'
        data_remote.layout.dim[0].size = 3
        data_remote.data = [msg_time, command_linear, command_angular]
        
        self.pub_qolo_command.publish(data_remote)

    def get_qolo_transform(self) -> int:
        """ Returns 0 in case of no-error."""
        try:
            trans = self.tf_buffer.lookup_transform(
                'world', 'tf_qolo', rospy.Time())

            quat = [trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w
                    ]
            
            theta = Rotation.from_quat(quat).as_euler('zyx')
            self.qolo_pose.theta = theta[0]
            
            self.qolo_pose.x = trans.transform.translation.x
            self.qolo_pose.y = trans.transform.translation.y

            return 0
            
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return 1

    def callback_remote(self, msg, transform_to_global_frame=False):
        """ Get remote message and return velocity in global frame. """
        # Immediate republish
        with self.lock:
            (msg_time, command_linear, command_angular) = msg.data
            if self.qolo.control_points.shape[1] > 1:
                raise NotImplementedError()
            
            velocity = (self.RemoteJacobian
                        @ np.array([command_linear, command_angular]))
            
            if transform_to_global_frame:
                breakpoint()
                velocity = self.agent.transform_relative2global_dir(velocity)
                
            self.remote_velocity_local = velocity

    def transform_velocity_from_world_to_robot(self, velocity):
        sin_val = np.sin(self.qolo_pose.theta)
        cos_val = np.cos(self.qolo_pose.theta)

        rot_matr = np.array([[cos_val, sin_val], [-sin_val, cos_val]])

        return rot_matr @ velocity
