#!/usr/bin/env python3
"""
QOLO Pedestrian collision free navigation using modulation-algorithm and python.
"""
# Author: Lukas Huber
# Created: 2021-12-15
# Email: lukas.huber@epfl.ch

import numpy as np
from numpy import linalg as LA

import rospy

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose2D

# Needs python2 (...)
import tf2_ros

from vartools.dynamical_systems import DynamicalSystem, LinearSystem

from ._base_controller import ControllerQOLO


class ControllerBasicDS(ControllerQOLO):
    def __init__(self, dynamical_system: DynamicalSystem, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        
        self.dynamical_system = dynamical_system

        # Increased influence of angular velocity
        #    (otherwise avoidance will be difficult)
        self.remote_velocity_local = np.zeros(self.dimension)

        # Jostick input
        self.sub_remote = rospy.Subscriber(
            'qolo/user_commands',
            Float32MultiArray, self.callback_remote)

        self.qolo_pose = Pose2D()
        # TODO: get updated pose of QOLO (subscriber or TF listener)

    def run(self):
        self.it_count = 0
        
        # Starting main loop
        while not rospy.is_shutdown():
            # Start with sleep for initialization & error in case of shutdown
            # (!) important to have this outside of lock
            self.rate.sleep()

            # Get new pose
            self.get_qolo_transform()
            
            with self.lock:
                ########## Evaluate the velocity in the local frame here ##########
                
                desired_velocity = self.dynamical_system.evaluate(
                    np.array([self.qolo_pose.x, self.qolo_pose.y])
                )
                # print('desired_velocity', desired_velocity)
                desired_velocity = self.transform_velocity_from_world_to_robot(
                    desired_velocity)
                
                ########## End custom code ##########
                
                command_linear, command_angular = self.controller_robot(
                    desired_velocity)

                print(desired_velocity)
                self.publish_command(command_linear, command_angular)
                
            self.it_count += 1

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

def log_print(text, log_name="[qolo_controller] "):
    print(log_name + text)


if (__name__)=="__main__":
    log_print("Startup...")
    dynamical_system = LinearSystem(
        attractor_position=np.array([0, 2.0]),
        maximum_velocity=0.8,
    )
    
    my_controller = ControllerBasicDS(dynamical_system)
    
    log_print("Initialization finished.")
    my_controller.run()

    print()
    log_print("Execution stopped.")
