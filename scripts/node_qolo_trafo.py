#!/usr/bin/env python3
"""
Translates velocity to qolo-transformation.
"""
# Author: Lukas Huber
# Created: 2021-12-20
# Email: lukas.huber@epfl.ch

import numpy as np
from numpy import linalg as LA

from scipy.spatial.transform import Rotation

from threading import Lock
lock = Lock()

import rospy

# from geometry_msgs.msg import Pose2D, TransformStamped
import geometry_msgs
import std_msgs

import tf2_ros


class QoloTransformPublisher:
    wheel_distance = 0.545
    
    def __init__(self, loop_rate=1000):
        rospy.init_node('tf2_turtle_broadcaster')
        
        self.sub_qolo_command = rospy.Subscriber(
            'qolo/remote_commands',
            std_msgs.msg.Float32MultiArray,
            self.callback_remote_command
        )

        self.dimension = 2
        self.qolo_pose2d = geometry_msgs.msg.Pose2D()

        # Velocities in local frame
        self.linear_velocity = 0
        self.angular_velocity = 0

        self.loop_rate = loop_rate
        self.loop_dt = 1./self.loop_rate
        self.rate = rospy.Rate(self.loop_rate) # Hz

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Frame id's
        self.header_frame_id = "world"
        self.child_frame_id = "tf_qolo"

    def callback_remote_command(self, msg):
        with lock:
            self.linear_velocity = msg.data[1]
            self.angular_velocity = msg.data[2]

    @property
    def rotation_matrix(self):
        cos_phi = np.cos(self.qolo_pose2d.theta)
        sin_phi = np.sin(self.qolo_pose2d.theta)
        
        return np.array([[cos_phi, (-1)*sin_phi],
                         [sin_phi, cos_phi]])
                        
    def euler_step(self, linear_velocity_local, angular_velocity, dt):
        linear_velocity = self.rotation_matrix @ np.array([linear_velocity_local, 0])

        self.qolo_pose2d.x = self.qolo_pose2d.x + linear_velocity[0]*dt
        self.qolo_pose2d.y = self.qolo_pose2d.y + linear_velocity[1]*dt
        self.qolo_pose2d.theta = self.qolo_pose2d.theta + angular_velocity*dt
        
    def RK4_step(self, pose, linear_velocity, angular_velocity, dt):
        # 3-dimension // 4 RK-stesps
        velocities = np.zeros((3, 4))

        velocities[:2, 0] = self.rotation_matrix @ linear_velocity_local
        velocities[2, 0] = angular_velocity

        raise NotImplementedError()
    
    def run(self):
        while not rospy.is_shutdown():
            # Start with sleep for initialization & error in case of shutdown
            self.rate.sleep()

            # All the rest is happening within the lock
            with lock:
                self.euler_step(
                    self.linear_velocity, self.angular_velocity, dt=self.loop_dt)
                
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = self.header_frame_id
                t.child_frame_id = self.child_frame_id
                t.transform.translation.x = self.qolo_pose2d.x
                t.transform.translation.y = self.qolo_pose2d.y
                t.transform.translation.z = 0.0

                q = Rotation.from_euler('z', self.qolo_pose2d.theta).as_quat()
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]

                self.tf_broadcaster.sendTransform(t)


if (__name__) == "__main__":
    print("[qolo_transformer] Startup ...")
    qolo_trafo = QoloTransformPublisher()
    
    print("[qolo_transformer] Succesfully initialized.")
    qolo_trafo.run()
    
    print()
    print("[qolo_transformer] Shutdown of finished.")
