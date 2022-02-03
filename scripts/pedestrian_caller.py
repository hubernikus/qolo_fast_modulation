""" Only subscribes to the lock
"""
# Author: Lukas Huber
# Created: 2022-01-04
# Email: lukas.huber@epfl.ch
# License: BSD (c) 2022

import numpy as np

import rospy

from qolo_msgs.msg import TrackedPersons
from dynamic_obstacle_avoidance.containers import ObstacleContainer


class RealPedestrianSubscriber:
    """
    Attributes `obstacle_enviroment` and `lock` are references.
    """
    def __init__(
        self, lock, robot, dist_far=None
    ):
        # Shared lock with main node
        self.lock = lock
        
        # Everything is happening in the robot
        self.robot = robot

        self.sub_tracker = rospy.Subscriber(
            'rwth_tracker/tracked_persons', TrackedPersons, self.callback_tracker)

    def callback_tracker(self, msg):
        ''' Tracker which updates obstacle list and assigns new reference points'''
        with self.lock:
            print(f"Recieved numer of obstacles: {len(msg.tracks)}")
            self.robot.set_crowdtracker(msg)
