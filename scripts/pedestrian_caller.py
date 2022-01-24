""" Only subscribes to the lock
"""
# Author: Lukas Huber
# Created: 2022-01-04
# Email: lukas.huber@epfl.ch
# License: BSD (c) 2022

import numpy as np

import rospy

from qolo_fast_modulation.msg import TrackedPersons
from dynamic_obstacle_avoidance.containers import ObstacleContainer()

class RealPedestrianSubscriber:
    """
    Attributes `obstacle_enviroment` and `lock` are references.
    """
    def __init__(
        self, lock, obstacle_enviroment=None,
        human_radius=0.35, margin_absolut=0, dist_far
    ):
        self.lock = lock
        
        if obstacle_enviroment is None:
            self.obstacle_enviroment = ObstacleContainer()
        else:
            self.obstacle_enviroment = obstacle_enviroment
        

        self.human_radius = human_radius
        self.margin_absolut = margin_absolut

        self.sub_tracker = rospy.Subscriber(
            'rwth_tracker/tracked_persons', TrackedPersons, self.callback_tracker)

    def callback_tracker(self, msg):
        ''' Tracker which updates obstacle list and assigns new reference points'''
        with self.lock:
            if self.awaiting_crowd:
                self.awaiting_crowd = False

            self.update_obstacle_list(msg.detections)
            
    
