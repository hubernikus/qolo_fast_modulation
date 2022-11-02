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
from geometry_msgs.msg import Pose2D

from vartools.states import ObjectPose
from vartools.dynamical_systems import LinearSystem


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


class ControllerSharedLaserscan(ControllerQOLO):
    def callback_laserscan(self, msg, topic_name):
        with lock:
            self.last_laserscan_time = msg.header.stamp
            self.qolo.set_laserscan(msg, topic_name=topic_name)

    def callback_remote(self, msg, tranform_to_global_frame=False):
        """Get remote message and return velocity in global frame."""
        # Immediate republish
        with lock:
            (msg_time, command_linear, command_angular) = msg.data
            # print(f"Got linear={command_linear} | angular={command_angular}")

            if self.qolo.control_points.shape[1] > 1:
                raise NotImplementedError()

            # ii = 0
            # velocity = np.array([
            # command_linear, command_angular*self.qolo.control_points[0, ii]
            # ])
            velocity = self.RemoteJacobian @ np.array([command_linear, command_angular])
            # print(f"linear {command_linear} --- angular = {command_angular}")
            # print(f"velocity: {velocity}")

            # print('time', msg_time)
            if tranform_to_global_frame:
                breakpoint()
                velocity = self.agent.transform_relative2global_dir(velocity)

            self.remote_velocity_local = velocity

    def callback_qolo_pose2D(self, msg: Pose2D):
        # TODO: this should update the real-agent (!)

        if self.awaiting_pose:
            self.awaiting_pose = False

        self.qolo_pose.position = np.array([msg.x, msg.y])
        self.qolo_pose.orientation = msg.theta

    def __init__(
        self,
        loop_rate: float = 200,
        use_tracker: bool = False,
        algotype: AlgorithmType = AlgorithmType.SAMPLED,
        linear_command_scale: float = 1.0,
        relative_attractor_position: np.ndarray = None,
    ):
        """Setup the laserscan controller."""
        # Don't publish when visualize is on (since this is only on laptop computer)
        super().__init__(do_publish_command=not (DEBUG_FLAG_VISUALIZE), loop_rate=10)

        self.loop_rate = loop_rate
        self.loop_dt = 1.0 / self.loop_rate
        self.rate = rospy.Rate(self.loop_rate)  # Hz

        # Scale linear velocity command
        self.linear_command_scale = linear_command_scale

        self.msg_pose = None
        self.msg_laserscan_front = None
        self.msg_laserscan_rear = None

        self.remote_velocity = None

        self.remote_velocity_local = np.zeros(self.dimension)

        # QOLO State and geometry definition
        self.qolo = QoloRobot(
            pose=ObjectPose(position=[0.0, 0.0], orientation=00 * np.pi / 180)
        )

        # self.Jacobian = np.diag([1,  self.qolo.control_points[0, 0]])
        # Increased influence of angular velocity
        # self.RemoteJacobian = np.diag([1, 0.15])

        ##### Subscriber #####
        # Since everthing is in the local frame. This is not needed

        if relative_attractor_position is None:
            # Jostick input
            self.sub_remote = rospy.Subscriber(
                "qolo/user_commands", Float32MultiArray, self.callback_remote
            )
            self.initial_dynamics = None

        else:
            self.awaiting_pose = True
            self.qolo_pose = ObjectPose(np.zeros(2), 0)

            self.sub_qolo_pose2D = rospy.Subscriber(
                "/qolo/pose2D",
                Pose2D,
                self.callback_qolo_pose2D,
            )

            while self.awaiting_pose and not rospy.is_shutdown():
                print("Waiting for first scans.")
                self.rate.sleep()

            self.initial_dynamics = LinearSystem(
                attractor_position=self.qolo_pose.transform_position_from_relative(
                    relative_attractor_position
                ),
                # maximum_velocity=1.0,
                maximum_velocity=0.3,
            )

        topic_rear_scan = "/rear_lidar/scan"
        self.sub_laserscan_rear = rospy.Subscriber(
            topic_rear_scan, LaserScan, self.callback_laserscan, topic_rear_scan
        )

        topic_front_scan = "/front_lidar/scan"
        self.sub_laserscan_front = rospy.Subscriber(
            topic_front_scan, LaserScan, self.callback_laserscan, topic_front_scan
        )

        if use_tracker:
            algotype = AlgorithmType.MIXED

        if algotype == AlgorithmType.MIXED:
            from pedestrian_caller import RealPedestrianSubscriber

            self.pedestrian_subscriber = RealPedestrianSubscriber(
                lock=lock, robot=self.qolo
            )

            from fast_obstacle_avoidance.obstacle_avoider import MixedEnvironmentAvoider

            self.fast_avoider = MixedEnvironmentAvoider(robot=self.qolo)

        elif algotype == AlgorithmType.SAMPLED:
            # Define avoider object
            self.fast_avoider = FastLidarAvoider(robot=self.qolo, evaluate_normal=False)
            self.fast_avoider.weight_factor = 2 * np.pi / 1000
            # self.fast_avoider.weight_factor = 2 * np.pi / 1000
            self.fast_avoider.weight_power = 2.0

        elif algotype == AlgorithmType.VFH:
            # Define avoider object
            self.fast_avoider = VFH_Avoider(robot=self.qolo)

        else:
            warnings.warn("Desired algorithm is not implemented.")
            return

        print(f"[INFO] Algorithm Type: {algotype}")

        if DEBUG_FLAG_VISUALIZE:
            from debug_visualization_animator import DebugVisualizer

            self.visualizer = DebugVisualizer(main_controller=self, robot=self.qolo)

        if DEBUG_FLAG_PUBLISH:
            from debug_publisher import DebugPublisher

            self.debug_publisher = DebugPublisher(main_controller=self)

    def run(self):
        while (
            len(self.qolo.laser_data) != len(self.qolo.laser_poses)
            and not rospy.is_shutdown()
        ):
            print("Awaiting first messages...")

            if len(self.qolo.laser_data) != len(self.qolo.laser_poses):
                print("Waiting for first scans.")
                self.rate.sleep()

        self.it_count = 0

        print_freq = 4
        print_int = int(self.loop_rate / print_freq)

        # t_sum = 0

        # Starting main loop
        print("\nStarting looping")
        while not rospy.is_shutdown():
            # Start with sleep for initialization & error in case of shutdown
            self.rate.sleep()

            with lock:
                # TODO: check if laserscan has been updated
                if self.initial_dynamics is not None:
                    # Update velocity if an intial DS is given; otherwise take from remote
                    self.remote_velocity_local = self.initial_dynamics.evaluate(
                        self.qolo_pose.position
                    )
                    self.remote_velocity_local = (
                        self.qolo_pose.transform_direction_to_relative(
                            self.remote_velocity_local
                        )
                    )

                # t_start = timer()
                # if self.qolo.has_newscan and True:
                if self.qolo.has_newscan:
                    self.fast_avoider.update_reference_direction(
                        self.qolo.get_allscan()
                    )

                modulated_velocity = self.fast_avoider.avoid(self.remote_velocity_local)

                # t_end = timer()

                # t_sum += (t_end - t_start)
                # print(f"Ellapsed time: {(t_end-t_start)*1000}ms")
                # if not self.it_count % print_int:
                # print(f"Average ellapsed time: {t_sum/print_int*1000}ms")
                # t_sum = 0     # Reset value

                command_linear, command_angular = self.controller_robot(
                    modulated_velocity
                )
                # [WARNING] Command gets multiplied
                command_linear *= self.linear_command_scale

                if not self.it_count % print_int:
                    print("lin= {},   ang= {}".format(command_linear, command_angular))

                if self.do_publish_command:
                    # DO not publish when visialize - debug
                    # print(
                    #     f"[PRE-PUBLISH] {command_linear:.2f} - f{command_angular:.2f}"
                    # )
                    self.publish_command(command_linear, command_angular)
                    # breakpoint()

                print("Qolor Pose", self.qolo.pose)

                if DEBUG_FLAG_VISUALIZE:
                    if not self.visualizer.figure_is_open:
                        self.shutdown()
                        continue

                    self.visualizer.update_step(
                        ii=self.it_count,
                        initial_velocity=self.remote_velocity_local,
                        modulated_velocity=modulated_velocity,
                        ros_time=rospy.get_time(),
                    )

                if DEBUG_FLAG_PUBLISH:
                    self.debug_publisher.update_step(
                        initial_velocity=self.remote_velocity_local,
                        modulated_velocity=modulated_velocity,
                        msg_time=self.last_laserscan_time,
                    )
            self.it_count += 1


class FloatOfRange(float):
    """Checks if the float value is in the given range and returns it."""

    def __init__(self, minimum=None, maximum=None, infimum=None, supremum=None):
        self.minimum = minimum
        self.maximum = maximum
        self.infimum = infimum
        self.supremum = supremum

    def __call__(self, value):
        value = float(value)
        if self.infimum is not None:
            if value <= self.infimum:
                raise Exception(
                    f"Input {value} is smaller than the infimum of {self.infimum}"
                )

        if self.minimum is not None:
            if value < self.minimum:
                raise Exception(
                    f"Input {value} is smaller than the minimum of {self.minimum}"
                )

        if self.supremum is not None:
            if value >= self.supremum:
                raise Exception(
                    f"Input {value} is larger than the supremum of {self.supremum}"
                )

        if self.maximum is not None:
            if value > self.maximum:
                raise Exception(
                    f"Input {value} is larger than the maximum of {self.maximum}"
                )

        return value


if (__name__) == "__main__":
    # First Parse input arguments (don't pollute the `--help` messsage)
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-t",
        "--tracker",
        action="store_true",
        help="Use tracker for avoidance [default false]",
    )
    parser.add_argument(
        "-v",
        "--visualize",
        action="store_true",
        help="Visualize using matplotlib (!) only for debug puposes, "
        + "since this might significantly slow down the calculation no control command is send..",
    )
    parser.add_argument(
        "-p",
        "--publish",
        action="store_true",
        help="Additionally publish velocity commands, only for debug puposes.",
    )
    parser.add_argument(
        "-s",
        "--scale",
        # type=float,
        type=FloatOfRange(infimum=0, maximum=1.5),
        default=1.0,
        help="Scale velocity input by float command "
        + "(a scale of 1.5 is adviced for the joystick).",
    )
    args = parser.parse_args()

    print("Trying.")
    print("Run python {}".format(sys.version_info))

    print("Setting up controller.")

    # Set the flags (this could be transformed to class parameters)
    DEBUG_FLAG_VISUALIZE = args.visualize
    DEBUG_FLAG_PUBLISH = args.publish

    main_controller = ControllerSharedLaserscan(
        use_tracker=args.tracker,
        linear_command_scale=args.scale,
        algotype=AlgorithmType.SAMPLED,
        # algotype=AlgorithmType.VFH,
        relative_attractor_position=np.array([4, -4]),
    )

    print("Starting controller.")
    main_controller.run()

    print("\nLet's call it a day and go home.\n")
