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

import tf

# import tf2_ros

# TODO: this should be tf2...

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

        self.qolo.pose.position = np.array([msg.x, msg.y])
        self.qolo.pose.orientation = msg.theta

        self.msg_qolo_pose = msg

    def update_pose_using_tf(self, orientation_init=0, position_init=0):
        """Get pose of agent by looking up transform."""
        try:
            (agent_position, agent_orientation) = self.tf_listener.lookupTransform(
                "/tf_qolo_world", "/tf_qolo", rospy.Time(0)
            )

            euler_angels_agent = tf.transformations.euler_from_quaternion(
                agent_orientation
            )

        except:
            print("Transform not found.")
            # Connection not found error
            return 404

        self.qolo.pose.position = np.array([agent_position[0], agent_position[1]])
        # Somehow angle is opposite my estimate
        # self.qolo.pose.orientation = euler_angels_agent[2] * (-1)
        self.qolo.pose.orientation = euler_angels_agent[2]

        # print(f"Robot orientation: {self.qolo.pose.orientation / np.pi * 180:.1f}")
        self.awaiting_pose = False

        # # Adapt with initial offset
        # if orientation_init:
        #     cos = np.cos(orientation_init)
        #     sin = np.sin(orientation_init)
        #     ROT_INIT = np.array([[cos, sin], [-sin, cos]])

        #     self.agent.position = ROT_INIT.T.dot(self.agent_position) + position_init
        #     # self.agent_position = self.agent_position

        # self.agent.orientation = self.agent.orientation + orientation_init

        # # Everything worked fine
        # return 0

    def __init__(
        self,
        loop_rate: float = 100,
        maximum_velocity: float = 0.3,
        sample_weight_factor: float = None,
        use_tracker: bool = False,
        algotype: AlgorithmType = AlgorithmType.SAMPLED,
        linear_command_scale: float = 1.0,
        relative_attractor_position: np.ndarray = None,
    ):
        """Setup the laserscan controller."""
        if DEBUG_FLAG_VISUALIZE:
            # The matplotlib will be too slow otherwise
            loop_rate = 0.5

        # Don't publish when visualize is on (since this is only on laptop computer)
        super().__init__(
            do_publish_command=not (DEBUG_FLAG_VISUALIZE), loop_rate=loop_rate
        )

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

        self.maximum_velocity = maximum_velocity

        # self.Jacobian = np.diag([1,  self.qolo.control_points[0, 0]])
        # Increased influence of angular velocity
        # self.RemoteJacobian = np.diag([1, 0.15])

        ##### Subscriber #####
        # Since everthing is in the local frame. This is not needed
        self.tf_listener = tf.TransformListener()
        # self.tfBuffer = tf2_ros.Buffer()
        # self.listener = tf2_ros.TransformListener(tfBuffer)

        if relative_attractor_position is None:
            # Jostick input
            self.sub_remote = rospy.Subscriber(
                "qolo/user_commands", Float32MultiArray, self.callback_remote
            )
            self.initial_dynamics = None

        else:
            if True:
                raise NotImplementedError(
                    "Absolute position currently has navigation errors."
                )
            self.awaiting_pose = True
            self.qolo.pose = ObjectPose(np.zeros(2), 0)

            while self.awaiting_pose:
                self.update_pose_using_tf()
                print("Waiting for first pose.")
                self.rate.sleep()

            # self.sub_qolo_pose2D = rospy.Subscriber(
            #     "/qolo/pose2D",
            #     Pose2D,
            #     self.callback_qolo_pose2D,
            # )

            # while self.awaiting_pose and not rospy.is_shutdown():
            #     print("Waiting for first pose.")
            #     self.rate.sleep()

            self.initial_dynamics = LinearSystem(
                attractor_position=self.qolo.pose.transform_position_from_relative(
                    relative_attractor_position
                ),
                # maximum_velocity=1.0,
                maximum_velocity=0.2,
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

            self.fast_avoider.weight_power = 2.0

            # Scale for spare crowd
            if sample_weight_factor is None:
                # No scale for doorpasssing
                # self.fast_avoider.weight_factor = 2 * np.pi / 1000 * 1
                self.fast_avoider.weight_factor = 2 * np.pi / 1000 * 5
            else:
                self.fast_avoider.weight_factor = sample_weight_factor

            print("sample weight", self.fast_avoider.weight_factor)

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
        print_int = max(int(self.loop_rate / print_freq), 1)

        t_sum = 0

        # Starting main loop
        print("\nStarting looping")
        while not rospy.is_shutdown():
            # Start with sleep for initialization & error in case of shutdown
            self.rate.sleep()

            with lock:
                # TODO: check if laserscan has been updated
                if self.initial_dynamics is not None:
                    self.update_pose_using_tf()

                    # Update velocity if an intial DS is given; otherwise take from remote
                    self.remote_velocity_local = self.initial_dynamics.evaluate(
                        self.qolo.pose.position
                    )
                    self.remote_velocity_local = (
                        self.qolo.pose.transform_direction_to_relative(
                            self.remote_velocity_local
                        )
                    )

                t_start = timer()
                # if self.qolo.has_newscan and True:
                if self.qolo.has_newscan:
                    self.fast_avoider.update_reference_direction(
                        self.qolo.get_allscan()
                    )

                modulated_velocity = self.fast_avoider.avoid(self.remote_velocity_local)

                if self.maximum_velocity:
                    if (
                        vel_norm := LA.norm(modulated_velocity)
                    ) > self.maximum_velocity:
                        modulated_velocity = (
                            modulated_velocity / vel_norm * self.maximum_velocity
                        )

                print("init vel", self.remote_velocity_local)
                print("mod vel", modulated_velocity)

                t_end = timer()
                t_sum += t_end - t_start
                # print(f"Ellapsed time: {(t_end-t_start)*1000:.2f}ms")

                command_linear, command_angular = self.controller_robot(
                    modulated_velocity
                )

                # [WARNING] Command gets multiplied -
                # BUT this is useful for joystick vs chest-control (not same calibration)
                command_linear *= self.linear_command_scale

                if not self.it_count % print_int:
                    print(
                        f"linear: {command_linear:.3f} | angular: {command_angular:.3f}"
                    )
                    print("Initial", self.remote_velocity_local)
                    print("Modulated", modulated_velocity)
                    print(f"Average ellapsed time: {t_sum/print_int*1000:.3f}ms")
                    t_sum = 0  # Reset value

                if self.do_publish_command:
                    # DO not publish when visialize - debug
                    # print(
                    #     f"[PRE-PUBLISH] {command_linear:.2f} - f{command_angular:.2f}"
                    # )
                    self.publish_command(command_linear, command_angular)

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

    print("Setting up controller...")

    # Set the flags (this could be transformed to class parameters)
    DEBUG_FLAG_VISUALIZE = args.visualize
    DEBUG_FLAG_PUBLISH = args.publish

    main_controller = ControllerSharedLaserscan(
        loop_rate=10,
        maximum_velocity=0.3,
        use_tracker=args.tracker,
        linear_command_scale=args.scale,
        algotype=AlgorithmType.SAMPLED,
        # sample_weight_factor=2 * np.pi / 1000 * 10,  # Weight for dynamic
        # sample_weight_factor=2 * np.pi / 1000 * 1e-4,  # Almost none
        sample_weight_factor=2 * np.pi / 1000 * 1,  # Weight for door-passing
        # algotype=AlgorithmType.VFH,
    )

    print("Starting controller.")
    main_controller.run()

    print("\nLet's call it a day and go home.\n")
