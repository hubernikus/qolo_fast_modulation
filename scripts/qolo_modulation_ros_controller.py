#!/usr/bin/env python2
"""
QOLO Pedestrian collision free navigation using modulation-algorithm and python.
"""
# Author: Lukas Huber
# Created: 2020-08-28
# Email: lukas.huber@epfl.ch

import sys
import os
import json
import signal
import copy
import getopt  # Getopt to be compatible with python 2
import warnings

import time
import datetime

import numpy as np
from math import pi

try:
    import matplotlib.pyplot as plt
except ModuleNotFoundError:
    warnings.warn("Matplotlib not installed.")

from threading import Lock

lock = Lock()

if sys.version_info < (3, 0):
    from itertools import izip as zip

    print("Run python {}".format(sys.version_info))

import rospy

try:
    import rospkg
except ModuleNotFoundError:
    # Define exception
    raise

try:
    import tf
except ModuleNotFoundError:
    # Define exception
    raise

from std_msgs.msg import Float32MultiArray, String, Bool
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension
from geometry_msgs.msg import Twist, TwistStamped, Pose2D

# PoseWithCovariance
# from frame_msgs.msg import TrackedPersons
# from qolo_modulation.msg import TrackedPersons
from qolo_fast_modulation.msg import TrackedPersons

try:
    import dynamic_obstacle_avoidance
except ModuleNotFoundError:
    rospack = rospkg.RosPack()
    # rospack.list()
    # Add obstacle avoidance without 'setting' up

    # directory_path = rospack.get_path('qolo_modulation')
    directory_path = "/home/qolo/autonomy_ws/src/qolo_modulation"

    path_avoidance = os.path.join(
        directory_path, "scripts", "dynamic_obstacle_avoidance", "src"
    )
    if not path_avoidance in sys.path:
        sys.path.append(path_avoidance)

#     import pdb; pdb.set_trace()


from vartools.dynamical_systems import LinearSystem
from vartools.angle_math import angle_difference_directional

# from dynamic_obstacle_avoidance.dynamical_system.dynamical_system_representation import (
#     linear_ds_max_vel,
# )

from dynamic_obstacle_avoidance.containers import GradientContainer
from dynamic_obstacle_avoidance.obstacles import Obstacle, Ellipse, Polygon
from dynamic_obstacle_avoidance.obstacles.human_ellipse import TrackedPedestrian
from dynamic_obstacle_avoidance.avoidance import obs_avoidance_interpolation_moving

from dynamic_obstacle_avoidance.containers.crowd_learning_container import (
    CrowdLearningContainer,
    CrowdCircleContainer,
)
from dynamic_obstacle_avoidance.agents.agent_qolo import AgentQOLO

from dynamic_obstacle_avoidance.metric_evaluation import MetricEvaluator
from dynamic_obstacle_avoidance.visualization.vector_field_visualization import (
    Simulation_vectorFields,
)

# path_crowdsim = os.path.join("home", "crowdbot", "CrowdBot_simulator", "CrowdBotUnity", "catkin_crowdbot", "sim", "src", "crowdbotsim", "msg")
# path_crowdsim = "/home/crowdbot/CrowdBot_simulator/CrowdBotUnity/catkin_crowdbotsim/src/crowdbotsim/msg"
# if not path_crowdsim in sys.path:
#    sys.path.append(path_crowdsim)

# Set simulation
FLAG_DETECTOR_STATIC = True
if FLAG_DETECTOR_STATIC:
    from qolo_fast_modulation.msg import DetectedPersons

DEBUG_FLAG = False
LOG_ENVIORNMENT = False
IGNORE_TRACKER = True

### ROBOT PARAMETERS
ROBOT_MARGIN = 0.3  # m
HUMAN_RADIUS = 0.3

### REFERENCE PARAMETERS
# Simulation Parameters

MAX_ANGULAR_SPEED = 0.6  # rad/s
MAX_SPEED = 0.65  # m/s

TIME_LIMIT = 9000

# Surrounding definition
# from corridor_setup import get_setup


class ObstacleAvoidanceQOLO(object):
    """
    Attributes
    ----------
    is_in_shared_control_mode (bool)
    """

    def __init__(
        self,
        obstacle_list=None,
        robot_margin=0.0,
        human_radius=0.25,
        save_metrics=True,
        save_name=None,
    ):
        print("")
        print("Initializiation starting.\n")

        rospy.init_node("Qolo_Modulation", anonymous=True)
        if IS_SIMULATION:
            self.loop_rate = 10  # Hz
        else:
            self.loop_rate = 100  # Hz

        self.loop_dt = 1.0 / self.loop_rate
        self.rate = rospy.Rate(self.loop_rate)  # Hz

        self.shutdown_finished = False

        self.is_clicked = False

        self.save_metrics = save_metrics
        if save_metrics:
            print("Saving Metrics")
            self.MetricsSaver = MetricEvaluator()

        self.save_name = save_name
        self.human_radius = human_radius

        # Initiate first time to obstacle list
        if obstacle_list is None:
            self.obstacle_list = CrowdCircleContainer(
                robot_margin=robot_margin,
                # human_radius=human_radius
            )
        else:
            self.obstacle_list = CrowdCircleContainer(
                robot_margin=robot_margin,
                # human_radius=human_radius,
                obs_list=None,
            )
            for obs in obstacle_list:
                # TODO: include loop in function
                self.obstacle_list.append(obs)

        self.obstacle_list.last_update_time = rospy.get_time()

        self.target_reached = False

        self.awaiting_pose = True
        self.msg_crowd = None

        self.msg_qolo_pose = None

        # Default
        # self.callback_tracker_persons = None

        # sub_emergency = rospy.Subscriber('qolo/emergency', Bool, self.callback_emergency)

        if IS_SIMULATION:
            self.sub_qolo_pose = rospy.Subscriber(
                "/qolo/pose", Twist, self.callback_qolo_pose
            )
            self.sub_tracker = rospy.Subscriber(
                "/crowdbot", CrowdStamped, self.callback_tracker_simu
            )

            self.pub_qolo_twist = rospy.Publisher(
                "qolo/twist_cmd", Twist, queue_size=1
            )  # Simulation

        else:
            # Real live
            self.sub_qolo_pose2D = rospy.Subscriber(
                "/qolo/pose2D", Pose2D, self.callback_qolo_pose2D
            )

            self.pub_qolo_command = rospy.Publisher(
                "qolo/remote_commands", Float32MultiArray, queue_size=1
            )

            if FLAG_DETECTOR_STATIC:
                self.sub_tracker = rospy.Subscriber(
                    "detected_persons_synchronized",
                    DetectedPersons,
                    self.callback_tracker_real,
                )
            else:
                self.sub_tracker = rospy.Subscriber(
                    "rwth_tracker/tracked_persons",
                    TrackedPersons,
                    self.callback_tracker_real,
                )

        if GET_VELOCITY_FROM_JOYSTICK:
            self.sub_remote = rospy.Subscriber(
                "qolo/user_commands", Float32MultiArray, self.callback_remote
            )

        # self.pub_qolo_twist = rospy.Publisher('qolo/twist', TwistStamped, queue_size=1)

        # self.pub_reset = rospy.Publisher('syscommand', String, queue_size=5)
        # self.pub_joystick_mode = rospy.Publisher('qolo/joystick_mode', Bool, queue_size=5)

        self.pub_qolo_global_vel = rospy.Publisher(
            "qolo/global_vel", Twist, queue_size=1
        )  # Simulation

        self.tf_listener = tf.TransformListener()

        # QOLO State and geometry definition
        self.agent = AgentQOLO(center_position=np.array([0, 0]), orientation=0)
        self.agent.margin = robot_margin
        # self.agent.control_point_local = np.array([0.3, 0.0])
        self.agent.control_point_local = np.array([0.3, 0.0])

        self.attractor_it = 0

        # Each direction 5x
        self.max_attractor_it = 1 * 1

        if IS_SIMULATION:
            self.attractor_list = [
                np.array([0.0, 20.0]),
                np.array([0.0, -20.0]),
            ]
        else:
            self.attractor_list = [
                np.array([4.5, 0.0]),
            ]

        # Only rotate after reaching attractor
        self.realign_mode = False

        self.attractor_position = self.attractor_list[self.attractor_it]

        self.max_iteration_time = 1 * 60  # 1 minutes

        self.des_speed = MAX_SPEED

        # self.tracker_not_considered = True

        self.start_time = rospy.get_time()

        if LOG_ENVIORNMENT:
            # TODO: get figure&axes handles
            self.setup_log_environment()
            self.log_dt = 0.5
            self.log_modulo = int(self.log_dt / self.loop_dt)

        self.automatic_outer_boundary = False

        if GET_VELOCITY_FROM_JOYSTICK:
            self.remote_velocity = np.zeros(2)

        print("Initialization successful.")

    def run(self, reset_relative_attractor=True):
        """Main ROS loop with"""
        print("control point", self.agent.control_point_local)

        while (self.awaiting_pose) and not rospy.is_shutdown():
            print("Awaiting first messages...")
            if self.awaiting_pose:
                print("Waiting for pose.")

            rospy.sleep(0.1)

        if reset_relative_attractor:
            # Adding initil pose correction (Attractor placement N meters ahead of the robot)
            # Need to add the correction to all list of attractors.
            time.sleep(1.5)

            self.attractor_position = self.set_initial_relative_attractor(
                self.attractor_position
            )

            print(
                " Attractor X, Y = ",
                self.attractor_position[0],
                self.attractor_position[1],
            )
            print("Execution time: ", self.max_iteration_time)
            time.sleep(2.0)

        # while (self.update_pose() and not rospy.is_shutdown()):
        # print("Awaiting first pose-lookup...")
        # rospy.sleep(0.1)

        self.it_count = 0
        self.last_rostime = rospy.get_time()
        self.last_time = time.time()
        self.diff_angular = 0  # Initialize for controller

        self.attractor_reset_time = time.time()

        # Starting main loop
        print("\nStarting looping")
        self.rate.sleep()  # Sleep once to initialize clock correctly
        while not rospy.is_shutdown():
            t_start = time.time()
            lock.acquire()  ##### LOCK ACQUIRE ######
            self.it_count += 1

            # Update agent position
            # self.update_pose()

            # Update environment
            # TODO: reactivate!!!

            self.obstacle_list.update_step(
                self.msg_crowd,
                agent_position=self.agent.position,
                human_radius=self.human_radius,
                automatic_outer_boundary=self.automatic_outer_boundary,
                is_simulation=IS_SIMULATION,
                FLAG_DETECTOR_STATIC=FLAG_DETECTOR_STATIC,
            )

            # If update reference point
            # self.obstacle_list.update_reference_points()

            attr_control_point = self.agent.get_control_point_attractor(
                self.attractor_position
            )

            # Project attractor (close to circle surface)
            self.projected_attractor = self.agent.transform_global2relative(
                attr_control_point
            )

            dist_attractor = np.linalg.norm(self.projected_attractor)
            if (
                self.automatic_outer_boundary
                and dist_attractor > self.obstacle_list[-1].radius
            ):
                self.projected_attractor = (
                    self.projected_attractor
                    / dist_attractor
                    * self.obstacle_list[-1].radius
                )  # project on wall
                self.projected_attractor = self.agent.transform_relative2global(
                    self.projected_attractor
                )
            else:
                self.projected_attractor = attr_control_point

            # evaluation_position = self.agent.position
            evaluation_position = self.agent.control_point_global

            # Get linear velocity
            if GET_VELOCITY_FROM_JOYSTICK:
                linear_ds = self.remote_velocity

            else:
                self.initial_dynamics = LinearSystem(
                    attractor_position=self.projected_attractor,
                    # maximum_velocity=1.0,
                    maximum_velocity=MAX_SPEED,
                    # evaluation_position,
                )
                # linear_ds = linear_ds_max_vel(
                #     evaluation_position,
                #     attractor=self.projected_attractor,
                #     vel_max=MAX_SPEED,
                # )

            time_start = time.time()
            # modulated_ds = obs_avoidance_interpolation_moving(self.agent.position, linear_ds, self.obstacle_list, tangent_eigenvalue_isometric=False, repulsive_obstacle=False)
            modulated_ds = obs_avoidance_interpolation_moving(
                evaluation_position,
                linear_ds,
                self.obstacle_list,
                tangent_eigenvalue_isometric=False,
                repulsive_obstacle=False,
            )

            if np.linalg.norm(linear_ds) > 0.1:
                print("init ds", linear_ds)
                print("mod ds", modulated_ds)
            # modulated_ds = linear_ds

            if np.linalg.norm(modulated_ds) > 10.0:
                self.modulated_ds = modulated_ds
                import pdb

                pdb.set_trace()

                # Make velocity safe
                self.modulated_ds = self.modulated_ds / np.linalg.norm(modulated_ds)

            delta_time = time.time() - time_start

            # command_linear, command_angular = self.controller_robot(self.agent, vel_desired=modulated_ds)
            command_linear, command_angular = self.controller_robot_floating_ds(
                self.agent, vel_desired=modulated_ds
            )

            if LOG_ENVIORNMENT and not bool(self.it_count % self.log_modulo):
                self.log_environment()

            if self.save_metrics and not self.realign_mode:
                self.MetricsSaver.update_list(
                    position=np.copy(self.agent.position),
                    velocity=np.copy(self.agent.linear_velocity),
                    closest_dist=self.get_distance_to_closets_obstacle(),
                    time=rospy.get_time(),
                )

            # angular_ds = self.angular_ds(direction_desired=np.arctan2(modulated_ds[1], modulated_ds[0]), direction_init=self.ridgeback_obs.orientation)

            lock.release()  ##### LOCK RELEASE ######

            # Don't publish command when already shutdown
            if not rospy.is_shutdown():
                # command_angular, command_linear = 0, 0

                # self.publish_command(command_linear, command_angular)
                if self.realign_mode:
                    # Turn around in place
                    command_linear, command_angular = self.controller_robot(
                        self.agent, vel_desired=linear_ds
                    )
                    command_linear = 0

                    print("Realligning robot...")
                    if command_angular < 0.01:  # Turning margin
                        self.realign_mode = False

                        if self.save_metrics:
                            self.MetricsSaver.reset_saver()

                        self.attractor_reset_time = time.time()
                        print("Reseting saver")

                # FOR SIMULATION IN GRADIENT & SOMEHOW NEGATIVE (!)
                if IS_SIMULATION:
                    self.publish_twist(
                        command_linear, command_angular / np.pi * 180 * (-1)
                    )
                else:
                    self.publish_command(command_linear, command_angular)
                # self.publish_twist(0, command_angular/np.pi*180 * (-1))

                print_info = False
                if print_info:
                    print("linear_ds", linear_ds)
                    print("modulated_ds", modulated_ds)

                    print(
                        "position",
                        np.round(self.agent.position, 2),
                        "orientation",
                        round(self.agent.orientation / np.pi * 180, 2),
                    )
                    print("local attractor", self.projected_attractor)

                    if self.obstacle_list.has_wall and self.automatic_outer_boundary:
                        print("wall rad", self.obstacle_list[-1].radius)
                        print("wall center", self.obstacle_list[-1].center_position)

                    # print('linear ds', np.round(linear_ds, 2))
                    print(
                        "ds",
                        np.round(modulated_ds, 2),
                        "--- angle",
                        round(dir_of_ds * 180 / np.pi, 2),
                    )
                    print(
                        "linear",
                        round(command_linear, 2),
                        "angular",
                        round(command_angular * 180 / np.pi, 2),
                    )
                    print("")

                # Publish global velocity (debugging only)
                msg = Twist()
                msg.linear.x, msg.linear.y, msg.angular.z = (
                    modulated_ds[0],
                    modulated_ds[1],
                    command_angular,
                )
                self.pub_qolo_global_vel.publish(msg)

                # Circular poltting takes time and reduces update rate!
                if PLOT_SHOW:
                    # if IS_SIMULATION and PLOT_SHOW:
                    self.plot_circular_world(linear_ds, modulated_ds)
                    # self.plot_circular_world(linear_ds, modulated_ds, set_automatic_yrange=False, x_lim=[-12, 10], y_lim=[-12, 12])

                if self.is_clicked:
                    import pdb

                    pdb.set_trace()

                # Toggle attractor
                self.check_if_attractor_reached()

            # Collect messages outside of lock
            if not rospy.is_shutdown():
                # Check shutdown to avoid error at end
                self.rate.sleep()

            new_time = time.time()

            print_loop_time = False
            if print_loop_time:
                new_rostime = rospy.get_time()

                rostime = new_rostime - self.last_rostime
                loop_time = new_time - self.last_time
                print(
                    "ros-loop {}ms // Actual loop {}ms".format(
                        np.round(rostime * 1000, 1), np.round(loop_time * 1000, 1)
                    )
                )

                self.last_rostime = new_rostime
            self.last_time = new_time

            if (
                not GET_VELOCITY_FROM_JOYSTICK
                and self.attractor_it > self.max_attractor_it
            ):
                print("Finished after many iterations")
                break

            if (
                not GET_VELOCITY_FROM_JOYSTICK
                and (self.last_time - self.attractor_reset_time)
                > self.max_iteration_time
            ):
                if IS_SIMULATION and self.save_metrics:
                    self.MetricsSaver.converged = False
                    self.save_metric_saver()
                print(
                    "No convergence anymore after {} lengths".format(self.attractor_it)
                )
                break

        self.shutdown()

    def get_distance_to_closets_obstacle(self):
        """Get distance to closes obstacle"""
        dist_min = 1e10  # Very large number

        for obs in self.obstacle_list:
            dist = (
                np.linalg.norm(self.agent.position - obs.position) - obs.margin_absolut
            )
            dist_min = min(dist_min, dist)
        return dist_min

    def controller_robot_floating_ds(
        self, agent, vel_desired, MaxAngular=MAX_ANGULAR_SPEED
    ):
        # Translate for consistency
        control_point = agent.control_point_local

        J_p_ref_inv = np.array(
            [[1.0, control_point[1] / control_point[0]], [0.0, 1.0 / control_point[0]]]
        )

        v_local = agent.transform_global2relative_dir(vel_desired)
        # if np.sum(v_local) > 0.1:
        # print("v_local", v_local)

        command_robot = np.matmul(J_p_ref_inv, v_local)

        command_linear = command_robot[0]

        if command_robot[1] > MaxAngular:
            command_angular = MaxAngular
        elif command_robot[1] < -MaxAngular:
            command_angular = -MaxAngular
        else:
            command_angular = command_robot[1]

        # print('Linear: {} -- Angular: {}'.format(np.round(command_linear,2), np.round(command_angular,2)))

        if command_linear > 2.0:
            import pdb

            pdb.set_trace()

        return command_linear, command_angular

    def controller_robot(
        self,
        agent,
        vel_desired,
        max_delta_angle=180.0 / 180 * np.pi,
        max_linear=None,
        p_angular=0.5,
        i_angular=0.9,
        max_command_angular=MAX_ANGULAR_SPEED,
    ):
        """Convert dynamical system into robot command.
        P-D controller in angular direction"""

        command_linear = np.linalg.norm(vel_desired)

        dir_of_ds = np.arctan2(vel_desired[1], vel_desired[0])
        diff_angular_old = copy.deepcopy(self.diff_angular)
        self.diff_angular = angle_difference_directional(
            dir_of_ds, self.agent.orientation
        )

        if self.diff_angular > max_delta_angle:
            command_linear = -command_linear

            if self.diff_angular > 0:
                self.diff_angular = self.diff_angular - np.pi
            else:
                self.diff_angular = np.pi + self.diff_angular

        # P-controller
        p_angular = 1.0 / self.loop_dt * p_angular
        command_angular = self.diff_angular * p_angular
        # command_angular = self.diff_angular*p_angular + (self.diff_angular-diff_angular_old)/self.loop_dt*i_angular

        # import pdb; pdb.set_trace()
        if abs(command_angular) > max_command_angular:
            command_angular = np.copysign(max_command_angular, command_angular)
            command_linear = 0

        return command_linear, command_angular

    def set_initial_relative_attractor(attractor_position):
        attractor_gx = (
            attractor_position[0] * np.cos(self.agent.orientation)
            - attractor_position[1] * np.sin(self.agent.orientation)
            + self.agent.position[0] * np.cos(self.agent.orientation)
            - self.agent.position[1] * np.sin(self.agent.orientation)
        )
        attractor_gy = (
            attractor_position[0] * np.sin(self.agent.orientation)
            + attractor_position[1] * np.cos(self.agent.orientation)
            + self.agent.position[0] * np.sin(self.agent.orientation)
            + self.agent.position[1] * np.cos(self.agent.orientation)
        )

        # self.attractor_position = np.array([attractor_gx, attractor_gy])
        return np.array([attractor_gx, attractor_gy])

    def check_if_attractor_reached(self, dist_convergence=0.2):
        # Turn around when attracotr reached
        if (
            np.linalg.norm(self.attractor_position - self.agent.position)
            < dist_convergence
        ):
            self.attractor_it = self.attractor_it + 1
            self.attractor_position = self.attractor_list[
                self.attractor_it % len(self.attractor_list)
            ]

            self.realign_mode = True
            self.save_metric_saver()

        # HARD CODED!!! MAKE NICER!
        # if self.attractor_position[1]<0:
        #    self.des_speed = 0.35
        # else:
        #    self.des_speed = 0.6

    def plot_circular_world(
        self,
        ds_initial,
        ds_modulated,
        x_lim=[-10.0, 10.0],
        y_lim=[-10, 10.0],
        set_automatic_xrange=True,
        set_automatic_yrange=True,
        precision=2.0,
        ax_range=14.0,
    ):

        if set_automatic_yrange:
            # prec = 5            # [m]
            # y_range = 15.0

            y_lim[0] = (
                round((self.agent.position[1] - ax_range / 2) / precision, 0)
                * precision
            )
            y_lim[1] = y_lim[0] + ax_range

        if set_automatic_xrange:

            x_lim[0] = (
                round((self.agent.position[0] - ax_range / 2) / precision, 0)
                * precision
            )
            x_lim[1] = x_lim[0] + ax_range

        if not hasattr(self, "fig"):
            # Setup plot
            self.fig = plt.figure(figsize=(7.5, 9.5))
            self.fig.canvas.mpl_connect(
                "button_press_event", self.on_click
            )  # Button Click Enabled
            self.ax = self.fig.add_subplot(1, 1, 1)

            # Get qolo image at center
            try:
                self.arr_img = mpimg.imread(
                    os.path.join(directory_path, "data", "Qolo_T_CB_top_bumper.JPG")
                )
                self.length_x = 1.4
                self.length_y = (
                    (1.0)
                    * self.arr_img.shape[0]
                    / self.arr_img.shape[1]
                    * self.length_x
                )

                self.plt_qolo = True
            except:
                print("No center qolo")
                self.plt_qolo = False

        self.ax.cla()  # Clear axes
        # import pdb; pdb.set_trace()
        # print('clearing axes')

        Simulation_vectorFields(
            x_lim,
            y_lim,
            obs=self.obstacle_list,
            xAttractor=self.projected_attractor,
            saveFigure=False,
            figName="linearSystem_boundaryCuboid",
            noTicks=False,
            showLabel=False,
            draw_vectorField=False,
            reference_point_number=False,
            drawVelArrow=True,
            automatic_reference_point=False,
            point_grid=10,
            fig_and_ax_handle=(self.fig, self.ax),
        )

        if False:
            # Annotate with time
            self.ax.annotate(
                "{}s".format(np.round(time.time())),
                xy=[
                    x_lim[1] - (x_lim[1] - x_lim[0]) * 0.2,
                    y_lim[1] - (y_lim[1] - y_lim[0]) * 0.08,
                ],
                textcoords="data",
                size=16,
                weight="bold",
            )

        # if self.plt_qolo:
        if False:
            rot = self.agent.orientation
            arr_img_rotated = ndimage.rotate(
                self.arr_img, rot * 180.0 / np.pi, cval=255
            )

            lenght_x_rotated = (
                np.abs(np.cos(rot)) * self.length_x
                + np.abs(np.sin(rot)) * self.length_y
            )
            lenght_y_rotated = (
                np.abs(np.sin(rot)) * self.length_x
                + np.abs(np.cos(rot)) * self.length_y
            )

            self.ax.imshow(
                arr_img_rotated,
                extent=[
                    self.agent.position[0] - lenght_x_rotated / 2.0,
                    self.agent.position[0] + lenght_x_rotated / 2.0,
                    self.agent.position[1] - lenght_y_rotated / 2.0,
                    self.agent.position[1] + lenght_y_rotated / 2.0,
                ],
            )
        else:
            self.ax.plot(
                self.agent.position[0],
                self.agent.position[1],
                "ko",
                markeredgewidth=4,
                markersize=13,
            )

        plt.quiver(
            self.agent.position[0],
            self.agent.position[1],
            ds_initial[0],
            ds_initial[1],
            color="r",
        )

        plt.quiver(
            self.agent.position[0],
            self.agent.position[1],
            ds_modulated[0],
            ds_modulated[1],
            color="b",
        )

        plt.quiver(
            self.agent.position[0],
            self.agent.position[1],
            self.agent.linear_velocity[0],
            self.agent.linear_velocity[1],
            color="g",
        )

        # Always pause to allow to load...
        plt.pause(5e-2)
        # import pdb; pdb.set_trace()

    def update_pose(self, orientation_init=0, position_init=0, get_pos_from_tf=False):
        """Get pose of agent by looking up transform."""

        if get_pos_from_tf:
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

            self.agent_position = np.array([agent_position[0], agent_position[1]])
            self.agent_orientation = euler_angels_agent[2]
        else:
            raise

        # Adapt with initial offset
        if orientation_init:
            cos = np.cos(orientation_init)
            sin = np.sin(orientation_init)
            ROT_INIT = np.array([[cos, sin], [-sin, cos]])

            self.agent.position = ROT_INIT.T.dot(self.agent_position) + position_init
            # self.agent_position = self.agent_position

        self.agent.orientation = self.agent.orientation + orientation_init

        # Everything worked fine
        return False

    def update_tracker_positions(self):
        # TODO: removed --- depreciated
        """Update position of (people-)obstacles based on the tracker."""
        delta_t = rospy.get_time() - self.last_update_time

        for oo in range(len(self.obstacle_list)):
            if self.obstacle_list[oo] is TrackedPedestrian():
                position = (
                    self.obstacle_list[oo].centr_position
                    + self.obstacle_list[oo].linear_velocity * delta_t
                )

    def callback_tracker_simu(self, msg):
        """Tracker which updates obstacle list and assigns new reference points"""
        with lock:
            self.msg_crowd = msg.crowd

    def callback_tracker_real(self, msg):
        """Tracker which updates obstacle list and assigns new reference points"""
        with lock:
            # CHECK IF IT's the same with the FLAG_DETECTOR_STATIC
            if FLAG_DETECTOR_STATIC:
                self.msg_crowd = msg.detections
            else:
                self.msg_crowd = msg.tracks

    def callback_qolo_pose(self, msg):
        """Callback qolo pose from simulation (Pose message)"""
        with lock:
            if self.awaiting_pose:
                self.awaiting_pose = False

            self.agent.position = np.array([msg.linear.x, msg.linear.y])
            # SIMULATION ORIENTATION IN GRADIENT
            self.agent.orientation = msg.angular.z / 180 * np.pi

    def callback_qolo_pose2D(self, msg):
        """Get agent position from 2D pose"""

        with lock:
            if self.awaiting_pose:
                self.awaiting_pose = False

            self.agent.position = np.array([msg.x, msg.y])
            self.agent.orientation = msg.theta

    def callback_remote(self, msg, input_in_local_frame=True, ref_dist_angular=1.0):
        """Get remote message and return velocity in global frame."""
        with lock:
            (msg_time, command_linear, command_angular) = msg.data
            velocity = np.array(
                [command_linear, command_angular * self.agent.control_point_local[0]]
            )

            if input_in_local_frame:
                velocity = self.agent.transform_relative2global_dir(velocity)

            self.remote_velocity = velocity

    def setup_log_environment(self):
        file_path = os.path.join(directory_path, "data", "")
        now = datetime.datetime.now()

        # dd/mm/YY
        dt_string = now.strftime("%d%m%Y_%H%M%S")

        file_name = file_path + "_qolocording_" + dt_string + ".txt"
        self.write_file = open(file_name, "w")

    def log_environment(self):
        now = rospy.get_time()

        self.write_file.write("time,{}\n".format(rospy.get_time()))
        self.write_file.write("agent\n")
        self.write_file.write(
            "position,{},{}\n".format(self.agent_position[0], self.agent_position[1])
        )
        self.write_file.write("orientation,{}\n".format(self.agent_orientation))

        for oo in range(len(self.obstacle_list)):
            self.write_file.write(
                "object," + type(self.obstacle_list[oo]).__name__ + "\n"
            )
            self.write_file.write(
                "center_position,{},{}\n".format(
                    self.obstacle_list[oo].center_position[0],
                    self.obstacle_list[oo].center_position[1],
                )
            )
            self.write_file.write(
                "orientation,{}\n".format(self.obstacle_list[oo].orientation)
            )

            self.write_file.write(
                "reference_point,{},{}\n".format(
                    self.obstacle_list[oo].reference_point[0],
                    self.obstacle_list[oo].reference_point[1],
                )
            )

            self.write_file.write(
                "linear_velocity,{},{}\n".format(
                    self.obstacle_list[oo].linear_velocity[0],
                    self.obstacle_list[oo].linear_velocity[1],
                )
            )
            self.write_file.write(
                "angular_velocity,{}\n".format(self.obstacle_list[oo].angular_velocity)
            )

            self.write_file.write(
                "is_boundary,{}\n".format(self.obstacle_list[oo].is_boundary)
            )

            if hasattr(self.obstacle_list[oo], "axes_length"):
                self.write_file.write(
                    "axes_length,{},{}\n".format(
                        self.obstacle_list[oo].axes_length[0],
                        self.obstacle_list[oo].axes_length[1],
                    )
                )

            # if hasattr(self.obstacle_list[oo], "boundary_points"):
            # self.write_file.write("boundary_points,.format(", self.obstacle_list[oo].boundary_points)

        delta_t = rospy.get_time() - now

        print("Saving took {} seconds.".format(delta_t))

    def control_c_handler(self, sig, frame):
        """User defined handling of ctrl-c"""
        print("\nCaught ctrl-c by user. Shutdown is initiated ...")
        self.shutdown()
        rospy.signal_shutdown("Caught ctrl-c by user. Shutdown is initiated ...")

    def save_metric_saver(self, current_datetime_as_name=True):
        if current_datetime_as_name:
            dt_string = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

            file_path = os.path.join(directory_path, "data")

            n_people = len(self.msg_crowd)

            self.save_name = os.path.join(
                file_path,
                "metrics_recording",
                "metrics_recording_" + str(n_people) + "_people_" + dt_string,
            )

        if self.save_metrics:
            self.MetricsSaver.save_saver(file_name=self.save_name)

    def shutdown(self):
        """User defined shutdown command."""

        print("\nDoing shutdown.")
        # lock.acquire()
        if self.shutdown_finished:
            return

        # Published repeated zero velocity to ensure stand-still
        for ii in range(10):
            if IS_SIMULATION:
                self.publish_twist(0, 0)
            else:
                self.publish_command(0, 0)
            rospy.sleep(0.01)  # ? why error...

        if LOG_ENVIORNMENT:
            self.write_file.close()
            print("\n Saved data to file ...\n")

        self.shutdown_finished = True
        # lock.release()
        print("\nShutdown successful.")

    def publish_command(self, command_linear, command_angular, tt=None):
        """
        Command to QOLO motor [Real Implementation].
        """
        # MASTER CHECK if velocity reached limit
        if np.abs(command_linear) > MAX_SPEED:
            # warnings.warn("Max linear velocity exceeded.")
            # rospy.logwarn("Max linear velocity exceeded.")
            command_linear = np.copysign(MAX_SPEED, command_linear)

        if np.abs(command_angular) > MAX_ANGULAR_SPEED:
            # warnings.warn("Max angular velocity exceeded.")
            # rospy.logwarn("Max angular velocity exceeded.")
            command_linear = np.copysign(MAX_ANGULAR_SPEED, command_angular)

        data_remote = Float32MultiArray()
        data_remote.layout.dim.append(MultiArrayDimension())
        data_remote.layout.dim[0].label = "Trajectory Commands [V, W]"
        data_remote.layout.dim[0].size = 3
        data_remote.data = [0] * 3

        if tt is None:
            # Use 'time' since 'rospy.time' is a too large float
            msg_time = round(time.clock(), 4)
        else:
            msg_time = round(tt, 4)

        data_remote.data = [msg_time, command_linear, command_angular]
        # print('pub data ', data_remote.data[0], data_remote.data[1], data_remote.data[2])

        self.pub_qolo_command.publish(data_remote)

    def publish_twist(self, command_linear, command_angular, tt=None):
        """Command to QOLO motor [Simulator]."""
        msg = Twist()
        # msg.header.stamp = rospy.Time.now()
        # msg.header.frame_id = "tf_qolo"
        msg.linear.x = command_linear
        msg.angular.z = command_angular

        self.pub_qolo_twist.publish(msg)

    def on_click(self, event):
        # Toggle is clicked
        import pdb

        pdb.set_trace()
        self.is_clicked ^= True

    def save_crowd_list_file(self):
        dir_save = os.path.join(directory_path, "data", "temp_crowd.txt")

        f = open(dir_save, "w")
        f.write(str(self.msg_crowd))
        # json.dump(self.msg_crowd, f)
        # print(f.readline())
        f.close()

    def save_obslist_to_file(self, filename=None):
        """Save current obstacle list to file
        Call with:
        self.save_obslist_to_file()
        Evaluate this with the 'debug_environment.py' script."""

        obs_dict_list = []
        for obs in self.obstacle_list:
            position = obs.center_position.tolist()
            velocity = obs.linear_velocity.tolist()
            obs_dict_list.append({"position": position, "velocity": velocity})

        dir_save = os.path.join(directory_path, "data", "temp_crowd.json")
        agent = {
            "position": self.agent.position.tolist(),
            "orientation": self.agent.orientation,
            "velocity": self.agent.linear_velocity.tolist(),
        }

        data = {
            "obstacle_list": obs_dict_list,
            "agent": agent,
            "attractor": self.attractor_position.tolist(),
            "modulated_ds": self.modulated_ds.tolist(),
        }

        with open(dir_save, "w") as ff:
            json.dump(data, ff)

        print("Save successfull to file")


# Default parameters [can be adapted only by user input]
IS_SIMULATION = False
SAVE_METRICS = False
GET_VELOCITY_FROM_JOYSTICK = True
PLOT_SHOW = False

if GET_VELOCITY_FROM_JOYSTICK and PLOT_SHOW:
    warnings.warn("Reset to PLOT_SHOW=False")
    PLOT_SHOW = False


if (__name__) == "__main__":

    # Parsing of arguments
    short_options = "smpj"
    long_options = ["simulation", "metrics-save", "plot-show", "joystick-mode"]
    try:
        arguments, values = getopt.getopt(sys.argv[1:], short_options, long_options)

    except getopt.error as err:
        print(str(err))
        sys.exit(2)

    for o, a in arguments:
        if o in ("-" + long_options[0], "--" + long_options[0]):
            IS_SIMULATION = True
        elif o in ("-" + long_options[1], "--" + long_options[1]):
            SAVE_METRICS = True
        elif o in ("-" + long_options[2], "--" + long_options[2]):
            PLOT_SHOW = True
        elif o in ("-" + long_options[3], "--" + long_options[3]):
            GET_VELOCITY_FROM_JOYSTICK = True
        else:
            warnings.warn("Arguments not defined")

    print("Starting Qolo Modulation with:")
    print("- Arguments: ", arguments)
    print("- Values: ", values)

    if SAVE_METRICS:
        print("Saving metrics")

    if IS_SIMULATION:
        # Dispaly qolo image (visualization only)
        import matplotlib.image as mpimg
        from scipy import ndimage
        from scipy import misc

        from qolo_modulation.msg import CrowdStamped

        QOLO_IMG = misc.imread(
            "/home/crowdbot/qolo_ws/src/qolo_modulation/data/Qolo_T_CB_top_bumper.JPG"
        )

    # obstacle_list = []

    # import virtual_environment

    # obstacle_list = virtual_environment.get_static_pedestrian(
    #     robot_margin=ROBOT_MARGIN, center_position=np.array([2.0, 0.1])
    # )

    obstacle_list = GradientContainer()

    try:
        ObstacleAvoidanceController = ObstacleAvoidanceQOLO(
            obstacle_list=obstacle_list,
            robot_margin=ROBOT_MARGIN,
            human_radius=HUMAN_RADIUS,
            save_metrics=SAVE_METRICS,
        )

        # Create ctrl-c handler
        signal.signal(signal.SIGINT, ObstacleAvoidanceController.control_c_handler)

        ObstacleAvoidanceController.run(
            reset_relative_attractor=(not GET_VELOCITY_FROM_JOYSTICK)
        )

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("Exception during tf lookup ...")

# print("\n... finished script. Tune in another time!")
