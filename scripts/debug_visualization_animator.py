"""
Animation to Simplify Debugging
"""
import numpy as np
from numpy import linalg as LA

import matplotlib.pyplot as plt

import rospy 
from geometry_msgs.msg import PoseStamped, TwistStamped

from vartools.animator import Animator


class DebugVisualizer():
    @property
    def figure_is_open(self):
        return plt.fignum_exists(self.fig.number)

    def __init__(self, main_controller, robot, 
                 x_lim=[-3, 4], y_lim=[-3, 3], array_size=1000,
                 dt_sleep=0.05,
                 publish_command=False,
    ):
        plt.ion()
        
        self.main_controller = main_controller
        self.robot = robot

        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.obstacle_color = np.array([177, 124, 124]) / 255.0

        self.x_lim = x_lim
        self.y_lim = y_lim

        self.dimension = 2
        
        self.dt_sleep = dt_sleep

        self.linear_velocity = None
        self.modulated_velocity = None

        self.position_list = np.zeros((self.dimension, array_size))

        self.publish_command = publish_command
        if self.publish_command:

            self.publisher_vel_init = rospy.Publisher(
                'qolo/vel_init', TwistStamped, queue_size=1)

            self.publisher_vel_mod = rospy.Publisher(
                'qolo/vel_modul', TwistStamped, queue_size=1)

    # def publish_twist_message(self, command_linear, command_angular):
        # msg = TwistStamped()
        # msg.header.stamp = rospy.Time.now()
        # msg.twist.linear.x = command_linear
        # msg.twist.angular.z = command_angular
        # self.publisher_twist.publish(msg)

    def publish_vel_init(self, velocity, msg_time=None):
        msg = TwistStamped()
        if msg_time is None:
            msg.header.stamp = rospy.Time.now()
        else:
            msg.header.stamp = msg_time
        msg.header.frame_id = "tf_qolo"

        msg.twist.linear.x = velocity[0]
        msg.twist.linear.y = velocity[1]

        self.publisher_vel_init.publish(msg)

    def publish_vel_mod(self, velocity, msg_time=None):
        msg = TwistStamped()
        if msg_time is None:
            msg.header.stamp = rospy.Time.now()
        else:
            msg.header.stamp = msg_time
        msg.header.frame_id = "tf_qolo"

        msg.twist.linear.x = velocity[0]
        msg.twist.linear.y = velocity[1]
        
        self.publisher_vel_mod.publish(msg)
        
    def update_step(self, ii, initial_velocity, modulated_velocity,
                    command_linear=None, command_angular=None,
                    msg_time=None,
                    arrow_scale=0.4
    ):
        """Update robot and position."""
        # Update position_list -> todo in the future for better evaluation.

        if self.publish_command:
            # self.publish_twist_message(command_linear, command_angular)
            self.publish_vel_init(initial_velocity, msg_time=msg_time)
            self.publish_vel_mod(modulated_velocity, msg_time=msg_time)
            
        self.ax.clear()

        laserscan = self.robot.get_allscan()
        self.ax.plot(
            laserscan[0, :],
            laserscan[1, :],
            ".",
            color=self.obstacle_color,
            zorder=-1,
        )

        self.ax.set_xlim(self.x_lim)
        self.ax.set_ylim(self.y_lim)
        self.ax.set_aspect("equal")

        # We're evaluating everything in the local frame
        global_ctrl_point = self.robot.control_points[:, 0]


        reference = self.main_controller.fast_avoider.reference_direction
        ref_norm = LA.norm(reference)
                
        if ref_norm:
            self.ax.arrow(
                global_ctrl_point[0],
                global_ctrl_point[1],
                arrow_scale * reference[0],
                arrow_scale * reference[1],
                width=0.03*ref_norm,
                head_width=0.2*ref_norm,
                color="k",
                label="Initial",
            )
            
        
        if LA.norm(initial_velocity):
            init_norm = LA.norm(initial_velocity)
            self.ax.arrow(
                global_ctrl_point[0],
                global_ctrl_point[1],
                arrow_scale * initial_velocity[0],
                arrow_scale * initial_velocity[1],
                width=0.03*init_norm,
                head_width=0.2*init_norm,
                color="g",
                label="Initial",
            )

            mod_norm = LA.norm(modulated_velocity)
            self.ax.arrow(
                global_ctrl_point[0],
                global_ctrl_point[1],
                arrow_scale * modulated_velocity[0],
                arrow_scale * modulated_velocity[1],
                width=0.03*mod_norm,
                head_width=0.2*mod_norm,
                color="b",
                label="Modulated",
            )

        self.robot.plot2D(self.ax)
        self.ax.grid()
        
        plt.show()
        plt.pause(self.dt_sleep)
        # print("showing plot")

