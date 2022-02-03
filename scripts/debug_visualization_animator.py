"""
Animation to Simplify Debugging
"""
import numpy as np
from numpy import linalg as LA

import matplotlib.pyplot as plt

from vartools.animator import Animator

from dynamic_obstacle_avoidance.visualization import plot_obstacles

from fast_obstacle_avoidance.obstacle_avoider import MixedEnvironmentAvoider


class DebugVisualizer():
    @property
    def figure_is_open(self):
        return plt.fignum_exists(self.fig.number)

    def __init__(self, main_controller, robot,
                 x_lim=[-3, 4], y_lim=[-3, 3], array_size=1000,
                 dt_sleep=0.05,
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

    def update_step(self, ii, initial_velocity, modulated_velocity,
                    command_linear=None, command_angular=None,
                    arrow_scale=0.4,
                    ros_time=None
    ):
        """Update robot and position."""
        # Update position_list -> todo in the future for better evaluation.

            
        self.ax.clear()

        laserscan = self.robot.get_allscan()
        self.ax.plot(
            laserscan[0, :],
            laserscan[1, :],
            ".",
            # color=self.obstacle_color,
            color='k',
            zorder=-1,
        )

        # Plot obstacles (or at least limit environment)
        if self.robot.obstacle_environment is None:
            self.ax.set_xlim(self.x_lim)
            self.ax.set_ylim(self.y_lim)
            self.ax.set_aspect("equal")
            
        else:
            plot_obstacles(
                self.ax,
                self.robot.obstacle_environment,
                showLabel=False,
                draw_reference=True,
                velocity_arrow_factor=1.0,
                # noTicks=True,
                x_lim=self.x_lim,
                y_lim=self.y_lim
                )

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
                label="Main reference",
            )
            
        if isinstance(self.main_controller.fast_avoider, MixedEnvironmentAvoider):
            reference = self.main_controller.fast_avoider.obstacle_avoider.reference_direction
            ref_norm = LA.norm(reference)
            self.ax.arrow(
                global_ctrl_point[0],
                global_ctrl_point[1],
                arrow_scale * reference[0],
                arrow_scale * reference[1],
                width=0.03*ref_norm,
                head_width=0.2*ref_norm,
                color="c",
                label=f"Obs-Ref {np.round(ref_norm, 2)}",
            )
            # print('obs ref_norm', ref_norm)

            reference = self.main_controller.fast_avoider.lidar_avoider.reference_direction
            ref_norm = LA.norm(reference)
            self.ax.arrow(
                global_ctrl_point[0],
                global_ctrl_point[1],
                arrow_scale * reference[0],
                arrow_scale * reference[1],
                width=0.03*ref_norm,
                head_width=0.2*ref_norm,
                color="m",
                label=f"Las.-Ref {np.round(ref_norm, 2)}",
            )
            # print('lid ref_norm', ref_norm)
            # print()
            
        
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

        if ros_time is not None:
            # breakpoint()
        # if True:
            self.ax.text(
                0.78, 0.95, f"{round(ros_time, 2)} s",
                fontsize=12,
                # fontsize=10,
                # fontfamily='Georgia',
                color='k',
                ha='left', va='top',
                transform=plt.gca().transAxes
                )

        self.robot.plot2D(self.ax)
        self.ax.grid()
        self.ax.legend(loc="upper right")
        
        plt.show()
        plt.pause(self.dt_sleep)
        # print("showing plot")
