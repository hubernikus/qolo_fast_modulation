"""
Animation to Simplify Debugging
"""
import numpy as np
import matplotlib.pyplot as plt
from vartools.animator import Animator


class DebugVisualizer():
    def __init__(self, main_controller, robot, 
                 x_lim=[-3, 4], y_lim=[-3, 3], array_size=1000,
                 dt_sleep=0.05
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
        
    def update_step(self, ii, initial_velocity, modulated_velocity):
        """Update robot and position."""
        # Update position_list -> todo in the future for better evaluation.
        
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

        arrow_scale = 0.4
        self.ax.arrow(
            global_ctrl_point[0],
            global_ctrl_point[1],
            arrow_scale * initial_velocity[0],
            arrow_scale * initial_velocity[1],
            width=0.03,
            head_width=0.2,
            color="g",
            label="Initial",
        )

        self.ax.arrow(
            global_ctrl_point[0],
            global_ctrl_point[1],
            arrow_scale * modulated_velocity[0],
            arrow_scale * modulated_velocity[1],
            width=0.03,
            head_width=0.2,
            color="b",
            label="Modulated",
        )

        self.robot.plot2D(self.ax)
        self.ax.grid()
        
        plt.show()
        plt.pause(self.dt_sleep)
        # print("showing plot")
