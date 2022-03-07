from utils import utils_load_map
import math
import numpy as np


class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 2.0  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yaw_rate = 40 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.2  # [m/ss]
        self.max_delta_yaw_rate = 30.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.01  # [m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked

        # robot
        self.robot_width = 1.2  # [m] for collision check
        self.robot_length = 2.0  # [m] for collision check
        # obstacles [x(m) y(m), ....]
        self.map_file_name = "map/map.txt"
        self.ob_r, self.ob_b = utils_load_map.load_map_txt(self.map_file_name)


