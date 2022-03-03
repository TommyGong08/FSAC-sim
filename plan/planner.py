import math

import numpy as np


def cal_mid_points(red_cones, blue_cones):
    mid_trajectory = []
    for cone_i in red_cones:
        for cone_j in blue_cones:
            mid_cone = [(cone_i[0] + cone_j[0]) / 2, (cone_i[1] + cone_j[1]) / 2]
            mid_trajectory.append(mid_cone)
    mid_trajectory = np.array(mid_trajectory)
    return mid_trajectory


def cal_goal(x, mid_trajectory):
    """
    calculate the goal point of the mid line
    """
    if len(mid_trajectory) == 0:
        print("no mid trajectory")
    dist = -1
    for point in mid_trajectory:
        temp_dist = math.sqrt(pow(point[0]-x[0], 2) + pow(point[1]-x[1], 2))
        if temp_dist > dist:
            dist = temp_dist
            goal_point = point
    return goal_point


class Plan:
    """
    planning module
    """
    def __init__(self):
        self.mode = 0