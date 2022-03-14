import math
import numpy as np


def cal_nearest_goal(x, mid_trajectory):
    """
    calculate the goal point of the mid line
    """
    if len(mid_trajectory) == 0:
        print("no mid trajectory")
    dist = 9999
    for point in mid_trajectory:
        temp_dist = math.sqrt(pow(point[0]-x[0], 2) + pow(point[1]-x[1], 2))
        if temp_dist < dist:
            dist = temp_dist
            goal_point = point
    if len(mid_trajectory) == 1 or 2:
        goal_yaw = x[2]
    else:
        length = math.sqrt(pow(mid_trajectory[1][0] - mid_trajectory[2][0], 2) + pow(mid_trajectory[1][1]-mid_trajectory[2][1], 2))
        goal_yaw = np.arccos((mid_trajectory[0][0] - mid_trajectory[1][0])/length)
    return goal_point, goal_yaw


def cal_farthest_goal(x, mid_trajectory):
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
    if len(mid_trajectory) == 1:
        goal_yaw = x[2]
    else:
        length = math.sqrt(pow(mid_trajectory[1][0] - mid_trajectory[2][0], 2) + pow(mid_trajectory[1][1]-mid_trajectory[2][1], 2))
        goal_yaw = np.arccos((mid_trajectory[0][0] - mid_trajectory[1][0])/length)
    return goal_point, goal_yaw


class Plan:
    """
    planning module
    """
    def __init__(self):
        self.mode = 2
        """
        planning_mode = 1 五次曲线拟合
        planning_mode = 2 rrt
        """