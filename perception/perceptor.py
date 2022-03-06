import math
import numpy as np
from functools import cmp_to_key


def cal_nearest_cone(cones, x):
    dist = 9999
    for cone in cones:
        temp_dist = math.sqrt(pow(cone[0] - x[0], 2) + pow(cone[1] - x[1], 2))
        if temp_dist < dist:
            nearest_cone = cone
            dist = temp_dist
    return nearest_cone, nearest_cone[0], nearest_cone[1]


class Perception:

    """
    perception module
    """
    def __init__(self):
        self.max_distance = 15  # 最大感知距离,欧式距离
        self.red_cone = []  # 记录橙色锥桶
        self.blue_cone = []  # 记录蓝色锥桶
        self.max_angle = math.pi / 3  # 最大可观测与yaw的夹角
        self.dist_between_cones = 6
        self.x = None

    def compare_cone_dist(self, A, B):
        dist_A = math.sqrt(pow(A[0] - self.x[0], 2) + pow(A[1] - self.x[1], 2))
        dist_B = math.sqrt(pow(B[0] - self.x[0], 2) + pow(B[1] - self.x[1], 2))
        if dist_A >= dist_B:
            return 1
        if dist_A < dist_B:
            return -1

    def detect_cones(self, x, red_cones, blue_cones):
        """
        input: 在车辆周围一定范围内的锥桶
        output: 作为路径规划用的锥桶
        """
        self.x = x
        v = np.array([math.cos(x[2]), math.sin(x[2])])  # yaw单位向量
        Lv = np.sqrt(v.dot(v))
        detected_red_cone = []
        detected_blue_cone = []

        final_detected_red = []
        final_detected_blue = []

        for cone in red_cones:
            u = np.array([cone[0] - x[0], cone[1] - x[1]])
            Lu = np.sqrt(u.dot(u))
            cos_angle = v.dot(u) / (Lv * Lu)
            angle = np.arccos(cos_angle)  # 计算锥桶与车yaw的夹角
            dist = math.sqrt(pow(cone[0] - x[0], 2) + pow(cone[1] - x[1], 2))  # 锥桶和车的距离
            if dist < self.max_distance and 0 < angle < self.max_angle:
                detected_red_cone.append(cone)

        for cone in blue_cones:
            u = np.array([cone[0] - x[0], cone[1] - x[1]])
            Lu = np.sqrt(u.dot(u))
            cos_angle = v.dot(u) / (Lv * Lu)
            angle = np.arccos(cos_angle)  # 计算锥桶与车yaw的夹角
            dist = math.sqrt(pow(cone[0] - x[0], 2) + pow(cone[1] - x[1], 2))  # 锥桶和车的距离
            if dist < self.max_distance and 0 < angle < self.max_angle:
                detected_blue_cone.append(cone)
        # print("length detected red cones", len(detected_red_cone))
        # print("length detected blue cones", len(detected_blue_cone))

        # 计算最近的red/blue到car的距离
        nearest_red_cone, nearest_red_x, nearest_red_y = cal_nearest_cone(detected_red_cone, x)
        nearest_blue_cone, nearest_blue_x, nearest_blue_y = cal_nearest_cone(detected_blue_cone, x)
        final_detected_blue.append(nearest_blue_cone)
        final_detected_red.append(nearest_red_cone)
        # print(nearest_red_x, nearest_red_y)

        # 红蓝锥桶由近到远排序
        sorted(detected_red_cone, key=cmp_to_key(self.compare_cone_dist))
        sorted(detected_blue_cone, key=cmp_to_key(self.compare_cone_dist))
        # print(detected_red_cone)
        # print(detected_blue_cone)

        # 剔除不符合要求的锥桶
        for cone in detected_red_cone:
            dist_two_cones = math.sqrt(pow(cone[0] - nearest_red_x, 2) + pow(cone[1] - nearest_red_y, 2))
            if dist_two_cones < self.dist_between_cones :
                nearest_red_x = cone[0]
                nearest_red_y = cone[1]
                final_detected_red.append(cone)

        for cone in detected_blue_cone:
            dist_two_cones = math.sqrt(pow(cone[0] - nearest_blue_x, 2) + pow(cone[1] - nearest_blue_y, 2))
            if dist_two_cones < self.dist_between_cones :
                nearest_blue_x = cone[0]
                nearest_blue_y = cone[1]
                final_detected_blue.append(cone)
        final_detected_red = np.array(final_detected_red)
        final_detected_blue = np.array(final_detected_blue)

        return final_detected_red, final_detected_blue
