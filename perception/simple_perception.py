import math
import numpy as np


class Perception:

    """
    perception module
    """
    def __init__(self):
        self.max_distance = 15  # 最大感知距离,欧式距离
        self.red_cone = []  # 记录橙色锥桶
        self.blue_cone = []  # 记录蓝色锥桶
        self.max_angle = math.pi / 4  # 最大可观测与yaw的夹角

    def find_cones(self, x, red_cones, blue_cones):
        v = np.array([math.cos(x[2]), math.sin(x[2])])  # yaw单位向量
        detected_red_cone = []
        detected_blue_cone = []
        for cone in red_cones:
            dist = math.sqrt(pow(cone[0]-x[0], 2) + pow(cone[1] - x[1], 2))
            u = np.array([cone[0]-x[0], cone[1] - x[1]])
            Lv = np.sqrt(v.dot(v))
            Lu = np.sqrt(u.dot(u))
            cos_angle = v.dot(u) / (Lv * Lu)
            print(cos_angle)
            angle = np.arccos(cos_angle)  # 锥桶与yaw的角度限制
            if dist < self.max_distance and 0 < angle < self.max_angle:
                detected_red_cone.append(cone)
                print("####")

        for cone in blue_cones:
            dist = math.sqrt(pow(cone[0] - x[0], 2) + pow(cone[1] - x[1], 2))
            u = np.array([cone[0] - x[0], cone[1] - x[1]])
            Lv = np.sqrt(v.dot(v))
            Lu = np.sqrt(u.dot(u))
            cos_angle = v.dot(u) / (Lv * Lu)
            print(cos_angle)
            angle = np.arccos(cos_angle)  # 锥桶与yaw的角度限制
            if dist < self.max_distance and 0 < angle < self.max_angle:
                detected_blue_cone.append(cone)
                print("!!!!")
        detected_red_cone = np.array(detected_red_cone)
        detected_blue_cone = np.array(detected_blue_cone)
        print(detected_red_cone)
        return detected_red_cone, detected_blue_cone
