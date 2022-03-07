import math

import numpy as np


class Plan:
    """
    planning module
    """
    def __init__(self):
        self.mode = 0
        """
        control_mode = 0 动态窗口
        control_mode = 1 五次曲线拟合
        """