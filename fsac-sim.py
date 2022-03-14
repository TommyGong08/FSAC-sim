"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı

"""

import math
from enum import Enum
import matplotlib.pyplot as plt
import numpy as np
from config import config
from perception import perceptor
from plan import planner
from plan import dynamic_windows_planner
from plan import quintic_polynomials_planner
from plan import rrt_planner
show_animation = True

config = config.Config()
perception_module = perceptor.Perception()
planning_module = planner.Plan()


def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)


def plot_robot(x, y, yaw, config):  # pragma: no cover
    outline = np.array([[-config.robot_length / 2, config.robot_length / 2,
                         (config.robot_length / 2), -config.robot_length / 2,
                         -config.robot_length / 2],
                        [config.robot_width / 2, config.robot_width / 2,
                         - config.robot_width / 2, -config.robot_width / 2,
                         config.robot_width / 2]])
    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    outline = (outline.T.dot(Rot1)).T
    outline[0, :] += x
    outline[1, :] += y
    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), "-k")


def main(gx=10.0, gy=10.0):
    print(__file__ + " start!!!")
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([18.0, 0.0, 0, 0.0, 0.0])
    # goal position [x(m), y(m)]
    # goal = np.array([gx, gy])

    # input [forward speed, yaw_rate]

    ob_r = config.ob_r
    ob_b = config.ob_b

    if planning_module.mode == 2:
        obstacle_list = np.vstack((ob_r, ob_b))
        print(obstacle_list)
        # Set Initial parameters
        rrt = rrt_planner.RRT(
            rand_area=[-2, 15],
            obstacle_list=obstacle_list,
            # play_area=[0, 10, 0, 14]
        )

    while True:
        # perception
        """
        detect cones in front of the race car
        """
        detected_red_cones, detected_blue_cones = perception_module.detect_cones(x, ob_r, ob_b)

        # planning
        """
        input: location of red and blue cones in front of the race car
        output: the middle line or the best suitable trajectory 
        """
        mid_trajectory = dynamic_windows_planner.cal_mid_points(detected_red_cones, detected_blue_cones)
        """
        input:mid or best trajectory
        output: [x, y, yaw, v, yaw_rate]
        """
        # goal, goal_yaw = planner.cal_farthest_goal(x, mid_trajectory)
        goal, goal_yaw = planner.cal_nearest_goal(x, mid_trajectory)

        if planning_module.mode == 1:
            # 五次多项式曲线拟合
            # 计算出时间、空间、速度、加速度和加加速度的信息
            # 最大加速度与加加速度
            max_accel = 0.5  # max accel [m/ss]
            max_jerk = 0.3  # max jerk [m/sss]
            # 时间间隔0.1
            dt = 0.1  # time tick [s]
            sa = 0.1
            ga = 0.1
            goal_v = 1.0  # [m/s]
            x, j, predicted_trajectory = quintic_polynomials_planner.quintic_polynomials_planner(
                x[0], x[1], x[2], x[3], sa, goal[0], goal[1], goal_yaw, goal_v, ga, max_accel, max_jerk, dt)

        elif planning_module.mode == 2:  # rrt
            start = [x[0], x[1]]
            path = rrt.planning(start, goal, animation=False)

        # control
        # 动态窗口法
        ob = np.vstack((detected_red_cones, detected_blue_cones))
        best_w, predicted_trajectory = dynamic_windows_planner.dwa_control(x, config, goal, ob)  # 计算动态窗口
        x = dynamic_windows_planner.motion(x, best_w, config.dt)  # simulate robot; x为下一时刻的车辆状态
        # trajectory = np.vstack((trajectory, x))  # store state history

        # plot
        plt.cla()
        plt.plot(ob_r[:, 0], ob_r[:, 1], "or")
        plt.plot(ob_b[:, 0], ob_b[:, 1], "ob")
        plt.plot(detected_red_cones[:, 0], detected_red_cones[:, 1], "-g")
        plt.plot(detected_blue_cones[:, 0], detected_blue_cones[:, 1], "-g")
        plt.plot(mid_trajectory[:, 0], mid_trajectory[:, 1], ".")
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-y')  # rrt
        plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-r")
        plot_robot(x[0], x[1], x[2], config)  # draw race car
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.0001)
    plt.show()


if __name__ == '__main__':
    main()
