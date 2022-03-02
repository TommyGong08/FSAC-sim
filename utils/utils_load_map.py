import numpy
import numpy as np


def load_map_txt(map_file_name):
    """
    :param map_file_name:
    :return: list of red and blue cones
    """
    cone_r_list = []
    cone_b_list = []
    with open(map_file_name, 'r') as map_file:
        map = map_file.readlines()
        for cone in map:
            xx, yy, color = cone.split(' ')[0], cone.split(' ')[1], cone.split(' ')[2].split('\n')[0]
            # print(xx, yy, color)
            if color == "1":
                cone_r_list.append([float(xx), float(yy)])
            else:
                cone_b_list.append([float(xx), float(yy)])
    cones_r_np = np.array(cone_r_list)
    cones_b_np = np.array(cone_b_list)
    return cones_r_np, cones_b_np


if __name__ == '__main__':
    load_map_txt("../map.txt")