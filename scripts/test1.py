import numpy as np
import json
import os
import cv2

os.chdir(os.path.dirname(__file__))

with open("data.json") as fp:
    ranges = json.load(fp)


def robot_to_global(robot_pos, robot_orientation, lidar_pos, lidar_distances, lidar_angles):
    """
    Преобразует локальные координаты лидара в глобальные координаты, учитывая позицию лидара на роботе.
    
    :param robot_pos: Позиция робота в глобальной системе координат (массив [x, y]).
    :param robot_orientation: Ориентация робота (угол в радианах).
    :param lidar_pos: Позиция лидара относительно робота (массив [x_offset, y_offset]).
    :param lidar_distances: Массив расстояний, измеренных лидарами.
    :param lidar_angles: Массив углов (в радианах), соответствующих каждому расстоянию.
    :return: Массивы глобальных координат (x_global, y_global) лидара.
    """
    
    # Преобразуем полярные координаты в декартовы, избегая промежуточных массивов
    lidar_x_local = lidar_distances * np.cos(lidar_angles)
    lidar_y_local = lidar_distances * np.sin(lidar_angles)
    
    # Учитываем смещение относительно робота
    lidar_offset_x = lidar_x_local + lidar_pos[0]
    lidar_offset_y = lidar_y_local + lidar_pos[1]
    
    # Создаем матрицу вращения для ориентации робота
    cos_theta = np.cos(robot_orientation)
    sin_theta = np.sin(robot_orientation)

    # Применяем вращение и смещение глобальной позиции
    x_global = lidar_offset_x * cos_theta - lidar_offset_y * sin_theta + robot_pos[0]
    y_global = lidar_offset_x * sin_theta + lidar_offset_y * cos_theta + robot_pos[1]
    
    return x_global, y_global  # Возвращаем глобальные координаты x и y



angle_min = -0.7
angle_max = 0.7
angle_increment = (abs(angle_min) + abs(angle_max)) / len(ranges)

map_size = 20
map_res = 0.1

scan_pos = [2.2, 0.0, 0.0]

map = np.zeros([int(map_size / map_res), int(map_size / map_res)], np.uint8)


#######


map_center = map_size / 2 / map_res
robot_pos = [0, 0, 0]
robot_yaw = -3.14 / 2


x, y = robot_to_global(np.array(robot_pos), robot_yaw, np.array(scan_pos), np.array(ranges), np.array([angle_min + i * angle_increment for i in range(len(ranges))]))
inds = np.column_stack((x, y)) / map_res + map_center

base_x = int(map_center + 0 / map_res + scan_pos[0] / map_res)
base_y = int(map_center + 0 / map_res + scan_pos[2] / map_res)

map[base_x][base_y] = 255

for i in inds.astype(np.int32):
    if abs(i[0]) < map.shape[1] and abs(i[1]) < map.shape[0]:
        map[i[1]][i[0]] = 255

cv2.imshow("aa", map)
cv2.waitKey(0)

print()