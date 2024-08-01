import cv2
import time
import rclpy
import numpy as np
import ros2_numpy as rnp

from utils import *

map_size = 150
res = 0.1
scan_pos = [0, 0, 0]
yaw_ = 0
robot_pos = [0, 0, 0]
min_angle = 0.7

map_center = map_size / 2 / res


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


def first(rhos, phis):
    yaw = yaw_
    rho, th = decart_to_polar(scan_pos[0] / res, scan_pos[1] / res)
    sensor_x, sensor_y = polar_to_decart(rho, yaw)
    base_x = int(map_center + robot_pos[0] / res + sensor_x)
    base_y = int(map_center + robot_pos[1] / res + sensor_y)

    answ = []
    for rho, phi in zip(rhos, phis):
        x, y = polar_to_decart(rho / res, phi)
        x = int(x + base_x)
        y = int(y + base_y)
        answ.append([x, y])
    
    return answ
    

def sec(rhos, phis):
    yaw = yaw_
    x, y = robot_to_global(np.array(robot_pos), yaw, np.array(scan_pos), np.array(rhos), np.array(phis))
    return np.column_stack((x, y)) / res + map_center

rhos_ = [1, 2, 3]
phis_ = [1, 1.1, 1.2]

a = first(rhos_, phis_)
b = sec(rhos_, phis_)

print()
    
    
    