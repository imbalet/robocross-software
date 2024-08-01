#!/usr/bin/env python3

import cv2
import time
import rclpy
import numpy as np
import ros2_numpy as rnp

from utils import *
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseStamped
from pathfinding import Grid, AstarFinder
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path

def remap_robot_coord(current: Odometry, map_array: np.ndarray, map_resolution: float):
    x = int(map_array.shape[0] - map_array.shape[0] / 2 + current.pose.pose.position.x / map_resolution)
    y = int(map_array.shape[1] - map_array.shape[1] / 2 + current.pose.pose.position.y / map_resolution)
    a, b, th = euler_from_quaternion(current.pose.pose.orientation.x,
                                     current.pose.pose.orientation.y,
                                     current.pose.pose.orientation.z,
                                     current.pose.pose.orientation.w, )
    return x, y, th



class PathMapping(Node):

    UNKNOWN_CELL = 255
    FREE_CELL = 0
    OBSTACLE_CELL = 100

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('frequency', 30)
        self.declare_parameter('front_scan_topic', '/front_camera/scan')
        self.declare_parameter('rear_scan_topic', '/rear_camera/scan')
        self.declare_parameter('front_scan_position', [2.2, 0.0, 0.0])
        self.declare_parameter('rear_scan_position', [-2.2, 0.0, 0.0])

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_resolution', 0.1)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('map_infiltration_radius', 2.5)
        self.declare_parameter('map_size', 150)
        self.declare_parameter('local_map_size', 20)

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('robot_base_frame', 'chassis')

        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('path_base_frame', 'map')
        self.declare_parameter('path_collision_radius', 1.5)
        self.declare_parameter('goal_radius', 3.)
        self.declare_parameter('steering_value', 0.33)
        self.declare_parameter('path_discrete', 1.5)
        self.declare_parameter('timeout', 1.0)

        freq = self.get_parameter('frequency').get_parameter_value().integer_value
        front_scan_topic = self.get_parameter('front_scan_topic').get_parameter_value().string_value
        rear_scan_topic = self.get_parameter('rear_scan_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value

        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value

        self.robotBaseFrame = self.get_parameter('robot_base_frame').get_parameter_value().string_value
        self.mapFrame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.mapSize = self.get_parameter('map_size').get_parameter_value().integer_value
        self.localMapSize = self.get_parameter('local_map_size').get_parameter_value().integer_value
        self.mapRes = self.get_parameter('map_resolution').get_parameter_value().double_value
        self.robotColRadius = self.get_parameter('map_infiltration_radius').get_parameter_value().double_value
        self.frontScanPos = self.get_parameter('front_scan_position').get_parameter_value().double_array_value
        self.rearScanPos = self.get_parameter('rear_scan_position').get_parameter_value().double_array_value

        self.pathBaseFrame = self.get_parameter('path_base_frame').get_parameter_value().string_value
        self.pathCollisionRad = self.get_parameter('path_collision_radius').get_parameter_value().double_value
        self.goalRad = self.get_parameter('goal_radius').get_parameter_value().double_value
        self.steeringVal = self.get_parameter('steering_value').get_parameter_value().double_value
        self.pathDiscrete = self.get_parameter('path_discrete').get_parameter_value().double_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value

        self.mainTimer = self.create_timer(1 / freq, self.main_timer_callback)
        self.frontScanSub = self.create_subscription(LaserScan, front_scan_topic, self.front_scan_callback, 10)
        self.rearScanSub = self.create_subscription(LaserScan, rear_scan_topic, self.rear_scan_callback, 10)
        self.points = self.create_subscription(PointCloud2, '/front_camera/points', self.points_callback, 10)
        self.odomSub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.mapPub = self.create_publisher(OccupancyGrid, map_topic, 10)

        self.goalSub = self.create_subscription(PoseStamped, goal_topic, self.goal_callback, 10)
        self.pathPub = self.create_publisher(Path, path_topic, 10)

        

        self.tfBroadcaster = TransformBroadcaster(self)
        
        self.pc2Data = PointCloud2()
        self.odomData = Odometry()
        self.frontScanData = LaserScan()
        self.rearScanData = LaserScan()
        self.mapArray = np.zeros([int(self.mapSize / self.mapRes), int(self.mapSize / self.mapRes)], np.uint8)

        self.mapData = OccupancyGrid()
        self.goalData = PoseStamped()
        

    def front_scan_callback(self, msg):
        self.frontScanData = msg

    def rear_scan_callback(self, msg):
        self.rearScanData = msg

    def odom_callback(self, msg):
        self.odomData = msg    
        
    
    def main_timer_callback(self):
        self.set_scan(self.frontScanData, self.frontScanPos)
    

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
    
    
    def set_scan(self, scan_msg:LaserScan, scan_pos):
        if scan_msg != LaserScan() and self.odomData != Odometry():
            
            map_center = self.mapSize / 2 / self.mapRes
            robot_pos = [self.odomData.pose.pose.position.x, self.odomData.pose.pose.position.y]
            q = self.odomData.pose.pose.orientation
            yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)[2]
            x, y = PathMapping.robot_to_global(np.array(robot_pos), yaw, np.array(scan_pos), np.array(scan_msg.ranges), np.array([scan_msg.angle_min + i * scan_msg.angle_increment for i in range(len(scan_msg.ranges))]))
            inds = np.column_stack((x, y)) / self.mapRes + map_center
            base_x = int(map_center + self.odomData.pose.pose.position.x / self.mapRes + scan_pos[0])
            base_y = int(map_center + self.odomData.pose.pose.position.y / self.mapRes + scan_pos[2])
            for i in inds.astype(np.int32):
                cv2.line(self.mapArray, [base_x, base_y], i, [self.FREE_CELL], 1)
                self.mapArray[i[0], i[1]] = 255
        
    
    



def main():
    
    global node
    rclpy.init()
    node = PathMapping("path_mapping")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()    