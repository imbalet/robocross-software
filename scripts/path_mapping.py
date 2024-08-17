#!/usr/bin/env python3

# import cv2
# import time
# import rclpy
# import numpy as np
# import ros2_numpy as rnp

# from utils import *
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan, PointCloud2
# from tf2_ros import TransformBroadcaster
# from geometry_msgs.msg import PoseStamped
# from pathfinding import Grid, AstarFinder
# from geometry_msgs.msg import TransformStamped
# from nav_msgs.msg import OccupancyGrid, Odometry, Path

# from message_filters import Subscriber, ApproximateTimeSynchronizer
# from pc2scan import convert_from_msg

# def is_in_goal(current: Odometry, goal: PoseStamped, goal_rad: float):
#     x1, y1 = current.pose.pose.position.x, current.pose.pose.position.y
#     x2, y2 = goal.pose.position.x, goal.pose.position.y
#     if x2 - goal_rad > x1 > x2 + goal_rad:
#         if y2 - goal_rad > y1 > y2 + goal_rad:
#             return True
#     return False


# def remap_robot_coord(current: Odometry, map_array: np.ndarray, map_resolution: float):
#     x = int(map_array.shape[0] - map_array.shape[0] / 2 + current.pose.pose.position.x / map_resolution)
#     y = int(map_array.shape[1] - map_array.shape[1] / 2 + current.pose.pose.position.y / map_resolution)
#     a, b, th = euler_from_quaternion(current.pose.pose.orientation.x,
#                                      current.pose.pose.orientation.y,
#                                      current.pose.pose.orientation.z,
#                                      current.pose.pose.orientation.w, )
#     return x, y, th


# def remap_goal_coord(goal: PoseStamped, map_array: np.ndarray, map_resolution: float):
#     x = int(map_array.shape[0] - map_array.shape[0] / 2 + goal.pose.position.x / map_resolution)
#     y = int(map_array.shape[1] - map_array.shape[1] / 2 + goal.pose.position.y / map_resolution)
#     uturn = True if goal.pose.position.z == 1. else False
#     return x, y, uturn


# class PathMapping(Node):

#     UNKNOWN_CELL = 255
#     FREE_CELL = 0
#     OBSTACLE_CELL = 100

#     def __init__(self, node_name: str):
#         super().__init__(node_name)

#         self.declare_parameter('frequency', 30)
#         self.declare_parameter('front_scan_topic', '/front_camera/scan')
#         self.declare_parameter('rear_scan_topic', '/rear_camera/scan')
#         self.declare_parameter('front_scan_position', [2.2, 0.0, 0.0])
#         self.declare_parameter('rear_scan_position', [-2.2, 0.0, 0.0])

#         self.declare_parameter('map_topic', '/map')
#         self.declare_parameter('map_resolution', 0.1)
#         self.declare_parameter('map_frame', 'map')
#         self.declare_parameter('map_infiltration_radius', 2.5)
#         self.declare_parameter('map_size', 150)
#         self.declare_parameter('local_map_size', 20)

#         self.declare_parameter('odom_topic', '/odom')
#         self.declare_parameter('robot_base_frame', 'chassis')

#         self.declare_parameter('goal_topic', '/goal_pose')
#         self.declare_parameter('path_topic', '/path')
#         self.declare_parameter('path_base_frame', 'map')
#         self.declare_parameter('path_collision_radius', 1.5)
#         self.declare_parameter('goal_radius', 3.)
#         self.declare_parameter('steering_value', 0.33)
#         self.declare_parameter('path_discrete', 1.5)
#         self.declare_parameter('timeout', 1.0)

#         freq = self.get_parameter('frequency').get_parameter_value().integer_value
#         front_scan_topic = self.get_parameter('front_scan_topic').get_parameter_value().string_value
#         rear_scan_topic = self.get_parameter('rear_scan_topic').get_parameter_value().string_value
#         odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
#         map_topic = self.get_parameter('map_topic').get_parameter_value().string_value

#         goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
#         path_topic = self.get_parameter('path_topic').get_parameter_value().string_value

#         self.robotBaseFrame = self.get_parameter('robot_base_frame').get_parameter_value().string_value
#         self.mapFrame = self.get_parameter('map_frame').get_parameter_value().string_value
#         self.mapSize = self.get_parameter('map_size').get_parameter_value().integer_value
#         self.localMapSize = self.get_parameter('local_map_size').get_parameter_value().integer_value
#         self.mapRes = self.get_parameter('map_resolution').get_parameter_value().double_value
#         self.robotColRadius = self.get_parameter('map_infiltration_radius').get_parameter_value().double_value
#         self.frontScanPos = self.get_parameter('front_scan_position').get_parameter_value().double_array_value
#         self.rearScanPos = self.get_parameter('rear_scan_position').get_parameter_value().double_array_value

#         self.pathBaseFrame = self.get_parameter('path_base_frame').get_parameter_value().string_value
#         self.pathCollisionRad = self.get_parameter('path_collision_radius').get_parameter_value().double_value
#         self.goalRad = self.get_parameter('goal_radius').get_parameter_value().double_value
#         self.steeringVal = self.get_parameter('steering_value').get_parameter_value().double_value
#         self.pathDiscrete = self.get_parameter('path_discrete').get_parameter_value().double_value
#         self.timeout = self.get_parameter('timeout').get_parameter_value().double_value

#         self.mainTimer = self.create_timer(1 / freq, self.main_timer_callback)
#         self.frontScanSub = self.create_subscription(LaserScan, front_scan_topic, self.front_scan_callback, 10)
#         self.rearScanSub = self.create_subscription(LaserScan, rear_scan_topic, self.rear_scan_callback, 10)
#         self.points = self.create_subscription(PointCloud2, '/front_camera/points', self.points_callback, 10)
#         self.odomSub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
#         self.mapPub = self.create_publisher(OccupancyGrid, map_topic, 10)

#         self.goalSub = self.create_subscription(PoseStamped, goal_topic, self.goal_callback, 10)
#         self.pathPub = self.create_publisher(Path, path_topic, 10)
        
        
#         self.odometry_sub = Subscriber(self, Odometry, odom_topic)
#         self.laser_scan_sub = Subscriber(self, LaserScan, front_scan_topic)

#         # Создание синхронизирующего фильтра
#         self.sync = ApproximateTimeSynchronizer(
#             [self.odometry_sub, self.laser_scan_sub],
#             queue_size=10,
#             slop=0.005  
#         )
#         self.sync.registerCallback(self.callback)
        

#         self.tfBroadcaster = TransformBroadcaster(self)
        
#         self.pc2Data = PointCloud2()
#         self.odomData = Odometry()
#         self.frontScanData = LaserScan()
#         self.rearScanData = LaserScan()
#         self.mapArray = np.zeros([int(self.mapSize / self.mapRes), int(self.mapSize / self.mapRes)], np.uint8)

#         self.mapData = OccupancyGrid()
#         self.goalData = PoseStamped()

#         self.grid = Grid(np.zeros((1, 1)), self.steeringVal, self.pathDiscrete / self.mapRes)
#         self.finder = AstarFinder(self.pathCollisionRad / self.mapRes,
#                                   self.timeout,
#                                   self.goalRad / self.mapRes)
    
#     def points_callback(self, msg):
#         self.pc2Data = msg

#     def goal_callback(self, msg):
#         self.goalData = msg
    
#     def callback(self, odom_msg: Odometry, laser_msg: LaserScan):
#         self.frontScanData = laser_msg
#         self.odomData = odom_msg
#         n1 = laser_msg.header.stamp.nanosec
#         n2 = odom_msg.header.stamp.nanosec

#     def main_timer_callback(self):
#         self.publish_tf()

#         # start_time = time.time()
#         x1, y1, th1 = remap_robot_coord(self.odomData, self.mapArray, self.mapRes)

#         if len(self.pc2Data.data) == 0: return
#         self.set_scan1(convert_from_msg(self.pc2Data), self.frontScanPos)
        
#         # self.set_scan(self.frontScanData, self.frontScanPos)
#         # self.set_scan(self.rearScanData, self.rearScanPos)
#         # self.set_obstacles(x1, y1)

#         if self.goalData != PoseStamped():
#             if not is_in_goal(self.odomData, self.goalData, self.goalRad):
#                 map_array = np.copy(self.mapArray)
#                 self.grid.init_grid(self.mapArray)
#                 x2, y2, uturn = remap_goal_coord(self.goalData, self.mapArray, self.mapRes)
#                 if uturn and self.finder.uturnState != 3:
#                     path = self.finder.get_uturn(self.grid, (x1, y1, th1))
#                 elif uturn and self.finder.uturnState == 3:
#                     self.grid.neighbours = self.grid.forward_neighbours
#                     path = self.finder.get_path(self.grid, (x1, y1, th1), (x2, y2))
#                 else:
#                     self.finder.uturnState = 0
#                     path = self.finder.get_path(self.grid, (x1, y1, th1), (x2, y2))
#                 if type(path) is list:
#                     self.publish_path(path, map_array.shape)
#                 else:
#                     self.publish_path(None, map_array.shape)
#                     self.get_logger().warn(path)

#         self.publish_map_part(x1, y1)   
#         # self.publish_map()
#         # print(1 / (time.time() - start_time))

#     def publish_path(self, path, map_shape):
#         msg = Path()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = self.pathBaseFrame
#         if path:
#             for pose in path:
#                 p = PoseStamped()
#                 p.pose.position.x = float(pose[0]) * self.mapRes - map_shape[0] * self.mapRes / 2
#                 p.pose.position.y = float(pose[1]) * self.mapRes - map_shape[1] * self.mapRes / 2
#                 p.pose.orientation.z = float(pose[2])
#                 msg.poses.append(p)
#         self.pathPub.publish(msg)

#     def set_obstacles(self, x, y):
#         size = int(self.localMapSize / 2 / self.mapRes)
#         indexes = np.where(self.mapArray[y - size:y + size, x - size:x + size] == 255)
#         ys, xs = indexes[0], indexes[1]
#         for a, b in zip(xs, ys):
#             cv2.circle(self.mapArray[y - size:y + size, x - size:x + size], [a, b], int(self.robotColRadius / self.mapRes), [70], -1)
#         for a, b in zip(xs, ys):
#             try:
#                 self.mapArray[y - size:y + size, x - size:x + size][b, a] = 255
#             except IndexError:
#                 self.get_logger().info('Robot sensor vision is out of bounds')
#                 break


#     def robot_to_global(robot_pos, robot_orientation, lidar_pos, lidar_distances, lidar_angles):
#         """
#         Преобразует локальные координаты лидара в глобальные координаты, учитывая позицию лидара на роботе.
        
#         :param robot_pos: Позиция робота в глобальной системе координат (массив [x, y]).
#         :param robot_orientation: Ориентация робота (угол в радианах).
#         :param lidar_pos: Позиция лидара относительно робота (массив [x_offset, y_offset]).
#         :param lidar_distances: Массив расстояний, измеренных лидарами.
#         :param lidar_angles: Массив углов (в радианах), соответствующих каждому расстоянию.
#         :return: Массивы глобальных координат (x_global, y_global) лидара.
#         """
        
#         # Преобразуем полярные координаты в декартовы, избегая промежуточных массивов
#         lidar_x_local = lidar_distances * np.cos(lidar_angles)
#         lidar_y_local = lidar_distances * np.sin(lidar_angles)
        
#         # Учитываем смещение относительно робота
#         lidar_offset_x = lidar_x_local + lidar_pos[0]
#         lidar_offset_y = lidar_y_local + lidar_pos[1]
        
#         # Создаем матрицу вращения для ориентации робота
#         cos_theta = np.cos(robot_orientation)
#         sin_theta = np.sin(robot_orientation)

#         # Применяем вращение и смещение глобальной позиции
#         x_global = lidar_offset_x * cos_theta - lidar_offset_y * sin_theta + robot_pos[0]
#         y_global = lidar_offset_x * sin_theta + lidar_offset_y * cos_theta + robot_pos[1]
        
#         return x_global.astype(np.int32), y_global.astype(np.int32)  # Возвращаем глобальные координаты x и y

#     @staticmethod
#     def robot_to_global1(robot_pos:np.ndarray, robot_orientation, lidar_pos:np.ndarray, lidar_distances:np.ndarray, lidar_angles:np.ndarray):
#         """
#         Преобразует локальные координаты лидара в глобальные координаты, учитывая позицию лидара на роботе.
        
#         :param robot_pos: Позиция робота в глобальной системе координат (массив [x, y]).
#         :param robot_orientation: Ориентация робота (угол в радианах).
#         :param lidar_pos: Позиция лидара относительно робота (массив [x_offset, y_offset]).
#         :param lidar_distances: Массив расстояний, измеренных лидарами.
#         :param lidar_angles: Массив углов (в радианах), соответствующих каждому расстоянию.
#         :return: Массивы глобальных координат (x_global, y_global) лидара.
#         """
        
#         # Вычисляем глобальные координаты в одном шаге
#         cos_theta = np.cos(robot_orientation)
#         sin_theta = np.sin(robot_orientation)

#         lidar_x_global:np.ndarray = robot_pos[0] + lidar_pos[0] + lidar_distances * np.cos(lidar_angles) * cos_theta - lidar_distances * np.sin(lidar_angles) * sin_theta
#         lidar_y_global:np.ndarray = robot_pos[1] + lidar_pos[1] + lidar_distances * np.cos(lidar_angles) * sin_theta + lidar_distances * np.sin(lidar_angles) * cos_theta

#         return lidar_x_global.astype(np.int32), lidar_y_global.astype(np.int32)

#     def set_scan(self, scan_msg:LaserScan, scan_pos):
#         if scan_msg != LaserScan() and self.odomData != Odometry():
#             map_center = self.mapSize / 2 / self.mapRes
#             q = self.odomData.pose.pose.orientation
#             base_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)[2]
#             x, y = PathMapping.robot_to_global(np.array((self.odomData.pose.pose.position.x, self.odomData.pose.pose.position.y)), base_yaw, np.array(scan_pos[:2]), 
#                                         np.array(scan_msg.ranges), np.array([scan_msg.angle_min + i * scan_msg.angle_increment for i in range(len(scan_msg.ranges))]))
#             base_x = int(map_center + self.odomData.pose.pose.position.x / self.mapRes + scan_pos[0])
#             base_y = int(map_center + self.odomData.pose.pose.position.y / self.mapRes + scan_pos[2])
#             for i in (np.column_stack((x, y)) / self.mapRes).astype(np.int32):
#                 cv2.line(self.mapArray, [base_x, base_y], i, [self.FREE_CELL], 1)
#                 # if all(i != [base_x, base_y]): 
#                 self.mapArray[i[0], i[1]] = 255
                
            
            

#     def set_scan1(self, scan_msg:LaserScan, scan_pos):
#         if scan_msg != LaserScan() and self.odomData != Odometry():
#             map_center = self.mapSize / 2 / self.mapRes
#             rho, th = decart_to_polar(scan_pos[0] / self.mapRes, scan_pos[1] / self.mapRes)
#             q = self.odomData.pose.pose.orientation
#             base_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)[2]
#             sensor_x, sensor_y = polar_to_decart(rho, base_yaw)
#             base_x = int(map_center + self.odomData.pose.pose.position.x / self.mapRes + sensor_x)
#             base_y = int(map_center + self.odomData.pose.pose.position.y / self.mapRes + sensor_y)
#             base_yaw += scan_msg.angle_min
#             angles = [base_yaw + i * scan_msg.angle_increment for i in range(len(scan_msg.ranges))]

#             non_empty_ranges = []
#             non_empty_angles = []
#             for rho, phi in zip(scan_msg.ranges, angles):
#                 if rho != 0.:
#                     non_empty_ranges.append(rho)
#                     non_empty_angles.append(phi)
#                 else:
#                     rho = scan_msg.range_max
#                     x, y = polar_to_decart(rho / self.mapRes, phi)
#                     x = int(x + base_x)
#                     y = int(y + base_y)
#                     cv2.line(self.mapArray, [base_x, base_y], [x, y], [self.FREE_CELL], 1)
#             for rho, phi in zip(non_empty_ranges, non_empty_angles):
#                 x, y = polar_to_decart(rho / self.mapRes, phi)
#                 x = int(x + base_x)
#                 y = int(y + base_y)
#                 cv2.line(self.mapArray, [base_x, base_y], [x, y], [self.FREE_CELL], 1)
#                 try:
#                     self.mapArray[y, x] = 255
#                 except IndexError:
#                     self.get_logger().info('Robot sensor vision is out of bounds')
#                     break

#     def publish_map(self):
#         oc_array = np.copy(self.mapArray)
#         oc_array[oc_array == 255] = self.OBSTACLE_CELL
#         oc_array = np.array(oc_array, np.int8)
#         grid = rnp.msgify(OccupancyGrid, oc_array)
#         grid.header.stamp = self.get_clock().now().to_msg()
#         grid.header.frame_id = self.mapFrame
#         grid.info.resolution = self.mapRes
#         grid.info.origin.position.x = -self.mapSize / 2
#         grid.info.origin.position.y = -self.mapSize / 2
#         self.mapPub.publish(grid)

#     def publish_map_part(self, x1, y1):
#         oc_array = np.copy(self.mapArray)
#         size = int(self.localMapSize / 2 / self.mapRes)
#         # oc_array = oc_array[y1 - size:y1 + size, x1 - size:x1 + size]
#         oc_array[oc_array == 255] = self.OBSTACLE_CELL
#         oc_array = np.array(oc_array, np.int8)
#         grid = rnp.msgify(OccupancyGrid, oc_array)
#         grid.header.stamp = self.get_clock().now().to_msg()
#         grid.header.frame_id = self.mapFrame
#         grid.info.resolution = self.mapRes
#         grid.info.origin.position.x = self.odomData.pose.pose.position.x - size * self.mapRes
#         grid.info.origin.position.y = self.odomData.pose.pose.position.y - size * self.mapRes

#         self.mapPub.publish(grid)

#     def publish_tf(self):
#         msg = TransformStamped()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = self.mapFrame
#         msg.child_frame_id = self.robotBaseFrame
#         msg.transform.translation.x = self.odomData.pose.pose.position.x
#         msg.transform.translation.y = self.odomData.pose.pose.position.y
#         msg.transform.translation.z = self.odomData.pose.pose.position.z + 1.065
#         msg.transform.rotation = self.odomData.pose.pose.orientation
#         self.tfBroadcaster.sendTransform(msg)

#     def front_scan_callback(self, msg):
#         # self.frontScanData = msg
#         pass

#     def rear_scan_callback(self, msg):
#         self.rearScanData = msg

#     def odom_callback(self, msg):
#         # self.odomData = msg
#         pass

# node = None

# def main():
    
#     global node
#     rclpy.init()
#     node = PathMapping("path_mapping")
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()



#!/usr/bin/env python3

import json

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
        self.declare_parameter('map_size', 40)
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
        # self.points = self.create_subscription(PointCloud2, '/front_camera/points', self.points_callback, 10)
        self.odomSub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.mapPub = self.create_publisher(OccupancyGrid, map_topic, 10)

        # self.goalSub = self.create_subscription(PoseStamped, goal_topic, self.goal_callback, 10)
        self.pathPub = self.create_publisher(Path, path_topic, 10)

        

        self.tfBroadcaster = TransformBroadcaster(self)
        
        self.pc2Data = PointCloud2()
        self.odomData = Odometry()
        self.odom_temp = Odometry()
        self.frontScanData = LaserScan()
        self.front_temp = LaserScan()
        self.rearScanData = LaserScan()
        self.mapArray = np.zeros([int(self.mapSize / self.mapRes), int(self.mapSize / self.mapRes)], np.uint8)

        self.mapData = OccupancyGrid()
        self.goalData = PoseStamped()
        

    def front_scan_callback(self, msg):
        self.front_temp = msg
        # self.frontScanData = msg

    def rear_scan_callback(self, msg):
        self.rearScanData = msg

    def odom_callback(self, msg):
        self.odom_temp = msg
        # self.odomData = msg    
        
    def publish_tf(self):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.mapFrame
        msg.child_frame_id = self.robotBaseFrame
        msg.transform.translation.x = self.odomData.pose.pose.position.x
        msg.transform.translation.y = self.odomData.pose.pose.position.y
        msg.transform.translation.z = self.odomData.pose.pose.position.z + 1.065
        msg.transform.rotation = self.odomData.pose.pose.orientation
        self.tfBroadcaster.sendTransform(msg)
        
        
    
    def main_timer_callback(self):
        try:
            self.publish_tf()
            self.set_scan(self.frontScanData, self.frontScanPos)
            self.publish_map()
            self.frontScanData = self.front_temp
            self.odomData = self.odom_temp
        except Exception as e:
            print(e)
            pass
    

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
            # print(inds)
            base_x = int(map_center + self.odomData.pose.pose.position.x / self.mapRes + scan_pos[0])
            base_y = int(map_center + self.odomData.pose.pose.position.y / self.mapRes + scan_pos[1])
            for i in inds.astype(np.int32):
                if abs(i[0]) < self.mapArray.shape[1] and abs(i[1]) < self.mapArray.shape[0]:
                    cv2.line(self.mapArray, [base_x, base_y], i, [self.FREE_CELL], 1)
                    self.mapArray[i[1]][i[0]] = 255
                    # print(self.mapArray[i[0], i[1]])
        
    def publish_map(self):
        oc_array = np.copy(self.mapArray)
        oc_array[oc_array == 255] = self.OBSTACLE_CELL
        oc_array = np.array(oc_array, np.int8)
        grid = rnp.msgify(OccupancyGrid, oc_array)
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.mapFrame
        grid.info.resolution = self.mapRes
        grid.info.origin.position.x = -self.mapSize / 2
        grid.info.origin.position.y = -self.mapSize / 2
        self.mapPub.publish(grid)
    
    



def main():
    rclpy.init()
    node = PathMapping("path_mapping")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()    