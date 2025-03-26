#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
import time
import math
import sys, os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from util.utils import euler_from_quaternion
from astar import Astar

import debugpy

debugpy.listen(5678)


class AStarPathPlanner(Node):
    def __init__(self, name):
        super().__init__(name)

        self.subscription = self.create_subscription(
            OccupancyGrid, "/costmap/costmap", self.map_callback, 1
        )

        self.goal_subscription = self.create_subscription(
            PoseStamped, "goal_pose", self.goal_callback, 1
        )

        self.path_publisher = self.create_publisher(Path, "planned_path", 10)

        self.finded_path = True
        self.path_finder = Astar(
            robot_radius=5,
            goal_radius=10,
            car_steering=0.2,
            path_discrete=5,
        )

        self.map_data = None
        self.resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_origin_yaw = 0.0
        self.map_frame = ""

        self.robot_pose = Pose()
        self.robot_pose.position.x = 0.0
        self.robot_pose.position.y = 0.0
        self.robot_pose.orientation.w = 1.0

        self.goal_point = None

    def world_to_grid(self, world_x, world_y, yaw):
        rel_x = world_x - self.map_origin_x
        rel_y = world_y - self.map_origin_y
        grid_x = int(rel_x / self.resolution)
        grid_y = int(rel_y / self.resolution)
        return grid_y, grid_x, -yaw

    def grid_to_world(self, grid_y, grid_x, yaw):
        rel_x = grid_x * self.resolution
        rel_y = grid_y * self.resolution
        world_x = rel_x + self.map_origin_x
        world_y = rel_y + self.map_origin_y
        return world_x, world_y, -yaw

    def goal_callback(self, msg):
        # Извлекаем yaw из ориентации цели
        _, _, goal_yaw = euler_from_quaternion(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )

        goal_world_x = msg.pose.position.x
        goal_world_y = msg.pose.position.y
        goal_grid_y, goal_grid_x, goal_grid_yaw = self.world_to_grid(
            goal_world_x, goal_world_y, goal_yaw
        )

        # Сохраняем цель как (y, x, yaw)
        self.goal_point = (goal_grid_y, goal_grid_x, goal_grid_yaw)

        if self.map_data is not None:
            if (
                0 <= goal_grid_y < self.map_data.shape[0]
                and 0 <= goal_grid_x < self.map_data.shape[1]
            ):
                self.find_path()
            else:
                self.get_logger().warn("Goal out of map bounds")
        self.finded_path = False

    def map_callback(self, msg: OccupancyGrid):
        self.resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_frame = msg.header.frame_id

        _, _, self.map_origin_yaw = euler_from_quaternion(
            msg.info.origin.orientation.x,
            msg.info.origin.orientation.y,
            msg.info.origin.orientation.z,
            msg.info.origin.orientation.w,
        )

        width = msg.info.width
        height = msg.info.height

        grid = np.array(msg.data, dtype=np.int8).reshape(height, width)
        self.map_data = np.where(grid > 50, 255, 0).astype(np.uint8)

        if self.goal_point:
            self.find_path()

    def find_path(self):
        if not self.finded_path:
            if self.map_data is None:
                self.get_logger().warn(
                    "Карта не получена, невозможно выполнить поиск пути"
                )
                return

            _, _, robot_yaw = euler_from_quaternion(
                self.robot_pose.orientation.x,
                self.robot_pose.orientation.y,
                self.robot_pose.orientation.z,
                self.robot_pose.orientation.w,
            )

            robot_grid_y, robot_grid_x, robot_grid_yaw = self.world_to_grid(
                self.robot_pose.position.x, self.robot_pose.position.y, robot_yaw
            )

            start_point = (robot_grid_y, robot_grid_x, robot_grid_yaw)

            path = self.path_finder.astar(
                self.map_data, start=start_point, goal=self.goal_point
            )

            if path:
                self.publish_path(path)
            else:
                self.get_logger().error("Path not found")
            self.finded_path = True

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map_frame

        for grid_y, grid_x, yaw in path:
            pose = PoseStamped()
            pose.header = path_msg.header

            # Преобразование сетки в мировые координаты
            world_x, world_y, world_yaw = self.grid_to_world(grid_y, grid_x, yaw)

            # Заполняем позицию
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0

            # Заполняем ориентацию из yaw
            pose.pose.orientation.z = math.sin(world_yaw / 2)
            pose.pose.orientation.w = math.cos(world_yaw / 2)

            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)

    def update_robot_pose(self, pose):
        self.robot_pose = pose


def main(args=None):
    rclpy.init(args=args)
    node = AStarPathPlanner("pathfinder")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
