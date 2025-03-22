#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
import time
import math
from utils import euler_from_quaternion
from astar import Astar


class AStarPathPlanner(Node):
    def __init__(self, name):
        super().__init__(name)

        self.subscription = self.create_subscription(
            OccupancyGrid, "/costmap/costmap", self.map_callback, 10
        )

        # RVIZ topic
        self.goal_subscription = self.create_subscription(
            PoseStamped, "goal_pose", self.goal_callback, 10
        )

        self.path_publisher = self.create_publisher(Path, "planned_path", 10)

        # for test
        self.path_finder = Astar(
            5,
            10,
            0.3,
            5,
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

    def world_to_grid(self, world_x, world_y):
        rel_x = world_x - self.map_origin_x
        rel_y = world_y - self.map_origin_y
        grid_x = int(rel_x / self.resolution)
        grid_y = int(rel_y / self.resolution)
        return grid_y, grid_x

    def grid_to_world(self, grid_y, grid_x):
        rel_x = grid_x * self.resolution
        rel_y = grid_y * self.resolution
        world_x = rel_x + self.map_origin_x
        world_y = rel_y + self.map_origin_y
        return world_x, world_y

    def goal_callback(self, msg):
        goal_world_x = msg.pose.position.x
        goal_world_y = msg.pose.position.y

        _, _, yaw = euler_from_quaternion(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )

        goal_grid_y, goal_grid_x = self.world_to_grid(goal_world_x, goal_world_y)

        if self.map_data is not None:
            height, width = self.map_data.shape
            if 0 <= goal_grid_y < height and 0 <= goal_grid_x < width:
                self.goal_point = (goal_grid_y, goal_grid_x)
                self.find_path()
            else:
                self.get_logger().warn(
                    f"Целевая точка ({goal_grid_x}, {goal_grid_y}) находится за пределами карты"
                )
        else:
            self.get_logger().warn(
                "Карта еще не получена. Поиск пути будет выполнен после получения карты."
            )
            self.goal_point = (goal_grid_y, goal_grid_x)

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
        matrix = np.zeros(grid.shape, dtype=np.uint8)

        matrix[grid == -1] = 127
        matrix[grid > 50] = 255

        mask = (grid >= 0) & (grid <= 50)
        matrix[mask] = (grid[mask] * 127 // 50).astype(np.uint8)

        self.map_data = matrix

        if self.goal_point is not None:
            self.find_path()

    def find_path(self):
        if self.map_data is None:
            self.get_logger().warn("Карта не получена, невозможно выполнить поиск пути")
            return

        if self.goal_point is None:
            self.get_logger().warn(
                "Целевая точка не установлена, используйте RViz для выбора цели"
            )
            return

        robot_grid_y, robot_grid_x = self.world_to_grid(
            self.robot_pose.position.x, self.robot_pose.position.y
        )

        _, _, robot_yaw = euler_from_quaternion(
            self.robot_pose.orientation.x,
            self.robot_pose.orientation.y,
            self.robot_pose.orientation.z,
            self.robot_pose.orientation.w,
        )

        height, width = self.map_data.shape
        if (
            robot_grid_y < 0
            or robot_grid_y >= height
            or robot_grid_x < 0
            or robot_grid_x >= width
        ):
            self.get_logger().error(
                f"Начальная позиция ({robot_grid_x}, {robot_grid_y}) вне границ карты {width}x{height}"
            )
            return

        start_point = (robot_grid_y, robot_grid_x, robot_yaw)

        start_time = time.time()
        path = self.path_finder.astar(self.map_data, start_point, self.goal_point)
        elapsed_time = time.time() - start_time  # noqa

        if path:
            self.publish_path(path)
        else:
            self.get_logger().error(
                "Путь не найден! Проверьте начальные и конечные точки."
            )

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map_frame

        for point in path:
            pose = PoseStamped()
            pose.header = path_msg.header

            grid_y, grid_x, theta = point

            world_x, world_y = self.grid_to_world(grid_y, grid_x)

            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0

            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(theta / 2)
            pose.pose.orientation.w = math.cos(theta / 2)

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
