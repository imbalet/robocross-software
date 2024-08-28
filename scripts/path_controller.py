#!/usr/bin/env python3


from tf2_ros import TransformBroadcaster
import numpy as np
import ros2_numpy as rnp
from pathfinding import Grid, AstarFinder
import rclpy

from rclpy.node import Node
from utils import euler_from_quaternion, decart_to_polar
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped, Pose, TransformStamped










def is_in_goal(current: Odometry, goal: PoseStamped, goal_rad: float):
    x1, y1 = current.pose.pose.position.x, current.pose.pose.position.y
    x2, y2 = goal.pose.position.x, goal.pose.position.y
    if x2 - goal_rad > x1 > x2 + goal_rad:
        if y2 - goal_rad > y1 > y2 + goal_rad:
            return True
    return False



def remap_robot_coord(current: Odometry, map_array: np.ndarray, map_resolution: float):
    x = int(map_array.shape[0] - map_array.shape[0] / 2 + current.pose.pose.position.x / map_resolution)
    y = int(map_array.shape[1] - map_array.shape[1] / 2 + current.pose.pose.position.y / map_resolution)
    a, b, th = euler_from_quaternion(current.pose.pose.orientation.x,
                                     current.pose.pose.orientation.y,
                                     current.pose.pose.orientation.z,
                                     current.pose.pose.orientation.w, )
    return x, y, th


def remap_goal_coord(goal: PoseStamped, map_array: np.ndarray, map_resolution: float):
    x = int(map_array.shape[0] - map_array.shape[0] / 2 + goal.pose.position.x / map_resolution)
    y = int(map_array.shape[1] - map_array.shape[1] / 2 + goal.pose.position.y / map_resolution)
    uturn = True if goal.pose.position.z == 1. else False
    return x, y, uturn

class PathController(Node):
    FPS = 30

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('frequency', 30)
        self.declare_parameter('p_steering_ratio', 3.0)
        self.declare_parameter('p_speed_ratio', 1.)
        self.declare_parameter('average_speed', 1.3)
        self.declare_parameter('map_topic', "/mapfull")
        self.declare_parameter('goal_topic', "/goal_pose")
        self.declare_parameter('goal_radius', 2.0)
        self.declare_parameter("steering_value", 0.35)
        self.declare_parameter("path_discrete", 1.5)
        self.declare_parameter('map_resolution', 0.1)
        self.declare_parameter('path_collision_radius', 1.5)
        self.declare_parameter('timeout', 0.2)
        self.declare_parameter('path_base_frame', "map")

        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        frequency = self.get_parameter('frequency').get_parameter_value().integer_value
        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        
        self.pathBaseFrame = self.get_parameter('path_base_frame').get_parameter_value().string_value 
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.pathCollisionRad = self.get_parameter('path_collision_radius').get_parameter_value().double_value
        self.mapRes = self.get_parameter('map_resolution').get_parameter_value().double_value
        self.steeringVal = self.get_parameter('steering_value').get_parameter_value().double_value
        self.pathDiscrete = self.get_parameter('path_discrete').get_parameter_value().double_value

        self.averageSpeed = self.get_parameter('average_speed').get_parameter_value().double_value
        self.pSteeringRatio = self.get_parameter('p_steering_ratio').get_parameter_value().double_value
        self.pSpeedRatio = self.get_parameter('p_speed_ratio').get_parameter_value().double_value
        self.goalRad = self.get_parameter('goal_radius').get_parameter_value().double_value
        
        
        self.tfBroadcaster = TransformBroadcaster(self)

        self.pathSub = self.create_subscription(Path,
                                                path_topic,
                                                self.path_callback,
                                                10)

        self.odomSub = self.create_subscription(Odometry,
                                                odom_topic,
                                                self.odom_callback,
                                                10)
        self.cmdPub = self.create_publisher(Twist,
                                            cmd_topic,
                                            10)
        self.pathPub = self.create_publisher(Path, path_topic, 10)
        
        self.mapSub = self.create_subscription(OccupancyGrid, map_topic, self.map_callback, 10)
        self.goalSub = self.create_subscription(PoseStamped, goal_topic, self.goal_callback, 10)

        self.mainTimer = self.create_timer(1 / frequency, self.timer_callback)

        self.pathData = Path()
        self.odomData = Odometry()
        self.mapData = OccupancyGrid()
        self.goalData = PoseStamped()
        
        self.grid = Grid(np.zeros((1, 1)), self.steeringVal, self.pathDiscrete / self.mapRes)
        self.finder = AstarFinder(self.pathCollisionRad / self.mapRes,
                                10,
                                self.goalRad / self.mapRes)

    def path_callback(self, msg):
        self.pathData = msg

    def odom_callback(self, msg):
        self.odomData = msg
    
    def map_callback(self, msg):
        self.mapData = msg
    
    def goal_callback(self, msg):
        self.goalData = msg


    def publish_tf(self):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.pathBaseFrame
        msg.child_frame_id = "path"
        msg.transform.translation.x = -self.odomData.pose.pose.position.x
        msg.transform.translation.y = -self.odomData.pose.pose.position.y
        msg.transform.translation.z = 0.0
        self.tfBroadcaster.sendTransform(msg)

# ros2 param set /waypoint_following start_following True

    def timer_callback(self):
        self.publish_tf()
        
        if self.goalData != PoseStamped() and self.mapData != OccupancyGrid():
            if not is_in_goal(self.odomData, self.goalData, self.goalRad):
                map_array = rnp.numpify(self.mapData)
                x1, y1, th1 = remap_robot_coord(self.odomData, map_array, self.mapRes)
                self.grid.init_grid(map_array)
                x2, y2, uturn = remap_goal_coord(self.goalData, map_array, self.mapRes)
                path = None
                if uturn and self.finder.uturnState != 3:
                    path = self.finder.get_uturn(self.grid, (x1, y1, th1))
                elif uturn and self.finder.uturnState == 3:
                    self.grid.neighbours = self.grid.forward_neighbours
                    path = self.finder.get_path(self.grid, (x1, y1, th1), (x2, y2))
                else:
                    self.finder.uturnState = 0
                    path = self.finder.get_path(self.grid, (x1, y1, th1), (x2, y2))

                msg = Path()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "path"
                if isinstance(path, list):
                    for pose in path:
                        p = PoseStamped()
                        p.pose.position.x = float(pose[0]) * self.mapRes - map_array.shape[0] * self.mapRes / 2
                        p.pose.position.y = float(pose[1]) * self.mapRes - map_array.shape[1] * self.mapRes / 2
                        p.pose.orientation.z = float(pose[2])
                        msg.poses.append(p)
                self.pathPub.publish(msg)

        
        
        if self.pathData.poses != [] and self.odomData != Odometry():
            pose = self.odomData.pose.pose
            path = self.pathData.poses
            steer = self.p_control_steer(pose, path)
            speed = self.p_control_speed(pose, path)
            if speed < 0:
                steer = steer
        else:
            speed = 0.
            steer = 0.
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = steer
        self.cmdPub.publish(msg)

    def p_control_steer(self, pose: Pose, path: list[PoseStamped]):
        angle = euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)[2]
        goal_pose = path[0].pose
        goal_angle = goal_pose.orientation.z
        delta_angle = goal_angle - angle
        p_ = self.pSteeringRatio * delta_angle
        return p_

    def p_control_speed(self, pose: Pose, path: list[PoseStamped]):
        robot_x, robot_y = pose.position.x, pose.position.y
        try:
            goal_x, goal_y = path[1].pose.position.x, path[1].pose.position.y
        except:
            return 0.

        transformed_goal_x, transformed_goal_y = goal_x - robot_x, goal_y - robot_y
        rho, th = decart_to_polar(transformed_goal_x, transformed_goal_y)

        angle = euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)[2]
        direction = abs(angle - th)
        if direction > 2.5:
            return -self.averageSpeed
        else:
            return self.averageSpeed


def main():
    rclpy.init()
    node = PathController("path_controller")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
