from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
)
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("car_bot")

    costmap_node = LifecycleNode(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        name="costmap",
        namespace="costmap",
        output="screen",
        parameters=[os.path.join(pkg_dir, "config", "sim.yaml")],
    )
    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        parameters=[
            {
                "autostart": True,
                "node_names": ["/costmap/costmap"],
                "bond_timeout": 0.0,
                "use_sim_time": True,
            }
        ],
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(os.path.join(pkg_dir, "launch", "gz.launch.py")),
            IncludeLaunchDescription(
                os.path.join(pkg_dir, "launch", "state_publisher.launch.py")
            ),
            IncludeLaunchDescription(
                os.path.join(pkg_dir, "launch", "robot_localization_ekf.launch.py")
            ),
            Node(
                package="car_bot",
                executable="point_cloud_to_scan",
                name="front_cloud_to_scan",
                parameters=[os.path.join(pkg_dir, "config", "sim.yaml")],
            ),
            # Node(
            #     package="car_bot",
            #     executable="odometry.py",
            #     name="odometry",
            #     parameters=[os.path.join(pkg_dir, "config", "sim.yaml")],
            # ),
            Node(
                package="car_bot",
                executable="pathfinder.py",
                name="pathfinder",
                parameters=[os.path.join(pkg_dir, "config", "sim.yaml")],
            ),
            TimerAction(period=7.0, actions=[costmap_node]),
            TimerAction(period=13.0, actions=[lifecycle_manager_node]),
        ]
    )
