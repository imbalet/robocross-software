import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_path = get_package_share_directory('car_bot')
    launch_path = os.path.join(package_path, 'launch')
    param_file = os.path.join(get_package_share_directory("car_bot"), 'config', 'sim.yaml')

    action_simulation_launch = IncludeLaunchDescription(
        launch_description_source=os.path.join(launch_path, 'gz_test.launch.py')
    )

    action_rviz_launch = IncludeLaunchDescription(
        launch_description_source=os.path.join(launch_path, 'state_publisher.launch.py')
    )

    # action_front_cloud_to_scan = Node(
    #     package='car_bot',
    #     executable='cloud_to_scan.py',
    #     name='front_cloud_to_scan',
    #     parameters=[param_file]
    # )
    action_front_cloud_to_scan = Node(
        package='car_bot',
        executable='pc2scan',
        name='front_cloud_to_scan',
        parameters=[param_file]
    )

    # action_path_mapping = Node(
    #     package='car_bot',
    #     executable='path_mapping.py',
    #     name='path_mapping',
    #     output="screen",
    #     parameters=[param_file]
    # )
    
    action_path_mapping = Node(
        package='car_bot',
        executable='path_mapping',
        name='path_mapping',
        output="screen",
        parameters=[param_file]
    )

    action_path_controller = Node(
        package='car_bot',
        executable='path_controller.py',
        name='path_controller',
        parameters=[param_file]
    )

    action_waypoint_following = Node(
        package='car_bot',
        executable='waypoint_following.py',
        name='waypoint_following',
        parameters=[param_file]
    )
    
    # pc2ls = Node(
    #     package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
    #     remappings=[('cloud_in', '/front_camera/points'),
    #                 ('scan', '/front_camera/scan1')],
    #     parameters=[{
    #         # 'queue_size': 10.0,
    #         'target_frame': 'front_camera_link',
    #         'transform_tolerance': 0.01,
    #         'min_height': -0.15,
    #         'max_height': 0.0,
    #         'angle_min': -0.7,
    #         'angle_max': 0.7,
    #         'angle_increment': 1.4 / 200,
    #         'scan_time': 0.3333,
    #         'range_min': 0.0,
    #         'range_max': 24.0,
    #         'use_inf': True,
    #         'inf_epsilon': 1.0
    #     }],
    #     name='pointcloud_to_laserscan'
    # )

    ld = LaunchDescription()
    ld.add_action(action_simulation_launch)
    ld.add_action(action_rviz_launch)
    ld.add_action(action_front_cloud_to_scan)
    ld.add_action(action_path_mapping)
    # ld.add_action(action_path_controller)
    ld.add_action(action_waypoint_following)
    # ld.add_action(p2s)
    return ld
