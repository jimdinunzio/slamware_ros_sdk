import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='ip_address',
            default_value='192.168.1.37'
        ),
        launch.actions.DeclareLaunchArgument(
            name='move_base_goal_topic',
            default_value='/move_base_simple/goal'
        ),
        launch_ros.actions.Node(
            package='slamware_ros_sdk',
            executable='slamware_ros_sdk_server_node',
            name='slamware_ros_sdk_server_node',
            output='screen',
            parameters=[
                {
                    'ip_address': launch.substitutions.LaunchConfiguration('ip_address')
                },
                {
                    'angle_compensate': 'true'
                },
                {
                    'fixed_odom_map_tf': 'true'
                },
                {
                    'robot_frame': '/base_link'
                },
                {
                    'odom_frame': '/odom'
                },
                {
                    'laser_frame': '/laser'
                },
                {
                    'map_frame': '/slamware_map'
                },
                {
                    'robot_pose_pub_period': '0.05'
                },
                {
                    'scan_pub_period': '0.1'
                },
                {
                    'map_pub_period': '0.2'
                },
                {
                    'path_pub_period': '0.05'
                },
                {
                    '/cmd_vel': '/cmd_vel'
                },
                {
                    '/move_base_simple/goal': '/move_base_simple/goal'
                },
                {
                    'scan': 'scan'
                },
                {
                    'odom': 'odom'
                },
                {
                    'map': 'map'
                },
                {
                    'map_metadata': 'map_metadata'
                },
                {
                    'global_plan_path': 'global_plan_path'
                }
            ]
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map2odom',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                       '1.0', 'slamware_map', 'odom'],
        ),

        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base2laser',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                       '1.0', 'base_link', 'laser'],
        ),

    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
