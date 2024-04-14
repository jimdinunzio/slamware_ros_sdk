import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='ip_address',
            default_value='192.168.11.1'
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
                    'ip_address': launch.substitutions.LaunchConfiguration('ip_address'),
                    'robot_port': 1445,
                    'reconn_wait_ms': 3000,

                    'angle_compensate': True,
                    'fixed_odom_map_tf': True,

                    'robot_frame': '/base_link',
                    'laser_frame': '/laser',
                    'map_frame': '/map',
                    'odom_frame': '/odom',
                    
                    'robot_pose_pub_period': 0.05,
                    'scan_pub_period': 0.1,
                    'map_update_period': 0.2,
                    'map_pub_period': 0.2,
                    'path_pub_period': 0.05,

                    'scan_topic': 'scan',
                    'odom_topic': 'odom/unfiltered',
                    'map_topic': 'map',
                    'map_info_topic': 'map_metadata',
                    'path_topic': 'global_plan_path',

                    'vel_control_topic': '/cmd_vel',
                    'goal_topic': launch.substitutions.LaunchConfiguration('goal_topic'),
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
