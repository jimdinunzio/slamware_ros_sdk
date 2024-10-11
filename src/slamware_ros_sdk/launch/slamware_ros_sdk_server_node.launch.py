import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='gps',
            default_value='False',
            description='gps mode: do not publish map to odom identity static tf'
        ),
        launch.actions.DeclareLaunchArgument(
            name='ip_address',
            default_value='192.168.11.1'
        ),
        launch.actions.DeclareLaunchArgument(
            name='robot_pose_pub_period',
            default_value='0.1',
            description='robot pose publisher period. 0 to disable publishing pose'
        ),
        launch.actions.DeclareLaunchArgument(
            name='map_pub_period',
            default_value='0.2',
            description='map update period. 0 to disable publishing map'
        ),
        launch.actions.DeclareLaunchArgument(
            name='goal_topic',
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
                    'fixed_odom_map_tf': False,

                    'robot_frame': 'base_link',
                    'laser_frame': 'laser',
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    
                    'robot_pose_pub_period': launch.substitutions.LaunchConfiguration('robot_pose_pub_period'),
                    'scan_pub_period': 0.2,
                    'map_update_period': 0.2,
                    'map_pub_period': launch.substitutions.LaunchConfiguration('map_pub_period'),
                    'path_pub_period': 0.0,

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
        # launch this node only if param gps:=false
        launch_ros.actions.Node(
            condition=launch.conditions.UnlessCondition(launch.substitutions.LaunchConfiguration('gps')),
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map2odom',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                       '1.0', 'map', 'odom'],
        ),

        # launch_ros.actions.Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base2laser',
        #     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
        #                '1.0', 'base_link', 'laser'],
        # ),

    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
