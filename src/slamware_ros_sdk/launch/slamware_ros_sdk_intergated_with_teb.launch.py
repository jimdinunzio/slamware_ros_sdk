import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='move_base',
            executable='move_base',
            name='move_base',
            output='screen',
            parameters=[
                {
                    'base_global_planner': 'global_planner/GlobalPlanner'
                },
                {
                    'planner_frequency': '1.0'
                },
                {
                    'planner_patience': '5.0'
                },
                {
                    'base_local_planner': 'teb_local_planner/TebLocalPlannerROS'
                },
                {
                    'controller_frequency': '5.0'
                },
                {
                    'controller_patience': '15.0'
                },
                get_package_share_directory(
                    'slamware_ros_sdk') + '/cfg/diff_drive/local_costmap_params.yaml',
                get_package_share_directory(
                    'slamware_ros_sdk') + '/cfg/diff_drive/global_costmap_params.yaml'
            ]
        ),
        launch_ros.actions.Node(
            package='rviz',
            executable='rviz',
            name='rviz'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'slamware_ros_sdk'), 'launch/slamware_ros_sdk_server_node.launch.py')
            ),
            launch_arguments={
                'move_base_goal_topic': 'disable',
                'ip_address': '192.168.11.1'
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
