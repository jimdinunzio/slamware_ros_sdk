import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

#write a ROS2 python launch file to run python program get_map.py that is installed in the install/slamware_ros_sdk/lib/slamware_ros_sdk folder and takes one required argument, filename
def generate_launch_description():
    slamware_ros_sdk_dir = get_package_prefix('slamware_ros_sdk')
    get_map_path = os.path.join(slamware_ros_sdk_dir, 'lib/slamware_ros_sdk', 'get_map.py')
    filename = LaunchConfiguration('filename')

    get_map_node = ExecuteProcess(
        cmd=['python3', get_map_path, filename],
        output='screen'
    )

    return LaunchDescription([
        get_map_node
    ])


