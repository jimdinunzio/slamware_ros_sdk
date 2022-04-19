from launch import LaunchDescription
from launch_ros.actions import Node
#
# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node


def generate_launch_description():
  # ip_address = LaunchConfiguration('ip_address', default="192.168.50.158")
  # move_base_goal_topic = LaunchConfiguration('move_base_goal_topic', default="/move_base_simple/goal")

  ld = LaunchDescription()
  ros_sdk_node = Node(
    package="slamware_ros_sdk",
    executable="slamware_ros_sdk_server_node",
    output='screen',
    parameters=[
      {"ip_address": "192.168.1.37"},
      {"angle_compensate": "true"},
      {"fixed_odom_map_tf": "true"}
    ]
  )

  ld.add_action(ros_sdk_node)

#   return ld
# <launch>
#   <arg name="ip_address" default="192.168.153.1" />
#   <arg name="move_base_goal_topic" default="/move_base_simple/goal"/>
#
#   <node name="slamware_ros_sdk_server_node" pkg="slamware_ros_sdk" type="slamware_ros_sdk_server_node" output="screen">
#
#   <param name = "ip_address"                value = "$(arg ip_address)"/>
#   <param name = "angle_compensate"         value = "true"/>
#   <param name = "fixed_odom_map_tf"         value = "true"/>
#
#   <param name = "robot_frame"              value = "/base_link"/>
#   <param name = "odom_frame"               value = "/odom"/>
#   <param name = "laser_frame"              value = "/laser"/>
#   <param name = "map_frame"                value = "/slamware_map"/>
#
#   <param name = "robot_pose_pub_period"    value = "0.05"/>
#   <param name = "scan_pub_period"          value = "0.1"/>
#   <param name = "map_pub_period"           value = "0.2"/>
#   <param name = "path_pub_period"          value = "0.05"/>
#
#   <param name = "/cmd_vel"                 value = "/cmd_vel"/>
#   <param name = "/move_base_simple/goal"   value = "/move_base_simple/goal"/>
#   <param name = "scan"                     value = "scan"/>
#   <param name = "odom"                     value = "odom"/>
#   <param name = "map"                      value = "map"/>
#   <param name = "map_metadata"             value = "map_metadata"/>
#   <param name = "global_plan_path"         value = "global_plan_path"/>
#   <!-- topic remap /-->
#   <remap from = "/cmd_vel"                   to = "/cmd_vel"/>
#   <remap from = "/move_base_simple/goal"     to = "$(arg move_base_goal_topic)"/>
#   <remap from = "scan"                       to = "scan"/>
#   <remap from = "odom"                       to = "odom"/>
#
#   <remap from = "map"                        to = "slamware_map"/>
#   <remap from = "map_metadata"               to = "map_metadata"/>
#   <remap from = "global_plan_path"           to = "global_plan_path"/>
#   </node>
#
#   <node name="map2odom" pkg="tf" type="static_transform_publisher" args="0 0 0 0 0 0 1 /slamware_map /odom 100"/>
#   <node name="base2laser" pkg="tf" type="static_transform_publisher" args="0 0 0 0 0 0 1 /base_link /laser 100"/>
#
#
#
# </launch>
