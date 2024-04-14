
#include "server_params.h"

#include <cmath>

namespace slamware_ros_sdk {

    //////////////////////////////////////////////////////////////////////////

    const float C_FLT_PI = ((float)M_PI);
    const float C_FLT_2PI = (C_FLT_PI * 2);

    //////////////////////////////////////////////////////////////////////////

    ServerParams::ServerParams()
    {
        resetToDefault();
    }

    void ServerParams::declareParams(const rclcpp::Node::SharedPtr nhRos)
    {
        nhRos->declare_parameter("ip_address", rclcpp::ParameterValue(ip_address));
        nhRos->declare_parameter("robot_port", rclcpp::ParameterValue(robot_port));
        nhRos->declare_parameter("reconn_wait_ms", rclcpp::ParameterValue(reconn_wait_ms));

        nhRos->declare_parameter("angle_compensate", rclcpp::ParameterValue(angle_compensate));
        nhRos->declare_parameter("fixed_odom_map_tf", rclcpp::ParameterValue(fixed_odom_map_tf));

        nhRos->declare_parameter("robot_frame", rclcpp::ParameterValue(robot_frame));
        nhRos->declare_parameter("laser_frame", rclcpp::ParameterValue(laser_frame));
        nhRos->declare_parameter("map_frame", rclcpp::ParameterValue(map_frame));
        nhRos->declare_parameter("odom_frame", rclcpp::ParameterValue(odom_frame));

        nhRos->declare_parameter("robot_pose_pub_period", rclcpp::ParameterValue(robot_pose_pub_period));
        nhRos->declare_parameter("scan_pub_period", rclcpp::ParameterValue(scan_pub_period));
        nhRos->declare_parameter("map_update_period", rclcpp::ParameterValue(map_update_period));
        nhRos->declare_parameter("map_pub_period", rclcpp::ParameterValue(map_pub_period));
        nhRos->declare_parameter("basic_sensors_info_update_period", rclcpp::ParameterValue(basic_sensors_info_update_period));
        nhRos->declare_parameter("basic_sensors_values_pub_period", rclcpp::ParameterValue(basic_sensors_values_pub_period));
        nhRos->declare_parameter("path_pub_period", rclcpp::ParameterValue(path_pub_period));
        nhRos->declare_parameter("robot_basic_state_pub_period", rclcpp::ParameterValue(robot_basic_state_pub_period));
        nhRos->declare_parameter("virtual_walls_pub_period", rclcpp::ParameterValue(virtual_walls_pub_period));
        nhRos->declare_parameter("virtual_tracks_pub_period", rclcpp::ParameterValue(virtual_tracks_pub_period));

        nhRos->declare_parameter("map_sync_once_get_max_wh", rclcpp::ParameterValue(map_sync_once_get_max_wh));
        nhRos->declare_parameter("map_update_near_robot_half_wh", rclcpp::ParameterValue(map_update_near_robot_half_wh));

        nhRos->declare_parameter("scan_topic", rclcpp::ParameterValue(scan_topic));
        nhRos->declare_parameter("odom_topic", rclcpp::ParameterValue(odom_topic));
        nhRos->declare_parameter("map_topic", rclcpp::ParameterValue(map_topic));
        nhRos->declare_parameter("map_info_topic", rclcpp::ParameterValue(map_info_topic));
        nhRos->declare_parameter("basic_sensors_info_topic", rclcpp::ParameterValue(basic_sensors_info_topic));
        nhRos->declare_parameter("basic_sensors_values_topic", rclcpp::ParameterValue(basic_sensors_values_topic));
        nhRos->declare_parameter("path_topic", rclcpp::ParameterValue(path_topic));

        nhRos->declare_parameter("vel_control_topic", rclcpp::ParameterValue(vel_control_topic));
        nhRos->declare_parameter("goal_topic", rclcpp::ParameterValue(goal_topic));
    }

    void ServerParams::resetToDefault()
    {
        ip_address = "192.168.11.1";
        robot_port = 1445;
        reconn_wait_ms = (1000U * 3U);

        angle_compensate = true;
        fixed_odom_map_tf = true;

        robot_frame = "/base_link";
        laser_frame = "/laser";
        map_frame = "/map";
        odom_frame = "/odom";

        robot_pose_pub_period = 0.05f;
        scan_pub_period = 0.1f;
        map_update_period = 0.2f;
        map_pub_period = 0.2f;
        basic_sensors_info_update_period = 7.0f;
        basic_sensors_values_pub_period = 0.05f;
        path_pub_period = 0.05f;
        robot_basic_state_pub_period = 1.0f;
        virtual_walls_pub_period = 0.5f;
        virtual_tracks_pub_period = 0.5f;

        map_sync_once_get_max_wh = 100.f;
        map_update_near_robot_half_wh = 8.0f;

        scan_topic = "scan";
        odom_topic = "odom";
        map_topic = "map";
        map_info_topic = "map_metadata";
        basic_sensors_info_topic = "basic_sensors_info";
        basic_sensors_values_topic = "basic_sensors_values";
        path_topic = "global_plan_path";

        vel_control_topic = "/cmd_vel";
        goal_topic = "/move_base_simple/goal";
    }

    void ServerParams::setBy(const rclcpp::Node::SharedPtr nhRos)
    {
        nhRos->get_parameter("ip_address", ip_address);
        nhRos->get_parameter("robot_port", robot_port);
        nhRos->get_parameter("reconn_wait_ms", reconn_wait_ms);

        nhRos->get_parameter("angle_compensate", angle_compensate);
        nhRos->get_parameter("fixed_odom_map_tf", fixed_odom_map_tf);

        nhRos->get_parameter("robot_frame", robot_frame);
        nhRos->get_parameter("laser_frame", laser_frame);
        nhRos->get_parameter("map_frame", map_frame);
        nhRos->get_parameter("odom_frame", odom_frame);

        nhRos->get_parameter("robot_pose_pub_period", robot_pose_pub_period);
        nhRos->get_parameter("scan_pub_period", scan_pub_period);
        nhRos->get_parameter("map_update_period", map_update_period);
        nhRos->get_parameter("map_pub_period", map_pub_period);
        nhRos->get_parameter("basic_sensors_info_update_period", basic_sensors_info_update_period);
        nhRos->get_parameter("basic_sensors_values_pub_period", basic_sensors_values_pub_period);
        nhRos->get_parameter("path_pub_period", path_pub_period);
        nhRos->get_parameter("robot_basic_state_pub_period", robot_basic_state_pub_period);
        nhRos->get_parameter("virtual_walls_pub_period", virtual_walls_pub_period);
        nhRos->get_parameter("virtual_tracks_pub_period", virtual_tracks_pub_period);

        nhRos->get_parameter("map_sync_once_get_max_wh", map_sync_once_get_max_wh);
        nhRos->get_parameter("map_update_near_robot_half_wh", map_update_near_robot_half_wh);

        nhRos->get_parameter("scan_topic", scan_topic);
        nhRos->get_parameter("odom_topic", odom_topic);
        nhRos->get_parameter("map_topic", map_topic);
        nhRos->get_parameter("map_info_topic", map_info_topic);
        nhRos->get_parameter("basic_sensors_info_topic", basic_sensors_info_topic);
        nhRos->get_parameter("basic_sensors_values_topic", basic_sensors_values_topic);
        nhRos->get_parameter("path_topic", path_topic);

        nhRos->get_parameter("vel_control_topic", vel_control_topic);
        nhRos->get_parameter("goal_topic", goal_topic);
    }

    //////////////////////////////////////////////////////////////////////////
    
}
