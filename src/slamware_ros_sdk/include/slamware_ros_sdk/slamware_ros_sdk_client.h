
#pragma once

#include <rclcpp/rclcpp.hpp>
#include "message_filters/subscriber.h"

#include "utils.h"

#include <slamware_ros_msgs/msg/vec2_d_int32.hpp>
#include <slamware_ros_msgs/msg/line2_d_flt32_array.hpp>
#include <slamware_ros_msgs/msg/rect_int32.hpp>
#include <slamware_ros_msgs/msg/robot_device_info.hpp>
#include <slamware_ros_msgs/msg/basic_sensor_info_array.hpp>
#include <slamware_ros_msgs/msg/basic_sensor_value_data_array.hpp>
#include <slamware_ros_msgs/msg/robot_basic_state.hpp>
#include <slamware_ros_msgs/msg/sync_map_request.hpp>
#include <slamware_ros_msgs/msg/move_by_direction_request.hpp>
#include <slamware_ros_msgs/msg/move_by_theta_request.hpp>
#include <slamware_ros_msgs/msg/move_to_request.hpp>
#include <slamware_ros_msgs/msg/move_to_locations_request.hpp>
#include <slamware_ros_msgs/msg/rotate_to_request.hpp>
#include <slamware_ros_msgs/msg/rotate_request.hpp>
#include <slamware_ros_msgs/msg/recover_localization_request.hpp>
#include <slamware_ros_msgs/msg/clear_map_request.hpp>
#include <slamware_ros_msgs/msg/set_map_update_request.hpp>
#include <slamware_ros_msgs/msg/set_map_localization_request.hpp>
#include <slamware_ros_msgs/msg/go_home_request.hpp>
#include <slamware_ros_msgs/msg/cancel_action_request.hpp>
#include <slamware_ros_msgs/msg/add_line_request.hpp>
#include <slamware_ros_msgs/msg/add_lines_request.hpp>
#include <slamware_ros_msgs/msg/remove_line_request.hpp>
#include <slamware_ros_msgs/msg/clear_lines_request.hpp>
#include <slamware_ros_msgs/msg/move_line_request.hpp>
#include <slamware_ros_msgs/msg/move_lines_request.hpp>

#include <slamware_ros_msgs/srv/sync_get_stcm.hpp>
#include <slamware_ros_msgs/srv/sync_set_stcm.hpp>

#include <boost/filesystem/path.hpp>

using namespace slamware_ros_msgs::msg;

namespace slamware_ros_sdk {

    class SlamwareRosSdkClient
    {
    public:
        typedef boost::filesystem::path             fs_path_t;

    public:
        explicit SlamwareRosSdkClient(rclcpp::Node::SharedPtr nhRos
            , const char* serverNodeName = nullptr
            , const char* msgNamePrefix = nullptr
            );
        ~SlamwareRosSdkClient();

    public:
        //////////////////////////////////////////////////////////////////////////
//        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
        void syncMap(const SyncMapRequest& msg) { return pubSyncMap_->publish(msg); }
        void setPose(const geometry_msgs::msg::Pose& msg) { pubSetPose_->publish(msg); }

        void recoverLocalization(const RecoverLocalizationRequest& msg) { pubRecoverLocalization_->publish(msg); }
        void clearMap(const ClearMapRequest& msg) { pubClearMap_->publish(msg); }
        void setMapUpdate(const SetMapUpdateRequest& msg) { pubSetMapUpdate_->publish(msg); }
        void setMapLocalization(const SetMapLocalizationRequest& msg) { pubSetMapLocalization_->publish(msg); }

        void moveBy(const MoveByDirectionRequest& msg) { pubMoveByDirection_->publish(msg); }
        void moveBy(const MoveByThetaRequest& msg) { pubMoveByTheta_->publish(msg); }
        void moveTo(const MoveToRequest& msg) { pubMoveTo_->publish(msg); }
        void moveTo(const MoveToLocationsRequest& msg) { pubMoveToLocations_->publish(msg); }
        void rotateTo(const RotateToRequest& msg) { pubRotateTo_->publish(msg); }
        void rotate(const RotateRequest& msg) { pubRotate_->publish(msg); }

        void goHome(const GoHomeRequest& msg) { pubGoHome_->publish(msg); }
        void cancelAction(const CancelActionRequest& msg) { pubCancelAction_->publish(msg); }

        void addLine(const AddLineRequest& msg) { pubAddLine_->publish(msg); }
        void addLines(const AddLinesRequest& msg) { pubAddLines_->publish(msg); }
        void removeLine(const RemoveLineRequest& msg) { pubRemoveLine_->publish(msg); }
        void clearLines(const ClearLinesRequest& msg) { pubClearLines_->publish(msg); }
        void moveLine(const MoveLineRequest& msg) { pubMoveLine_->publish(msg); }
        void moveLines(const MoveLinesRequest& msg) { pubMoveLines_->publish(msg); }

        //////////////////////////////////////////////////////////////////////////

//        bool syncGetStcm(SyncGetStcm& srvMsg) { return scSyncGetStcm_.call(srvMsg); }
        // get stcm and write to filePath.
        bool syncGetStcm(std::string& errMsg
                , const fs_path_t& filePath
        );

//        bool syncSetStcm(SyncSetStcm& srvMsg) { return scSyncSetStcm_.call(srvMsg); }
        // load stcm from filePath, and upload to slamware.
        bool syncSetStcm(const fs_path_t& filePath
                , const geometry_msgs::msg::Pose& robotPose
                , std::string& errMsg
        );

//        bool syncGetStcm(slamware_ros_msgs::srv::SyncGetStcm::Request::SharedPtr& srvMsg);
////        { return scSyncGetStcm_->async_send_request(srvMsg,syncGetStcm); }
//        // get stcm and write to filePath.
//        bool syncGetStcm(std::string& errMsg
//            , const fs_path_t& filePath
//            );
//
////        bool syncSetStcm(slamware_ros_msgs::srv::SyncSetStcm& srvMsg) { return scSyncSetStcm_.call(srvMsg); }
//        // load stcm from filePath, and upload to slamware.
//        bool syncSetStcm(const fs_path_t& filePath
//            , const geometry_msgs::msg::Pose& robotPose
//            , std::string& errMsg
//            );

        //////////////////////////////////////////////////////////////////////////

    private:
        std::string genTopicFullName_(const std::string& strName) const { return msgNamePrefix_ + strName; }

    private:
//        ros::NodeHandle* nh_;

        rclcpp::Node::SharedPtr nh_;
        std::string sdkServerNodeName_;
        std::string msgNamePrefix_;
//        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr targetspub;
        rclcpp::Publisher<SyncMapRequest>::SharedPtr  pubSyncMap_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pubSetPose_;

        rclcpp::Publisher<RecoverLocalizationRequest>::SharedPtr pubRecoverLocalization_;
        rclcpp::Publisher<ClearMapRequest>::SharedPtr  pubClearMap_;
        rclcpp::Publisher<SetMapUpdateRequest>::SharedPtr  pubSetMapUpdate_;
        rclcpp::Publisher<SetMapLocalizationRequest>::SharedPtr  pubSetMapLocalization_;

        rclcpp::Publisher<MoveByDirectionRequest>::SharedPtr  pubMoveByDirection_;
        rclcpp::Publisher<MoveByThetaRequest>::SharedPtr  pubMoveByTheta_;
        rclcpp::Publisher<MoveToRequest>::SharedPtr  pubMoveTo_;
        rclcpp::Publisher<MoveToLocationsRequest>::SharedPtr  pubMoveToLocations_;
        rclcpp::Publisher<RotateToRequest>::SharedPtr  pubRotateTo_;
        rclcpp::Publisher<RotateRequest>::SharedPtr  pubRotate_;

        rclcpp::Publisher<GoHomeRequest>::SharedPtr  pubGoHome_;
        rclcpp::Publisher<CancelActionRequest>::SharedPtr  pubCancelAction_;

        rclcpp::Publisher<AddLineRequest>::SharedPtr  pubAddLine_;
        rclcpp::Publisher<AddLinesRequest>::SharedPtr  pubAddLines_;
        rclcpp::Publisher<RemoveLineRequest>::SharedPtr  pubRemoveLine_;
        rclcpp::Publisher<ClearLinesRequest>::SharedPtr  pubClearLines_;
        rclcpp::Publisher<MoveLineRequest>::SharedPtr  pubMoveLine_;
        rclcpp::Publisher<MoveLinesRequest>::SharedPtr  pubMoveLines_;

        rclcpp::Client<slamware_ros_msgs::srv::SyncGetStcm>::SharedPtr scSyncGetStcm_;
        rclcpp::Client<slamware_ros_msgs::srv::SyncSetStcm>::SharedPtr  scSyncSetStcm_;

    };

}
