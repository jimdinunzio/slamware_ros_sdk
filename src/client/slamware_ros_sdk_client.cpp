
#include <slamware_ros_sdk/slamware_ros_sdk_client.h>

#include <boost/assert.hpp>
#include <boost/filesystem/fstream.hpp>

namespace slamware_ros_sdk
{

    SlamwareRosSdkClient::SlamwareRosSdkClient(rclcpp::Node::SharedPtr nhRos, const char *serverNodeName,
                                               const char *msgNamePrefix
    )
            : nh_(nhRos)
    {
        if (nullptr != serverNodeName)
        {
            sdkServerNodeName_ = serverNodeName;
        }
        else
        {
            sdkServerNodeName_ = "slamware_ros_sdk_server_node";
        }

        if (nullptr != msgNamePrefix)
        {
            msgNamePrefix_ = msgNamePrefix;
        }
        else if (!sdkServerNodeName_.empty())
        {
            if ('/' != sdkServerNodeName_.front())
            {
                msgNamePrefix_ = "/";
            }
            msgNamePrefix_ += sdkServerNodeName_;
            if ('/' != msgNamePrefix_.back())
            {
                msgNamePrefix_ += "/";
            }
        }

        // initialize publishers
        {
//            argetspub = nh->create_publisher<geometry_msgs::msg::PointStamped>("/detected_points", 10);
            pubSyncMap_ = nh_->create_publisher<SyncMapRequest>(genTopicFullName_("sync_map"), 1);
            pubSetPose_ = nh_->create_publisher<geometry_msgs::msg::Pose>(genTopicFullName_("set_pose"), 1);

            pubRecoverLocalization_ = nh_->create_publisher<RecoverLocalizationRequest>(
                    genTopicFullName_("recover_localization"), 1);
            pubClearMap_ = nh_->create_publisher<ClearMapRequest>(genTopicFullName_("clear_map"), 1);
            pubSetMapUpdate_ = nh_->create_publisher<SetMapUpdateRequest>(genTopicFullName_("set_map_update"), 1);
            pubSetMapLocalization_ = nh_->create_publisher<SetMapLocalizationRequest>(
                    genTopicFullName_("set_map_localization"), 1);

            pubMoveByDirection_ = nh_->create_publisher<MoveByDirectionRequest>(genTopicFullName_("move_by_direction"),
                                                                                1);
            pubMoveByTheta_ = nh_->create_publisher<MoveByThetaRequest>(genTopicFullName_("move_by_theta"), 1);
            pubMoveTo_ = nh_->create_publisher<MoveToRequest>(genTopicFullName_("move_to"), 1);
            pubMoveToLocations_ = nh_->create_publisher<MoveToLocationsRequest>(genTopicFullName_("move_to_locations"),
                                                                                1);
            pubRotateTo_ = nh_->create_publisher<RotateToRequest>(genTopicFullName_("rotate_to"), 1);
            pubRotate_ = nh_->create_publisher<RotateRequest>(genTopicFullName_("rotate"), 1);

            pubGoHome_ = nh_->create_publisher<GoHomeRequest>(genTopicFullName_("go_home"), 1);
            pubCancelAction_ = nh_->create_publisher<CancelActionRequest>(genTopicFullName_("cancel_action"), 1);

            pubAddLine_ = nh_->create_publisher<AddLineRequest>(genTopicFullName_("add_line"), 1);
            pubAddLines_ = nh_->create_publisher<AddLinesRequest>(genTopicFullName_("add_lines"), 1);
            pubRemoveLine_ = nh_->create_publisher<RemoveLineRequest>(genTopicFullName_("remove_line"), 1);
            pubClearLines_ = nh_->create_publisher<ClearLinesRequest>(genTopicFullName_("clear_lines"), 1);
            pubMoveLine_ = nh_->create_publisher<MoveLineRequest>(genTopicFullName_("move_line"), 1);
            pubMoveLines_ = nh_->create_publisher<MoveLinesRequest>(genTopicFullName_("move_lines"), 1);
        }

//        // initialize service clients
//        {
//
        scSyncGetStcm_ = nh_->create_client<slamware_ros_msgs::srv::SyncGetStcm>(genTopicFullName_("sync_get_stcm"));
        scSyncSetStcm_ = nh_->create_client<slamware_ros_msgs::srv::SyncSetStcm>(genTopicFullName_("sync_set_stcm"));
//        }
    }

    SlamwareRosSdkClient::~SlamwareRosSdkClient()
    {
        //
    }

    bool SlamwareRosSdkClient::syncGetStcm(std::string &errMsg, const fs_path_t &filePath
    )
    {
        errMsg.clear();
        slamware_ros_msgs::srv::SyncGetStcm::Request::SharedPtr srvMsg;
        auto result = scSyncGetStcm_->async_send_request(srvMsg);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(nh_, result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {

            boost::filesystem::ofstream ofs(filePath,
                                            (std::ios_base::out | std::ios_base::trunc | std::ios_base::binary));
            if (!ofs.is_open())
            {
                errMsg = "failed to open file.";
                return false;
            }

            ofs.write((const char *) result.get()->raw_stcm.data(), result.get()->raw_stcm.size());
//            ofs.write((const char*)srvMsg.response.raw_stcm.data(), srvMsg.response.raw_stcm.size());
            if (ofs.fail())
            {
                errMsg = "failed to write file";
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "raw_stcm size %ld wrote to file", result.get()->raw_stcm.size());
        }
        else
        {
            errMsg = "failed to call syncGetStcm().";

            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call syncGetStcm()");
            return false;
        }
        return true;
    }

    bool SlamwareRosSdkClient::syncSetStcm(const fs_path_t &filePath, const geometry_msgs::msg::Pose &robotPose,
                                           std::string &errMsg
    )
    {
        errMsg.clear();

        slamware_ros_msgs::srv::SyncSetStcm::Request::SharedPtr srvMsg;
//        SyncSetStcm srvMsg;
        srvMsg->robot_pose = robotPose;

        boost::filesystem::ifstream ifs(filePath, (std::ios_base::in | std::ios_base::binary | std::ios_base::ate));
        if (!ifs.is_open())
        {
            errMsg = "failed to open file";
            return false;
        }
        const auto szDat = ifs.tellg();
        if (boost::filesystem::ifstream::pos_type(-1) == szDat)
        {
            errMsg = "failed to get file size.";
            return false;
        }
        ifs.seekg(0);

        srvMsg->raw_stcm.resize(szDat);
        ifs.read((char *) srvMsg->raw_stcm.data(), szDat);
        if (ifs.gcount() != szDat)
        {
            errMsg = "failed to read file data.";
            return false;
        }


        auto result = scSyncSetStcm_->async_send_request(srvMsg);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(nh_, result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Success to call syncSetStcm()");
            return true;
        }
        else
        {
            errMsg = "failed to call syncSetStcm().";
            return false;
        }

        return true;
    }

//    bool SlamwareRosSdkClient::syncGetStcm(std::string& errMsg
//        , const fs_path_t& filePath
//        )
//    {
//        errMsg.clear();
//
//        slamware_ros_msgs::srv::SyncGetStcm srvMsg;
//        if (!syncGetStcm(srvMsg))
//        {
//            errMsg = "failed to call syncGetStcm().";
//            return false;
//        }
//
//        {
//            boost::filesystem::ofstream ofs(filePath, (std::ios_base::out | std::ios_base::trunc | std::ios_base::binary));
//            if (!ofs.is_open())
//            {
//                errMsg = "failed to open file.";
//                return false;
//            }
//            ofs.write((const char*)srvMsg.response.raw_stcm.data(), srvMsg.response.raw_stcm.size());
//            if (ofs.fail())
//            {
//                errMsg = "failed to write file";
//                return false;
//            }
//        }
//        return true;
//    }

//    bool SlamwareRosSdkClient::syncSetStcm(const fs_path_t& filePath
//        , const geometry_msgs::msg::Pose& robotPose
//        , std::string& errMsg
//        )
//    {
//        errMsg.clear();
//
//        slamware_ros_msgs::srv::SyncSetStcm srvMsg;
//        srvMsg.request.robot_pose = robotPose;
//        {
//            boost::filesystem::ifstream ifs(filePath, (std::ios_base::in | std::ios_base::binary | std::ios_base::ate));
//            if (!ifs.is_open())
//            {
//                errMsg = "failed to open file";
//                return false;
//            }
//            const auto szDat = ifs.tellg();
//            if (boost::filesystem::ifstream::pos_type(-1) == szDat)
//            {
//                errMsg = "failed to get file size.";
//                return false;
//            }
//            ifs.seekg(0);
//
//            srvMsg.request.raw_stcm.resize(szDat);
//            ifs.read((char*)srvMsg.request.raw_stcm.data(), szDat);
//            if (ifs.gcount() != szDat)
//            {
//                errMsg = "failed to read file data.";
//                return false;
//            }
//        }
//
//        if (!syncSetStcm(srvMsg))
//        {
//            errMsg = "failed to call syncSetStcm().";
//            return false;
//        }
//        return true;
//    }

}
