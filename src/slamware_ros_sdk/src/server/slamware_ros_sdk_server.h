/**
 kint.zhao  huasheng_zyh@163.com
   2017.0721

  modified by yun.li@slamtec.com, 2019.
*/

#pragma once

#include "server_workers.h"
#include <slamware_ros_msgs/srv/sync_get_stcm.hpp>
#include <slamware_ros_msgs/srv/sync_set_stcm.hpp>

#include <message_filters/subscriber.h>

#include <boost/atomic.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>

#include <boost/function.hpp>
using namespace slamware_ros_msgs::msg;

namespace slamware_ros_sdk {

    class SlamwareRosSdkServer
    {
    private:
        friend class ServerWorkerBase;

    public:
        SlamwareRosSdkServer();
        ~SlamwareRosSdkServer();

        bool startRun(std::string& errMsg);
        void requestStop();
        void waitUntilStopped(); // not thread-safe
        rclcpp::Node::SharedPtr rosNodeHandle_() { return nh_; }

    public:
        void requestSyncMap();

    private:
        enum ServerState
        {
            ServerStateNotInit
            , ServerStateRunning
            , ServerStateStopped
        };

        typedef rpos::robot_platforms::SlamwareCorePlatform     slamware_platform_t;

        template<class MsgT>
        struct msg_cb_help_t
        {
            typedef MsgT                                    msg_t;
            typedef typename msg_t::ConstPtr                const_msg_shared_ptr;
            typedef void (SlamwareRosSdkServer::*msg_cb_perform_fun_t)(slamware_platform_t&, const const_msg_shared_ptr&);
            //Both the following are good
            typedef std::function< void(const const_msg_shared_ptr) >        ros_cb_fun_t; // callback function for ROS.
//            typedef std::function<void(std::shared_ptr<MsgT>)> ros_cb_fun_t; // callback function for ROS.
        };

        template<class SrvMsgT>
        struct srv_cb_help_t
        {
            typedef SrvMsgT srv_msg_t;
            typedef std::shared_ptr<typename srv_msg_t::Request> request_t;
            typedef std::shared_ptr<typename srv_msg_t::Response> response_t;

            typedef bool (SlamwareRosSdkServer::*srv_cb_perform_fun_t)(slamware_platform_t &, request_t, response_t);

            typedef std::function<bool(request_t, response_t)> ros_cb_fun_t; // callback function for ROS.
        };

//        template<class SrvMsgT>
//        struct srv_cb_help_t
//        {
//            typedef SrvMsgT                             srv_msg_t;
//            typedef typename srv_msg_t::Request         request_t;
//            typedef typename srv_msg_t::Response        response_t;
//            typedef bool (SlamwareRosSdkServer::*srv_cb_perform_fun_t)(slamware_platform_t&, request_t&, response_t&);
//            typedef std::function< bool(request_t, response_t) >            ros_cb_fun_t; // callback function for ROS.
//        };

    private:
        static boost::chrono::milliseconds sfConvFloatSecToBoostMs_(float fSec);

        bool isRunning_() const { return ServerStateRunning == state_.load(); }
        bool shouldContinueRunning_() const;

        const rclcpp::Node::SharedPtr rosNodeHandle_() const { return nh_; }


        const ServerParams& serverParams_() const { return params_; }
        
//        const tf2_ros::TransformBroadcaster& tfBroadcaster_() const { return tfBrdcstr_; }
        std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_() const { return tfBrdcstr_; }
//        tf2_ros::TransformBroadcaster tfBroadcaster_() { return tfBrdcstr_; }
        std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_() { return tfBrdcstr_; }


        ServerWorkData_ConstPtr safeGetWorkData_() const;
        ServerWorkData_Ptr safeGetMutableWorkData_();

        bool safeIsSlamwarePlatformConnected_() const;
        slamware_platform_t safeGetSlamwarePlatform_() const;
        void safeSetSlamwarePlatform_(const slamware_platform_t& pltfm);
        void safeReleaseSlamwarePlatform_();
        slamware_platform_t connectSlamwarePlatform_(const std::string& ip, int port) const;
        void disconnectSlamwarePlatform_(slamware_platform_t& pltfm) const;

        bool init_(std::string& errMsg);
        void cleanup_();

        void workThreadFun_();

        void roughSleepWait_(std::uint32_t maxSleepMs, std::uint32_t onceSleepMs);
        void loopTryConnectToSlamwarePlatform_();

        bool reinitWorkLoop_(slamware_platform_t& pltfm);
        void loopWork_();

        //        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr rviz_sub;
//        rviz_sub = nh->create_subscription<geometry_msgs::msg::PointStamped>("topic", 100, rvizCallBack);

        //////////////////////////////////////////////////////////////////////////
        // subscribed messages
        //////////////////////////////////////////////////////////////////////////

        template<class MsgT>
        void msgCbWrapperFun_T_(typename msg_cb_help_t<MsgT>::msg_cb_perform_fun_t mfpCbPerform
            , const std::string& msgTopic
            , const typename msg_cb_help_t<MsgT>::const_msg_shared_ptr & msg
            );

        template<class MsgT>
        typename rclcpp::Subscription<MsgT>::SharedPtr
        subscribe_T_(const std::string &msgTopic,std::uint32_t queueSize,
                                           typename msg_cb_help_t<MsgT>::msg_cb_perform_fun_t mfpCbPerform
        );

        void msgCbRobotControl_(slamware_platform_t& pltfm, const geometry_msgs::msg::Twist::ConstPtr& msg);
        void msgCbMoveToGoal_(slamware_platform_t& pltfm, const geometry_msgs::msg::PoseStamped::ConstPtr& msg);

        void msgCbSyncMap_(slamware_platform_t& pltfm, const SyncMapRequest::ConstPtr& msg);
        void msgCbSetPose_(slamware_platform_t& pltfm, const geometry_msgs::msg::Pose::ConstPtr& msg);

        void msgCbRecoverLocalization_(slamware_platform_t& pltfm, const RecoverLocalizationRequest::ConstPtr& msg);
        void msgCbClearMap_(slamware_platform_t& pltfm, const ClearMapRequest::ConstPtr& msg);
        void msgCbSetMapUpdate_(slamware_platform_t& pltfm, const SetMapUpdateRequest::ConstPtr& msg);
        void msgCbSetMapLocalization_(slamware_platform_t& pltfm, const SetMapLocalizationRequest::ConstPtr& msg);

        void msgCbMoveByDirection_(slamware_platform_t& pltfm, const MoveByDirectionRequest::ConstPtr& msg);
        void msgCbMoveByTheta_(slamware_platform_t& pltfm, const MoveByThetaRequest::ConstPtr& msg);
        void msgCbMoveTo_(slamware_platform_t& pltfm, const MoveToRequest::ConstPtr& msg);
        void msgCbMoveToLocations_(slamware_platform_t& pltfm, const MoveToLocationsRequest::ConstPtr& msg);
        void msgCbRotateTo_(slamware_platform_t& pltfm, const RotateToRequest::ConstPtr& msg);
        void msgCbRotate_(slamware_platform_t& pltfm, const RotateRequest::ConstPtr& msg);

        void msgCbGoHome_(slamware_platform_t& pltfm, const GoHomeRequest::ConstPtr& msg);
        void msgCbCancelAction_(slamware_platform_t& pltfm, const CancelActionRequest::ConstPtr& msg);

        void msgCbAddLine_(slamware_platform_t& pltfm, const AddLineRequest::ConstPtr& msg);
        void msgCbAddLines_(slamware_platform_t& pltfm, const AddLinesRequest::ConstPtr& msg);
        void msgCbRemoveLine_(slamware_platform_t& pltfm, const RemoveLineRequest::ConstPtr& msg);
        void msgCbClearLines_(slamware_platform_t& pltfm, const ClearLinesRequest::ConstPtr& msg);
        void msgCbMoveLine_(slamware_platform_t& pltfm, const MoveLineRequest::ConstPtr& msg);
        void msgCbMoveLines_(slamware_platform_t& pltfm, const MoveLinesRequest::ConstPtr& msg);

        //////////////////////////////////////////////////////////////////////////
        // advertised services
        //////////////////////////////////////////////////////////////////////////

        template<class SrvMsgT>
        bool srvCbWrapperFun_T_(typename srv_cb_help_t<SrvMsgT>::srv_cb_perform_fun_t mfpCbPerform
            , const std::string& srvMsgTopic
            , typename srv_cb_help_t<SrvMsgT>::request_t req
            , typename srv_cb_help_t<SrvMsgT>::response_t resp
            );

        template<class SrvMsgT>
        typename rclcpp::Service<SrvMsgT>::SharedPtr advertiseService_T_(const std::string& srvMsgTopic
                , typename srv_cb_help_t<SrvMsgT>::srv_cb_perform_fun_t mfpCbPerform
        );

//        ros::ServiceServer advertiseService_T_(const std::string& srvMsgTopic
//            , typename srv_cb_help_t<SrvMsgT>::srv_cb_perform_fun_t mfpCbPerform
//            );

        bool srvCbSyncGetStcm_(slamware_platform_t& pltfm, std::shared_ptr<slamware_ros_msgs::srv::SyncGetStcm::Request> req,
                               std::shared_ptr<slamware_ros_msgs::srv::SyncGetStcm::Response> resp);
        bool srvCbSyncSetStcm_(slamware_platform_t& pltfm, std::shared_ptr<slamware_ros_msgs::srv::SyncSetStcm::Request> req,
                               std::shared_ptr<slamware_ros_msgs::srv::SyncSetStcm::Response> resp);

        //////////////////////////////////////////////////////////////////////////

    private:
        boost::atomic<ServerState> state_;
        boost::atomic<bool> isStopRequested_;

        rclcpp::Node::SharedPtr nh_;
        ServerParams params_;

//        tf2_ros::TransformBroadcaster tfBrdcstr_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tfBrdcstr_;


        mutable boost::mutex workDatLock_;
        ServerWorkData_Ptr workDat_;

        std::vector<ServerWorkerBase_Ptr> serverWorkers_;

        // subscriptions
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_sub;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subRobotControl_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subMoveToGoal_;
        
        rclcpp::Subscription<SyncMapRequest>::SharedPtr subSyncMap_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subSetPose_;

        rclcpp::Subscription<RecoverLocalizationRequest>::SharedPtr subRecoverLocalization_;
        rclcpp::Subscription<ClearMapRequest>::SharedPtr subClearMap_;
        rclcpp::Subscription<SetMapUpdateRequest>::SharedPtr subSetMapUpdate_;
        rclcpp::Subscription<SetMapLocalizationRequest>::SharedPtr subSetMapLocalization_;

        rclcpp::Subscription<MoveByDirectionRequest>::SharedPtr subMoveByDirection_;
        rclcpp::Subscription<MoveByThetaRequest>::SharedPtr subMoveByTheta_;
        rclcpp::Subscription<MoveToRequest>::SharedPtr subMoveTo_;
        rclcpp::Subscription<MoveToLocationsRequest>::SharedPtr subMoveToLocations_;
        rclcpp::Subscription<RotateToRequest>::SharedPtr subRotateTo_;
        rclcpp::Subscription<RotateRequest>::SharedPtr subRotate_;

        rclcpp::Subscription<GoHomeRequest>::SharedPtr subGoHome_;
        rclcpp::Subscription<CancelActionRequest>::SharedPtr subCancelAction_;

        rclcpp::Subscription<AddLineRequest>::SharedPtr subAddLine_;
        rclcpp::Subscription<AddLinesRequest>::SharedPtr subAddLines_;
        rclcpp::Subscription<RemoveLineRequest>::SharedPtr subRemoveLine_;
        rclcpp::Subscription<ClearLinesRequest>::SharedPtr subClearLines_;
        rclcpp::Subscription<MoveLineRequest>::SharedPtr subMoveLine_;
        rclcpp::Subscription<MoveLinesRequest>::SharedPtr subMoveLines_;
        
        rpos::actions::VelocityControlMoveAction velocityControllAction_;
        
        // services
        std::shared_ptr<rclcpp::Service<slamware_ros_msgs::srv::SyncGetStcm>> srvSyncGetStcm_;
        std::shared_ptr<rclcpp::Service<slamware_ros_msgs::srv::SyncSetStcm>> srvSyncSetStcm_;

        boost::thread workThread_;

        mutable boost::mutex slamwarePltfmLock_;
        slamware_platform_t slamwarePltfm_;
    };
    
}
