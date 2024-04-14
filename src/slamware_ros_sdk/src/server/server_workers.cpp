
#include "server_workers.h"
#include "slamware_ros_sdk_server.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/assert.hpp>

namespace slamware_ros_sdk {

    //////////////////////////////////////////////////////////////////////////

    ServerRobotDeviceInfoWorker::ServerRobotDeviceInfoWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : super_t(pRosSdkServer, wkName, triggerInterval)
    {
        rclcpp::Node::SharedPtr nhRos = rosNodeHandle();
        pubRobotDeviceInfo_ = nhRos->create_publisher<RobotDeviceInfo>("robot_device_info", 1);
//        pubRobotDeviceInfo_ = nhRos->create_publisher<RobotDeviceInfo>("robot_device_info", 1, true);
    }

    ServerRobotDeviceInfoWorker::~ServerRobotDeviceInfoWorker()
    {
        //
    }

    bool ServerRobotDeviceInfoWorker::reinitWorkLoop(slamware_platform_t& pltfm)
    {
        if (!this->super_t::reinitWorkLoop(pltfm))
            return false;
        isWorkLoopInitOk_ = false;

        auto wkDat = mutableWorkData();
        auto& msgRobotDeviceInfo = wkDat->robotDeviceInfo;

        const auto devInfo = pltfm.getDeviceInfo();
        msgRobotDeviceInfo.device_id = devInfo.deviceID();
        msgRobotDeviceInfo.model_id = devInfo.modelID();
        msgRobotDeviceInfo.model_name = devInfo.modelName();
        msgRobotDeviceInfo.manufacturer_id = devInfo.manufacturerID();
        msgRobotDeviceInfo.manufacturer_name = devInfo.manufacturerName();
        msgRobotDeviceInfo.hardware_version = devInfo.hardwareVersion();
        msgRobotDeviceInfo.software_version = devInfo.softwareVersion();

        msgRobotDeviceInfo.sdp_version = pltfm.getSDPVersion();
        msgRobotDeviceInfo.sdk_version = pltfm.getSDKVersion();

        pubRobotDeviceInfo_->publish(msgRobotDeviceInfo);
        RCLCPP_INFO(rosNodeHandle()->get_logger(),"device_id: %s, hardware_version: %s, software_version: %s, sdp_version: %s, sdk_version: %s."
            , msgRobotDeviceInfo.device_id.c_str()
            , msgRobotDeviceInfo.hardware_version.c_str()
            , msgRobotDeviceInfo.software_version.c_str()
            , msgRobotDeviceInfo.sdp_version.c_str()
            , msgRobotDeviceInfo.sdk_version.c_str()
            );
        isWorkLoopInitOk_ = true;
        return isWorkLoopInitOk_;
    }

    void ServerRobotDeviceInfoWorker::doPerform(slamware_platform_t& /*pltfm*/)
    {
        // do nothing
    }

    //////////////////////////////////////////////////////////////////////////

    ServerRobotPoseWorker::ServerRobotPoseWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
    {
        const auto& srvParams = serverParams();
        rclcpp::Node::SharedPtr nhRos = rosNodeHandle();
        pubRobotPose_ = nhRos->create_publisher<nav_msgs::msg::Odometry>(srvParams.odom_topic, 10);
    }

    ServerRobotPoseWorker::~ServerRobotPoseWorker()
    {
        //
    }

//    geometry_msgs::msg::TransformStamped constuct_transform(rpos::core::Pose p)
//    {
//        geometry_msgs::msg::TransformStamped transformStamped;
//        transformStamped.header.stamp = rosNodeHandle()->now();
//        transformStamped.header.frame_id = map_frame;
//        transformStamped.child_frame_id = odom_frame;
//        transformStamped.transform.translation.x = p.x();
//        transformStamped.transform.translation.y = p.y();
//        transformStamped.transform.translation.z = p.z();
//        transformStamped.transform.rotation.x = 0;
//        transformStamped.transform.rotation.y = 0;
//        transformStamped.transform.rotation.z = 0;
//        transformStamped.transform.rotation.w = 1;
//        return transformStamped;
//    }

    void ServerRobotPoseWorker::doPerform(slamware_platform_t& pltfm)
    {
        const auto& srvParams = serverParams();
        auto tfBrdcst = tfBroadcaster();
        auto wkDat = mutableWorkData();

        // send TF transform
        if (srvParams.fixed_odom_map_tf) // only for debug rosrun
        {

//            std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
//            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
////
//            geometry_msgs::msg::TransformStamped transformStamped;
//            transformStamped.header.stamp = rosNodeHandle()->now();
//            transformStamped.header.frame_id = "odom";
//            transformStamped.child_frame_id = "base_link";
//            transformStamped.transform.translation.z = 0.0;
//            transformStamped.transform.rotation.x = 0.0;
//            transformStamped.transform.rotation.y = 0.0;
//            tf_broadcaster_->sendTransform(transformStamped);



            tf2::Transform tfIdenty;
            //fouad
//            tfIdenty.setIdentity();
            tfIdenty.setOrigin(tf2::Vector3 (0.0, 0.0, 0.0));
            tfIdenty.setRotation(tf2::Quaternion(0, 0, 0, 1));

            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped.header.stamp = rosNodeHandle()->now();
            transformStamped.header.frame_id = srvParams.map_frame;
            transformStamped.child_frame_id = srvParams.odom_frame;
            transformStamped.transform.translation.x = 0.0;
            transformStamped.transform.translation.y = 0.0;
            transformStamped.transform.translation.z = 0.0;
            transformStamped.transform.rotation.x = 0;
            transformStamped.transform.rotation.y = 0;
            transformStamped.transform.rotation.z = 0;
            transformStamped.transform.rotation.w = 1;

            tfBrdcst->sendTransform(transformStamped);
            //tfBrdcst.sendTransform(tf::StampedTransform(tfIdenty, ros::Time::now(), srvParams.robot_frame, srvParams.laser_frame));
        }

        // check power
        //int battPercentage = pltfm.getBatteryPercentage();
        //if (battPercentage < 10)
        //    std::cout << "lower power!! Battery: " << battPercentage << "%." << std::endl;

        //const rpos::core::Location location = pltfm.getLocation();
        const rpos::core::Pose robotPose = pltfm.getPose();
        wkDat->robotPose = robotPose;

        // publish odom transform
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = rosNodeHandle()->now();
        transformStamped.header.frame_id = srvParams.odom_frame;
        transformStamped.child_frame_id = srvParams.robot_frame;
        transformStamped.transform.translation.x = robotPose.x();
        transformStamped.transform.translation.y = robotPose.y();
        transformStamped.transform.translation.z = robotPose.z();

        tf2::Matrix3x3 obs_mat;
        obs_mat.setEulerYPR(robotPose.yaw(),0.0,0.0);

        tf2::Quaternion q_tf;
        obs_mat.getRotation(q_tf);
        transformStamped.transform.rotation.x = q_tf.getX();
        transformStamped.transform.rotation.y = q_tf.getY();
        transformStamped.transform.rotation.z = q_tf.getZ();
        transformStamped.transform.rotation.w = q_tf.getW();
        tfBrdcst->sendTransform(transformStamped);

//        tf2::Transform transform;
//        transform.setOrigin(tf2::Vector3(robotPose.x(), robotPose.y(), 0.0));
//        tf2::Quaternion q = tf2::createQuaternionFromYaw(robotPose.yaw());
//        transform.setRotation(q);
//        tfBrdcst->sendTransform(tf::StampedTransform(transform, rclcpp::Time, srvParams.odom_frame, srvParams.robot_frame));

        // send TF transform
        nav_msgs::msg::Odometry msgRobotPose;
        msgRobotPose.header.frame_id = srvParams.odom_frame;
        msgRobotPose.header.stamp = rosNodeHandle()->now();
        sltcToRosMsg(robotPose, msgRobotPose.pose.pose);
        pubRobotPose_->publish(msgRobotPose);
    }

    //////////////////////////////////////////////////////////////////////////

    ServerExploreMapUpdateWorker::ServerExploreMapUpdateWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : super_t(pRosSdkServer, wkName, triggerInterval)
        , shouldReinitMap_(true)
    {
        //
    }

    ServerExploreMapUpdateWorker::~ServerExploreMapUpdateWorker()
    {
        //
    }

    bool ServerExploreMapUpdateWorker::reinitWorkLoop(slamware_platform_t& pltfm)
    {
        if (!this->super_t::reinitWorkLoop(pltfm))
            return false;
        isWorkLoopInitOk_ = false;

        requestReinitMap_();

        isWorkLoopInitOk_ = true;
        return isWorkLoopInitOk_;
    }

    void ServerExploreMapUpdateWorker::doPerform(slamware_platform_t& pltfm)
    {
        const auto& srvParams = serverParams();
        auto wkDat = mutableWorkData();

        if (!checkToReinitMap_(pltfm, wkDat))
            return;

        if (wkDat->syncMapRequested.load())
        {
            RCLCPP_INFO(rosNodeHandle()->get_logger() ,"try to sync whold explore map.");
            if (syncWholeMap_(srvParams, pltfm, wkDat))
            {
                wkDat->syncMapRequested.store(false);
                RCLCPP_INFO(rosNodeHandle()->get_logger() ,"whold explore map synchronized.");
            }
            else
            {
                RCLCPP_INFO(rosNodeHandle()->get_logger() ,"failed to sync whole explore map.");
            }
            return;
        }

        updateMapNearRobot_(srvParams, pltfm, wkDat);
    }

    rpos::features::location_provider::Map ServerExploreMapUpdateWorker::getMapByPltfm_(slamware_platform_t& pltfm, const rpos::core::RectangleF& area) const
    {
        try
        {
            return pltfm.getMap(rpos::features::location_provider::MapTypeBitmap8Bit, area, rpos::features::location_provider::EXPLORERMAP);
        }
        catch (const rpos::robot_platforms::OperationFailException&)
        {
            //
        }
        return rpos::features::location_provider::Map();
    }

    void ServerExploreMapUpdateWorker::requestReinitMap_()
    {
        shouldReinitMap_ = true;
    }

    bool ServerExploreMapUpdateWorker::checkToReinitMap_(slamware_platform_t& pltfm, const ServerWorkData_Ptr& wkDat)
    {
        if (!shouldReinitMap_)
            return true;

        RCLCPP_INFO(rosNodeHandle()->get_logger() ,"try to reinit explore map.");
        wkDat->syncMapRequested.store(true);

        const float fHalfWH = 0.5f;
        const float fWH = fHalfWH + fHalfWH;
        const auto tArea = rpos::core::RectangleF(-fHalfWH, -fHalfWH, fWH, fWH);
        auto hMap = getMapByPltfm_(pltfm, tArea);
        if (!hMap)
        {
            RCLCPP_INFO(rosNodeHandle()->get_logger() ,"failed get map when init explore map.");
            return false;
        }
        const auto& mapResolution = hMap.getMapResolution();
        wkDat->exploreMapHolder.reinit(mapResolution.x());

        RCLCPP_INFO(rosNodeHandle()->get_logger() ,"explore map initialized, resolution: %f, moreCellCntToExtend: %d."
            , mapResolution.x()
            , wkDat->exploreMapHolder.getMoreCellCountToExtend()
            );
        shouldReinitMap_ = false;
        return true;
    }

    bool ServerExploreMapUpdateWorker::checkRecvResolution_(float recvResolution, const ServerWorkData_Ptr& wkDat)
    {
        const auto fResolution = wkDat->exploreMapHolder.resolution();
        if (rpos::system::types::fequal(fResolution, recvResolution))
            return true;

        RCLCPP_ERROR(rosNodeHandle()->get_logger() ,"local resolution: %f, received resolution: %f, request reinit.", fResolution, recvResolution);
        requestReinitMap_();
        return false;
    }

    bool ServerExploreMapUpdateWorker::updateMapInCellIdxRect_(slamware_platform_t& pltfm
            , const rpos::core::RectangleI& reqIdxRect
            , const ServerWorkData_Ptr& wkDat
            )
    {
        const auto reqArea = wkDat->exploreMapHolder.calcAreaByCellIdxRect(reqIdxRect);
        auto hMap = getMapByPltfm_(pltfm, reqArea);
        if (!hMap)
            return false;

        const auto& mapResolution = hMap.getMapResolution();
        if (!checkRecvResolution_(mapResolution.x(), wkDat))
            return false;

        wkDat->exploreMapHolder.setMapData(hMap);
        return true;
    }

    bool ServerExploreMapUpdateWorker::syncWholeMap_(const ServerParams& srvParams
        , slamware_platform_t& pltfm
        , const ServerWorkData_Ptr& wkDat
        )
    {
        const float fOnceMaxWH = std::max<float>(16.0f, srvParams.map_sync_once_get_max_wh);
        const auto fResolution = wkDat->exploreMapHolder.resolution();
        const int iOnceMaxCellCntWH = static_cast<int>(std::round(fOnceMaxWH / fResolution));
        BOOST_ASSERT(0 < iOnceMaxCellCntWH);

        const auto knownArea = pltfm.getKnownArea(rpos::features::location_provider::MapTypeBitmap8Bit, rpos::features::location_provider::EXPLORERMAP);
        const auto knownCellIdxRect = wkDat->exploreMapHolder.calcRoundedCellIdxRect(knownArea);
        RCLCPP_INFO(rosNodeHandle()->get_logger() ,"known area: ((%f, %f), (%f, %f)), cell rect: ((%d, %d), (%d, %d)), iOnceMaxCellCntWH: %d."
            , knownArea.x(), knownArea.y(), knownArea.width(), knownArea.height()
            , knownCellIdxRect.x(), knownCellIdxRect.y(), knownCellIdxRect.width(), knownCellIdxRect.height()
            , iOnceMaxCellCntWH
            );
        if (ServerMapHolder::sfIsCellIdxRectEmpty(knownCellIdxRect))
        {
            RCLCPP_ERROR(rosNodeHandle()->get_logger() ,"sync map, knownCellIdxRect is empty.");
            return false;
        }

        wkDat->exploreMapHolder.clear();
        wkDat->exploreMapHolder.reserveByCellIdxRect(knownCellIdxRect);

        const int cellIdxXEnd = knownCellIdxRect.x() + knownCellIdxRect.width();
        const int cellIdxYEnd = knownCellIdxRect.y() + knownCellIdxRect.height();
        int cellIdxY = knownCellIdxRect.y();
        while (cellIdxY < cellIdxYEnd)
        {
            const int tmpRemY = cellIdxYEnd - cellIdxY;
            const int reqSizeY = std::min<int>(tmpRemY, iOnceMaxCellCntWH);
            //
            int cellIdxX = knownCellIdxRect.x();
            while (cellIdxX < cellIdxXEnd)
            {
                const int tmpRemX = cellIdxXEnd - cellIdxX;
                const int reqSizeX = std::min<int>(tmpRemX, iOnceMaxCellCntWH);
                //
                const auto reqIdxRect = rpos::core::RectangleI(cellIdxX, cellIdxY, reqSizeX, reqSizeY);
                if (!updateMapInCellIdxRect_(pltfm, reqIdxRect, wkDat))
                {
                    return false;
                }
                //
                cellIdxX += reqSizeX;
            }
            //
            cellIdxY += reqSizeY;
        }
        return true;
    }

    bool ServerExploreMapUpdateWorker::updateMapNearRobot_(const ServerParams& srvParams
            , slamware_platform_t& pltfm
            , const ServerWorkData_Ptr& wkDat
            )
    {
        const float fHalfWH = std::max<float>(2.0f, srvParams.map_update_near_robot_half_wh);
        const float fWH = fHalfWH + fHalfWH;
        const auto nearRobotArea = rpos::core::RectangleF(static_cast<float>(wkDat->robotPose.x() - fHalfWH)
            , static_cast<float>(wkDat->robotPose.y() - fHalfWH)
            , fWH
            , fWH
            );
        const auto nearRobotIdxRect = wkDat->exploreMapHolder.calcMinBoundingCellIdxRect(nearRobotArea);

        const auto knownArea = pltfm.getKnownArea(rpos::features::location_provider::MapTypeBitmap8Bit, rpos::features::location_provider::EXPLORERMAP);
        const auto knownCellIdxRect = wkDat->exploreMapHolder.calcRoundedCellIdxRect(knownArea);
    #if 0
        ROS_INFO("known area: ((%f, %f), (%f, %f)), cell rect: ((%d, %d), (%d, %d))."
            , knownArea.x(), knownArea.y(), knownArea.width(), knownArea.height()
            , knownCellIdxRect.x(), knownCellIdxRect.y(), knownCellIdxRect.width(), knownCellIdxRect.height()
            );
    #endif
        if (ServerMapHolder::sfIsCellIdxRectEmpty(knownCellIdxRect))
        {
            RCLCPP_ERROR(rosNodeHandle()->get_logger(),"update map, knownCellIdxRect is empty, request sync map.");
            rosSdkServer()->requestSyncMap();
            return false;
        }

        const auto reqIdxRect = ServerMapHolder::sfIntersectionOfCellIdxRect(nearRobotIdxRect, knownCellIdxRect);
        if (ServerMapHolder::sfIsCellIdxRectEmpty(reqIdxRect))
        {
            RCLCPP_WARN(rosNodeHandle()->get_logger(),"knownCellIdxRect: ((%d, %d), (%d, %d)), nearRobotIdxRect: ((%d, %d), (%d, %d)), intersection is empty."
                , knownCellIdxRect.x(), knownCellIdxRect.y(), knownCellIdxRect.width(), knownCellIdxRect.height()
                , nearRobotIdxRect.x(), nearRobotIdxRect.y(), nearRobotIdxRect.width(), nearRobotIdxRect.height()
                );
            return false;
        }
        return updateMapInCellIdxRect_(pltfm, reqIdxRect, wkDat);
    }

    //////////////////////////////////////////////////////////////////////////

    ServerExploreMapPublishWorker::ServerExploreMapPublishWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
    {
        const auto& srvParams = serverParams();
        rclcpp::Node::SharedPtr nhRos = rosNodeHandle();

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
        qos.history(rclcpp::HistoryPolicy::KeepLast);
        pubMapDat_ = nhRos->create_publisher<nav_msgs::msg::OccupancyGrid>(srvParams.map_topic, qos);
        pubMapInfo_ = nhRos->create_publisher<nav_msgs::msg::MapMetaData>(srvParams.map_info_topic, qos);
    }

    ServerExploreMapPublishWorker::~ServerExploreMapPublishWorker()
    {
        //
    }

    void ServerExploreMapPublishWorker::doPerform(slamware_platform_t& /*pltfm*/)
    {
        const auto& srvParams = serverParams();
        auto wkDat = workData();

        if (wkDat->exploreMapHolder.isMapDataEmpty())
        {
            //ROS_WARN("current explore map data is empty.");
            return;
        }

        nav_msgs::srv::GetMap::Response msgMap;
        wkDat->exploreMapHolder.fillRosMapMsg(msgMap);

        // Set the header information on the map
        msgMap.map.header.stamp = rosNodeHandle()->now();
        msgMap.map.header.frame_id = srvParams.map_frame;

        pubMapDat_->publish(msgMap.map);
        pubMapInfo_->publish(msgMap.map.info);
    }

    //////////////////////////////////////////////////////////////////////////

    ServerLaserScanWorker::ServerLaserScanWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
        , compensatedAngleCnt_(360u)
        , absAngleIncrement_(C_FLT_2PI / compensatedAngleCnt_)
    {
        const auto& srvParams = serverParams();
        rclcpp::Node::SharedPtr nhRos = rosNodeHandle();
        pubLaserScan_ = nhRos->create_publisher<sensor_msgs::msg::LaserScan>(srvParams.scan_topic, 10);
    }

    ServerLaserScanWorker::~ServerLaserScanWorker()
    {
        //
    }

    static std::string sanitize_frame_id(const std::string& frame_id)
    {
        if (frame_id.size() > 0 && frame_id[0] == '/')
        {
            return frame_id.substr(1);
        }
        return frame_id;
    }

    void ServerLaserScanWorker::doPerform(slamware_platform_t& pltfm)
    {
        const auto& srvParams = serverParams();
        auto tfBrdcst = tfBroadcaster();
        auto wkDat = workData();


        rclcpp::Time startScanTime = rosNodeHandle()->now();
        rpos::features::system_resource::LaserScan tLs = pltfm.getLaserScan();
        rclcpp::Time endScanTime = rosNodeHandle()->now();
        double dblScanDur = (endScanTime - startScanTime).seconds();

        const auto& points = tLs.getLaserPoints();
        if (points.size() < 2)
        {
            RCLCPP_ERROR(rosNodeHandle()->get_logger() , "laser points count: %u, too small, skip publish.", (unsigned int)points.size());
            return;
        }

        const auto laserPose = (tLs.getHasPose() ? tLs.getLaserPointsPose() : wkDat->robotPose);
        //ROS_INFO("has laser pose: %s, robotPose: ((%f, %f), (%f)), laserPose: ((%f, %f), (%f))."
        //    , (tLs.getHasPose() ? "true" : "false")
        //    , wkDat->robotPose.x(), wkDat->robotPose.y(), wkDat->robotPose.yaw()
        //    , laserPose.x(), laserPose.y(), laserPose.yaw()
        //    );

        std::string sanitized_laser_frame = sanitize_frame_id(srvParams.laser_frame);
        sensor_msgs::msg::LaserScan msgScan;
        msgScan.header.stamp = startScanTime;
        msgScan.header.frame_id = sanitized_laser_frame;
        fillRangeMinMaxInMsg_(points, msgScan);

        if (srvParams.angle_compensate)
        {
            compensateAndfillRangesInMsg_(points, msgScan);
        }
        else
        {
            //msgScan.intensities.resize(points.size());
            msgScan.ranges.resize(points.size());

            for (size_t i = 0; i < points.size(); ++i)
            {
                if (!points[i].valid())
                {
                    msgScan.ranges[i] = std::numeric_limits<float>::infinity();
                }
                else
                {
                    msgScan.ranges[i] = points[i].distance();
                }
            }
            msgScan.angle_min =  points.front().angle();
            msgScan.angle_max =  points.back().angle();
            msgScan.angle_increment = (msgScan.angle_max - msgScan.angle_min) / (double)(msgScan.ranges.size() - 1);
        }
        BOOST_ASSERT(2 <= msgScan.ranges.size());
        msgScan.scan_time = dblScanDur;
        msgScan.time_increment = dblScanDur / (double)(msgScan.ranges.size() - 1);

        {
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped.header.stamp = rosNodeHandle()->now();
            transformStamped.header.frame_id = srvParams.map_frame;
            transformStamped.child_frame_id = sanitized_laser_frame;
            transformStamped.transform.translation.x = laserPose.x();
            transformStamped.transform.translation.y = laserPose.y();
            transformStamped.transform.translation.z = 0.0;

            tf2::Matrix3x3 obs_mat;
            obs_mat.setEulerYPR(laserPose.yaw(),0.0,0.0);

            tf2::Quaternion q_tf;
            obs_mat.getRotation(q_tf);
            transformStamped.transform.rotation.x = q_tf.getX();
            transformStamped.transform.rotation.y = q_tf.getY();
            transformStamped.transform.rotation.z = q_tf.getZ();
            transformStamped.transform.rotation.w = q_tf.getW();
            tfBrdcst->sendTransform(transformStamped);

//            tf::Transform laserTrans;
//            laserTrans.setOrigin(tf::Vector3(laserPose.x(), laserPose.y(), 0.0));
//            tf::Quaternion qLaserTrans = tf::createQuaternionFromYaw(laserPose.yaw());
//            laserTrans.setRotation(qLaserTrans);
//            tfBrdcst.sendTransform(tf::StampedTransform(laserTrans, startScanTime, srvParams.map_frame, srvParams.laser_frame));
        }
        pubLaserScan_->publish(msgScan);
    }

    void ServerLaserScanWorker::fillRangeMinMaxInMsg_(const std::vector<rpos::core::LaserPoint> & laserPoints
            , sensor_msgs::msg::LaserScan& msgScan
            ) const
    {
        msgScan.range_min = std::numeric_limits<float>::infinity();
        msgScan.range_max = 0.0f;
        for (auto cit = laserPoints.cbegin(), citEnd = laserPoints.cend(); citEnd != cit; ++cit)
        {
            if (cit->valid())
            {
                const float tmpDist = cit->distance();
                
                if (tmpDist < msgScan.range_min)
                {
                    msgScan.range_min = std::max<float>(0.0f, tmpDist);
                }

                if (msgScan.range_max < tmpDist)
                {
                    msgScan.range_max = tmpDist;
                }
            }
        }
    }

    float ServerLaserScanWorker::calcAngleInNegativePiToPi_(float angle) const
    {
        float fRes = std::fmod(angle + C_FLT_PI, C_FLT_2PI);
        if (fRes < 0.0f)
            fRes += C_FLT_2PI;
        fRes -= C_FLT_PI;

        if (fRes < -C_FLT_PI)
            fRes = -C_FLT_PI;
        else if (C_FLT_PI <= fRes)
            fRes = -C_FLT_PI;
        return fRes;
    }

    std::uint32_t ServerLaserScanWorker::calcCompensateDestIndexBySrcAngle_(float srcAngle
        , bool isAnglesReverse
        ) const
    {
        BOOST_ASSERT(-C_FLT_PI <= srcAngle && srcAngle < C_FLT_PI);
        
        float fDiff = (isAnglesReverse ? (C_FLT_PI - srcAngle) : (srcAngle + C_FLT_PI));
        fDiff = std::max<float>(0.0f, fDiff);
        fDiff = std::min<float>(fDiff, C_FLT_2PI);

        std::uint32_t destIdx = static_cast<std::uint32_t>(std::round(fDiff / absAngleIncrement_));
        if (compensatedAngleCnt_ <= destIdx)
            destIdx = 0;
        return destIdx;
    }

    bool ServerLaserScanWorker::isSrcAngleMoreCloseThanOldSrcAngle_(float srcAngle, float destAngle, float oldSrcAngle) const
    {
        BOOST_ASSERT(-C_FLT_PI <= srcAngle && srcAngle < C_FLT_PI);
        BOOST_ASSERT(-C_FLT_PI <= destAngle && destAngle < C_FLT_PI);
        BOOST_ASSERT(-C_FLT_PI <= oldSrcAngle && oldSrcAngle < C_FLT_PI);

        float newDiff = std::abs(destAngle - srcAngle);
        if (C_FLT_2PI <= newDiff)
            newDiff = 0.0f;
        else if (C_FLT_PI < newDiff)
            newDiff = C_FLT_2PI - newDiff;

        float oldDiff = std::abs(destAngle - oldSrcAngle);
        if (C_FLT_2PI <= oldDiff)
            oldDiff = 0.0f;
        else if (C_FLT_PI < oldDiff)
            oldDiff = C_FLT_2PI - oldDiff;

        return (newDiff < oldDiff);
    }

    void ServerLaserScanWorker::compensateAndfillRangesInMsg_(const std::vector<rpos::core::LaserPoint> & laserPoints
            , sensor_msgs::msg::LaserScan& msgScan
            ) const
    {
        BOOST_ASSERT(2 <= laserPoints.size());

        msgScan.ranges.clear();
        msgScan.ranges.resize(compensatedAngleCnt_, std::numeric_limits<float>::infinity());

        const bool isAnglesReverse = (laserPoints.back().angle() < laserPoints.front().angle());
        if (!isAnglesReverse)
        {
            msgScan.angle_min = -C_FLT_PI;
            msgScan.angle_max = C_FLT_PI - absAngleIncrement_;
            msgScan.angle_increment = absAngleIncrement_;
        }
        else
        {
            msgScan.angle_min = C_FLT_PI;
            msgScan.angle_max = (-C_FLT_PI + absAngleIncrement_); 
            msgScan.angle_increment = -absAngleIncrement_;
        }

        std::vector<float> tmpSrcAngles(compensatedAngleCnt_);
        for (auto cit = laserPoints.cbegin(), citEnd = laserPoints.cend(); citEnd != cit; ++cit)
        {
            if (cit->valid())
            {
                const float srcAngle = calcAngleInNegativePiToPi_(cit->angle());
                const std::uint32_t destIdx = calcCompensateDestIndexBySrcAngle_(srcAngle, isAnglesReverse);
                BOOST_ASSERT(destIdx < compensatedAngleCnt_);
                const float destAngle = calcAngleInNegativePiToPi_(msgScan.angle_min + msgScan.angle_increment * destIdx);

                const bool shouldWrite = (std::isinf(msgScan.ranges[destIdx])
                    || isSrcAngleMoreCloseThanOldSrcAngle_(srcAngle, destAngle, tmpSrcAngles[destIdx])
                    );
                if (shouldWrite)
                {
                    msgScan.ranges[destIdx] = cit->distance();
                    tmpSrcAngles[destIdx] = srcAngle;
                }
            }
        }

        //ROS_INFO("compensatedAngleCnt_: %u, isAnglesReverse: %s.", compensatedAngleCnt_, (isAnglesReverse ? "true": "false"));
    }

    //////////////////////////////////////////////////////////////////////////

    ServerBasicSensorsInfoWorker::ServerBasicSensorsInfoWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
    {
        const auto& srvParams = serverParams();
        rclcpp::Node::SharedPtr nhRos = rosNodeHandle();
        pubSensorsInfo_ = nhRos->create_publisher<BasicSensorInfoArray>(srvParams.basic_sensors_info_topic, 1);
    }

    ServerBasicSensorsInfoWorker::~ServerBasicSensorsInfoWorker()
    {
        //
    }

    void ServerBasicSensorsInfoWorker::doPerform(slamware_platform_t& pltfm)
    {
        auto wkDat = mutableWorkData();
        
        {
            sensors_info_map_t sensorsInfo;
            if (!getSensorsInfo_(pltfm, sensorsInfo))
            {
                RCLCPP_ERROR(rosNodeHandle()->get_logger() ,"failed to get sensors info from slamware platform.");
                return;
            }
            if (isSensorsInfoAsTheSame_(wkDat->sensorsInfo, sensorsInfo))
                return;

            wkDat->sensorsInfo.swap(sensorsInfo);
            sltcToRosMsg(wkDat->sensorsInfo, wkDat->rosBasicSensorsInfo);
        }

        BasicSensorInfoArray msgSensorsInfo;
        const size_t sensorsCnt = wkDat->rosBasicSensorsInfo.size();
        msgSensorsInfo.sensors_info.resize(sensorsCnt);
        size_t t = 0;
        for (auto cit = wkDat->rosBasicSensorsInfo.cbegin(), citEnd = wkDat->rosBasicSensorsInfo.cend(); citEnd != cit; ++cit, ++t)
        {
            const auto& rcRosInfo = cit->second;
            BOOST_ASSERT(rcRosInfo.id == cit->first);
            msgSensorsInfo.sensors_info[t] = rcRosInfo;
        }
        pubSensorsInfo_->publish(msgSensorsInfo);
        RCLCPP_INFO(rosNodeHandle()->get_logger() ,"new sensors info published, sensorsCnt: %u.", (unsigned int)sensorsCnt);
    }

    bool ServerBasicSensorsInfoWorker::getSensorsInfo_(slamware_platform_t& pltfm, sensors_info_map_t& sensorsInfo) const
    {
        std::vector<rpos::features::impact_sensor::ImpactSensorInfo> vSensorsInfo;
        if (!pltfm.getSensors(vSensorsInfo))
            return false;

        sensorsInfo.clear();
        for (auto cit = vSensorsInfo.cbegin(), citEnd = vSensorsInfo.cend(); citEnd != cit; ++cit)
        {
            sensorsInfo.insert(sensors_info_map_t::value_type(cit->id, *cit));
        }
        return true;
    }

    bool ServerBasicSensorsInfoWorker::isSensorsInfoAsTheSame_(const sensors_info_map_t& sensorsInfoA, const sensors_info_map_t& sensorsInfoB) const
    {
        const size_t sensorsCnt = sensorsInfoA.size();
        if (sensorsInfoB.size() != sensorsCnt)
        {
            RCLCPP_INFO(rosNodeHandle()->get_logger() ,"sensorsCnt: %u --> %u.", (unsigned int)sensorsCnt, (unsigned int)sensorsInfoB.size());
            return false;
        }

        for (auto citA = sensorsInfoA.cbegin(), citAEnd = sensorsInfoA.cend(); citAEnd != citA; ++citA)
        {
            const auto& rcInfoA = citA->second;
            BOOST_ASSERT(rcInfoA.id == citA->first);

            auto citB = sensorsInfoB.find(rcInfoA.id);
            if (sensorsInfoB.cend() == citB)
            {
                RCLCPP_INFO(rosNodeHandle()->get_logger() ,"sensor id %d has gone away.", rcInfoA.id);
                return false;
            }

            const auto& rcInfoB = citB->second;
            BOOST_ASSERT(rcInfoB.id == citB->first);

            if (rcInfoA.coreSensorType != rcInfoB.coreSensorType)
            {
                RCLCPP_INFO(rosNodeHandle()->get_logger() ,"sensor id %d, core type: %d --> %d.", rcInfoA.id, (int)rcInfoA.coreSensorType, (int)rcInfoB.coreSensorType);
                return false;
            }
            if (rcInfoA.type != rcInfoB.type)
            {
                RCLCPP_INFO(rosNodeHandle()->get_logger() ,"sensor id %d, impact type: %d --> %d.", rcInfoA.id, (int)rcInfoA.type, (int)rcInfoB.type);
                return false;
            }
            if (!(rcInfoA.pose == rcInfoB.pose))
            {
                RCLCPP_INFO(rosNodeHandle()->get_logger() ,"sensor id %d, pose changed.", rcInfoA.id);
                return false;
            }
        }
        return true;
    }

    //////////////////////////////////////////////////////////////////////////

    ServerBasicSensorsValuesWorker::ServerBasicSensorsValuesWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
    {
        const auto& srvParams = serverParams();
        rclcpp::Node::SharedPtr nhRos = rosNodeHandle();
        pubSensorsValues_ = nhRos->create_publisher<BasicSensorValueDataArray>(srvParams.basic_sensors_values_topic, 5);
    }

    ServerBasicSensorsValuesWorker::~ServerBasicSensorsValuesWorker()
    {
        //
    }

    void ServerBasicSensorsValuesWorker::doPerform(slamware_platform_t& pltfm)
    {
        auto wkDat = mutableWorkData();
        
        sensors_values_map_t sensorsValues;
        if (!getSensorsValues_(pltfm, sensorsValues))
        {
            RCLCPP_ERROR(rosNodeHandle()->get_logger() ,"failed to get sensors values from slamware platform.");
            return;
        }

        BasicSensorValueDataArray msgSensorsValues;
        const size_t sensorsCnt = wkDat->rosBasicSensorsInfo.size();
        msgSensorsValues.values_data.resize(sensorsCnt);
        size_t t = 0;
        for (auto citInfo = wkDat->rosBasicSensorsInfo.cbegin(), citInfoEnd = wkDat->rosBasicSensorsInfo.cend(); citInfoEnd != citInfo; ++citInfo, ++t)
        {
            const auto& rcInfo = citInfo->second;
            BOOST_ASSERT(rcInfo.id == citInfo->first);

            auto& destValDat = msgSensorsValues.values_data[t];
            destValDat.info = rcInfo;
            auto& destVal = destValDat.value;
            
            sensors_values_map_t::const_iterator citSrcVal = sensorsValues.find(rcInfo.id);
            if (sensorsValues.cend() == citSrcVal)
                continue;

            const auto& rcSrcVal = citSrcVal->second;
            destVal.value = rcSrcVal.value;
            destVal.is_in_impact = isSensorValueImpact_(rcInfo, rcSrcVal);
        }
        pubSensorsValues_->publish(msgSensorsValues);
    }

    bool ServerBasicSensorsValuesWorker::getSensorsValues_(slamware_platform_t& pltfm, sensors_values_map_t& sensorsValues) const
    {
        sensorsValues.clear();
        return pltfm.getSensorValues(sensorsValues);
    }

    bool ServerBasicSensorsValuesWorker::isSensorValueImpact_(const BasicSensorInfo& basicInfo, const sensor_value_t& sensorVal) const
    {
        switch (basicInfo.impact_type.type)
        {
        case ImpactType::DIGITAL:
            return ServerWorkData::sfIsDigitalSensorValueImpact(sensorVal.value);
        case ImpactType::ANALOG:
            {
                switch (basicInfo.sensor_type.type)
                {
                case SensorType::SONAR:
                    return true;
                default:
                    break;
                }
            }
            return false;
        default:
            break;
        }
        return false;
    }

    //////////////////////////////////////////////////////////////////////////

    ServerPlanPathWorker::ServerPlanPathWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
    {
        const auto& srvParams = serverParams();
        rclcpp::Node::SharedPtr nhRos = rosNodeHandle();
        pubPlanPath_ = nhRos->create_publisher<nav_msgs::msg::Path>(srvParams.path_topic, 10);
    }

    ServerPlanPathWorker::~ServerPlanPathWorker()
    {
        //
    }

    void ServerPlanPathWorker::doPerform(slamware_platform_t& pltfm)
    {
        const auto& srvParams = serverParams();
        
        nav_msgs::msg::Path msgPath;
        msgPath.poses.resize(0);
        msgPath.header.frame_id = srvParams.map_frame;

        rpos::actions::MoveAction actMove = pltfm.getCurrentAction();
        if (!actMove)
        {
            msgPath.header.stamp = rosNodeHandle()->now();
            pubPlanPath_->publish(msgPath);
            return;
        }
        rpos::features::motion_planner::Path remPath = actMove.getRemainingPath();
        if (!remPath)
        {
            msgPath.header.stamp = rosNodeHandle()->now();
            pubPlanPath_->publish(msgPath);
            return;
        }

        const auto& remPathPoints = remPath.getPoints();
        msgPath.poses.resize(remPathPoints.size());
        msgPath.header.stamp = rosNodeHandle()->now();
        for (size_t i = 0; i < remPathPoints.size(); ++i)
        {
            geometry_msgs::msg::PoseStamped tPoseStamp;
            tPoseStamp.header.frame_id = srvParams.map_frame;
            tPoseStamp.header.stamp = rosNodeHandle()->now();
            sltcToRosMsg(remPathPoints[i], tPoseStamp.pose.position);
            tPoseStamp.pose.orientation.x = 0;
            tPoseStamp.pose.orientation.y = 0;
            tPoseStamp.pose.orientation.z = 0;
            tPoseStamp.pose.orientation.w = 1;
            msgPath.poses[i] = tPoseStamp;
        }
        pubPlanPath_->publish(msgPath);
    }

    //////////////////////////////////////////////////////////////////////////

    ServerRobotBasicStateWorker::ServerRobotBasicStateWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
    {
        rclcpp::Node::SharedPtr nhRos = rosNodeHandle();
        pubRobotBasicState_ = nhRos->create_publisher<RobotBasicState>("robot_basic_state", 1);
    }

    ServerRobotBasicStateWorker::~ServerRobotBasicStateWorker()
    {
        //
    }

    void ServerRobotBasicStateWorker::doPerform(slamware_platform_t& pltfm)
    {
        RobotBasicState msgRobotBasicState;
        
        msgRobotBasicState.is_map_building_enabled = pltfm.getMapUpdate(rpos::features::location_provider::EXPLORERMAP);
        msgRobotBasicState.is_localization_enabled = pltfm.getMapLocalization();
        
        msgRobotBasicState.localization_quality = pltfm.getLocalizationQuality();

        msgRobotBasicState.board_temperature = pltfm.getBoardTemperature();

        const auto pwrStatus = pltfm.getPowerStatus();
        msgRobotBasicState.battery_percentage = pwrStatus.batteryPercentage;
        msgRobotBasicState.is_dc_in = pwrStatus.isDCConnected;
        msgRobotBasicState.is_charging = pwrStatus.isCharging;

        pubRobotBasicState_->publish(msgRobotBasicState);
    }

    //////////////////////////////////////////////////////////////////////////

    ServerArtifactLinesWorker::ServerArtifactLinesWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        , const params_t& rcParams
        )
        : super_t(pRosSdkServer, wkName, triggerInterval)
        , params_(rcParams)
        , sltcUsage_(rpos::features::artifact_provider::ArtifactUsageVirtualWall)
        , isFeatureSupported_(false)
    {
        rosMsgToSltc(params_.usage, sltcUsage_);
    }

    ServerArtifactLinesWorker::~ServerArtifactLinesWorker()
    {
        //
    }

    void ServerArtifactLinesWorker::resetOnWorkLoopBegin()
    {
//        pubArtifactLines_= rclcpp::Publisher();
        pubArtifactLines_.reset();
        isFeatureSupported_ = false;

        this->super_t::resetOnWorkLoopBegin();
    }

    bool ServerArtifactLinesWorker::reinitWorkLoop(slamware_platform_t& pltfm)
    {
        if (!this->super_t::reinitWorkLoop(pltfm))
            return false;
        isWorkLoopInitOk_ = false;

        std::vector<rpos::core::Line> vLines;
        try
        {
            vLines = pltfm.getLines(sltcUsage_);
            isFeatureSupported_ = true;
        }
        catch (const rpos::robot_platforms::UnsupportedCommandException& excp)
        {
            isFeatureSupported_ = false;
            RCLCPP_WARN(rosNodeHandle()->get_logger() ,"worker: %s, reinitWorkLoop(), usage: %d, exception: %s.", getWorkerName().c_str(), (int)params_.usage.usage, excp.what());
        }

        if (isFeatureSupported_)
        {
            Line2DFlt32Array msgLines;
            sltcToRosMsg(vLines, msgLines.lines);

            rclcpp::Node::SharedPtr nhRos = rosNodeHandle();
            pubArtifactLines_ = nhRos->create_publisher<Line2DFlt32Array>(params_.topic, params_.queueSize);
            pubArtifactLines_->publish(msgLines);
        }

        isWorkLoopInitOk_ = true;
        return isWorkLoopInitOk_;
    }

    void ServerArtifactLinesWorker::doPerform(slamware_platform_t& pltfm)
    {
        if (!isFeatureSupported_)
            return;

        const auto vLines = pltfm.getLines(sltcUsage_);
        
        Line2DFlt32Array msgLines;
        sltcToRosMsg(vLines, msgLines.lines);

        pubArtifactLines_->publish(msgLines);
    }

    //////////////////////////////////////////////////////////////////////////
    
}
