
#pragma once

#include "server_worker_base.h"

#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace slamware_ros_sdk {

    //////////////////////////////////////////////////////////////////////////

    class ServerRobotDeviceInfoWorker: public ServerWorkerBase
    {
    public:
        typedef ServerWorkerBase          super_t;

    public:
        ServerRobotDeviceInfoWorker(SlamwareRosSdkServer* pRosSdkServer
            , const std::string& wkName
            , const boost::chrono::milliseconds& triggerInterval
            );
        virtual ~ServerRobotDeviceInfoWorker();

        virtual bool reinitWorkLoop(slamware_platform_t& pltfm);

    protected:
        virtual void doPerform(slamware_platform_t& pltfm);

    private:
        rclcpp::Publisher<RobotDeviceInfo>::SharedPtr  pubRobotDeviceInfo_;
    };

    //////////////////////////////////////////////////////////////////////////

    class ServerRobotPoseWorker: public ServerWorkerBase
    {
    public:
        ServerRobotPoseWorker(SlamwareRosSdkServer* pRosSdkServer
            , const std::string& wkName
            , const boost::chrono::milliseconds& triggerInterval
            );
        virtual ~ServerRobotPoseWorker();

    protected:
        virtual void doPerform(slamware_platform_t& pltfm);

    private:
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubRobotPose_;
    };

    //////////////////////////////////////////////////////////////////////////

    class ServerExploreMapUpdateWorker: public ServerWorkerBase
    {
    public:
        typedef ServerWorkerBase          super_t;

    public:
        ServerExploreMapUpdateWorker(SlamwareRosSdkServer* pRosSdkServer
            , const std::string& wkName
            , const boost::chrono::milliseconds& triggerInterval
            );
        virtual ~ServerExploreMapUpdateWorker();

        virtual bool reinitWorkLoop(slamware_platform_t& pltfm);

    protected:
        virtual void doPerform(slamware_platform_t& pltfm);

    private:
        rpos::features::location_provider::Map getMapByPltfm_(slamware_platform_t& pltfm, const rpos::core::RectangleF& area) const;

        void requestReinitMap_();
        bool checkToReinitMap_(slamware_platform_t& pltfm, const ServerWorkData_Ptr& wkDat);

        bool checkRecvResolution_(float recvResolution, const ServerWorkData_Ptr& wkDat);

        bool updateMapInCellIdxRect_(slamware_platform_t& pltfm
            , const rpos::core::RectangleI& reqIdxRect
            , const ServerWorkData_Ptr& wkDat
            );

        bool syncWholeMap_(const ServerParams& srvParams
            , slamware_platform_t& pltfm
            , const ServerWorkData_Ptr& wkDat
            );

        bool updateMapNearRobot_(const ServerParams& srvParams
            , slamware_platform_t& pltfm
            , const ServerWorkData_Ptr& wkDat
            );

    private:
        bool shouldReinitMap_;
    };

    //////////////////////////////////////////////////////////////////////////

    class ServerExploreMapPublishWorker: public ServerWorkerBase
    {
    public:
        ServerExploreMapPublishWorker(SlamwareRosSdkServer* pRosSdkServer
            , const std::string& wkName
            , const boost::chrono::milliseconds& triggerInterval
            );
        virtual ~ServerExploreMapPublishWorker();

    protected:
        virtual void doPerform(slamware_platform_t& pltfm);

    private:
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubMapDat_;
        rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr pubMapInfo_;

//        ros::Publisher pubMapDat_;
//        ros::Publisher pubMapInfo_;
    };

    //////////////////////////////////////////////////////////////////////////

    class ServerLaserScanWorker: public ServerWorkerBase
    {
    public:
        ServerLaserScanWorker(SlamwareRosSdkServer* pRosSdkServer
            , const std::string& wkName
            , const boost::chrono::milliseconds& triggerInterval
            );
        virtual ~ServerLaserScanWorker();

    protected:
        virtual void doPerform(slamware_platform_t& pltfm);

    private:
        void fillRangeMinMaxInMsg_(const std::vector<rpos::core::LaserPoint> & laserPoints
            , sensor_msgs::msg::LaserScan& msgScan
            ) const;

        float calcAngleInNegativePiToPi_(float angle) const;
        
        std::uint32_t calcCompensateDestIndexBySrcAngle_(float srcAngle
            , bool isAnglesReverse
            ) const;
        bool isSrcAngleMoreCloseThanOldSrcAngle_(float srcAngle, float destAngle, float oldSrcAngle) const;
        void compensateAndfillRangesInMsg_(const std::vector<rpos::core::LaserPoint> & laserPoints
            , sensor_msgs::msg::LaserScan& msgScan
            ) const;

    private:
        std::uint32_t compensatedAngleCnt_;
        float absAngleIncrement_;

        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pubLaserScan_;
    };

    //////////////////////////////////////////////////////////////////////////

    class ServerBasicSensorsInfoWorker: public ServerWorkerBase
    {
    public:
        ServerBasicSensorsInfoWorker(SlamwareRosSdkServer* pRosSdkServer
            , const std::string& wkName
            , const boost::chrono::milliseconds& triggerInterval
            );
        virtual ~ServerBasicSensorsInfoWorker();

    protected:
        virtual void doPerform(slamware_platform_t& pltfm);

    private:
        typedef ServerWorkData::sensors_info_map_t          sensors_info_map_t;

        bool getSensorsInfo_(slamware_platform_t& pltfm, sensors_info_map_t& sensorsInfo) const;
        bool isSensorsInfoAsTheSame_(const sensors_info_map_t& sensorsInfoA, const sensors_info_map_t& sensorsInfoB) const;

    private:
        rclcpp::Publisher<BasicSensorInfoArray>::SharedPtr pubSensorsInfo_;
    };

    //////////////////////////////////////////////////////////////////////////

    class ServerBasicSensorsValuesWorker: public ServerWorkerBase
    {
    public:
        ServerBasicSensorsValuesWorker(SlamwareRosSdkServer* pRosSdkServer
            , const std::string& wkName
            , const boost::chrono::milliseconds& triggerInterval
            );
        virtual ~ServerBasicSensorsValuesWorker();

    protected:
        virtual void doPerform(slamware_platform_t& pltfm);

    private:
        typedef ServerWorkData::sensor_value_t                sensor_value_t;
        typedef ServerWorkData::sensors_values_map_t          sensors_values_map_t;

        bool getSensorsValues_(slamware_platform_t& pltfm, sensors_values_map_t& sensorsValues) const;

        bool isSensorValueImpact_(const BasicSensorInfo& basicInfo, const sensor_value_t& sensorVal) const;

    private:
        rclcpp::Publisher<BasicSensorValueDataArray>::SharedPtr pubSensorsValues_;
    };

    //////////////////////////////////////////////////////////////////////////

    class ServerPlanPathWorker: public ServerWorkerBase
    {
    public:
        ServerPlanPathWorker(SlamwareRosSdkServer* pRosSdkServer
            , const std::string& wkName
            , const boost::chrono::milliseconds& triggerInterval
            );
        virtual ~ServerPlanPathWorker();

    protected:
        virtual void doPerform(slamware_platform_t& pltfm);

    private:
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPlanPath_;
    };

    //////////////////////////////////////////////////////////////////////////

    class ServerRobotBasicStateWorker: public ServerWorkerBase
    {
    public:
        ServerRobotBasicStateWorker(SlamwareRosSdkServer* pRosSdkServer
            , const std::string& wkName
            , const boost::chrono::milliseconds& triggerInterval
            );
        virtual ~ServerRobotBasicStateWorker();

    protected:
        virtual void doPerform(slamware_platform_t& pltfm);

    private:
        rclcpp::Publisher<RobotBasicState>::SharedPtr pubRobotBasicState_;
    };

    //////////////////////////////////////////////////////////////////////////

    class ServerArtifactLinesWorker: public ServerWorkerBase
    {
    public:
        typedef ServerWorkerBase          super_t;

        struct params_t
        {
            ArtifactUsage usage;
            std::string topic;
            std::uint32_t queueSize;
            bool latch;

            params_t()
                : queueSize(1u)
                , latch(true)
            {
                usage.usage = ArtifactUsage::UNKNOWN;
            }
        };

    public:
        ServerArtifactLinesWorker(SlamwareRosSdkServer* pRosSdkServer
            , const std::string& wkName
            , const boost::chrono::milliseconds& triggerInterval
            , const params_t& rcParams
            );
        virtual ~ServerArtifactLinesWorker();

        virtual void resetOnWorkLoopBegin();
        virtual bool reinitWorkLoop(slamware_platform_t& pltfm);

    protected:
        virtual void doPerform(slamware_platform_t& pltfm);

    private:
        params_t params_;
        rpos::features::artifact_provider::ArtifactUsage sltcUsage_;

        bool isFeatureSupported_;
        rclcpp::Publisher<Line2DFlt32Array>::SharedPtr pubArtifactLines_;
    };

    //////////////////////////////////////////////////////////////////////////
    
}
