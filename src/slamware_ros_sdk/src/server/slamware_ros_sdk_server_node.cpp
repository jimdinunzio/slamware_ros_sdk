/**
 kint.zhao  huasheng_zyh@163.com
   2017.0721

  modified by yun.li@slamtec.com, 2019.
*/

#include <rclcpp/rclcpp.hpp>

#include  "slamware_ros_sdk_server.h"

int main(int argc, char** argv)
{
  std::string errMsg;
    rclcpp::init(argc, argv);
//  ros::init(argc, argv, "slamware_ros_sdk_server_node");

  {
    slamware_ros_sdk::SlamwareRosSdkServer rosSdkServer;
    if (!rosSdkServer.startRun(errMsg))
    {
      std::cout<<"failed to start slamware ros sdk server: "<<errMsg.c_str()<<"\n";
      return -1;
    }

    rclcpp::spin(rosSdkServer.rosNodeHandle_());

    rosSdkServer.requestStop();
    rosSdkServer.waitUntilStopped();
  }
  return 0;
}



//#include <functional>
//#include <memory>
//#include <boost/atomic.hpp>
//#include <boost/thread.hpp>
//#include <boost/thread/mutex.hpp>
//#include <boost/thread/lock_guard.hpp>
//
//#include <boost/function.hpp>
//
//#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/string.hpp"
//
////using std::placeholders::_1;
////using namespace std::placeholders;
//
//
//
//
//class Subclass
//{
//private:
//public:
//    Subclass(/* args */){};
//    ~Subclass(){};
//    std::string hello(std::string &s) const
//    {
//        return "Hello " + s;
//    }
//};
//
//
//typedef Subclass    slamware_platform_t;
//
//class MinimalSubscriber : public rclcpp::Node
//{
//public:
//    MinimalSubscriber()
//            : Node("minimal_subscriber")
//    {
//        subscription_ = this->create_subscription<std_msgs::msg::String>(
//                "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
//
//        subscription2_ = this->create_subscription<std_msgs::msg::String>(
//                "topic2", 10, std::bind(&MinimalSubscriber::topic2_callback, this, std::placeholders::_1));
//
//        subscription3_ = subscribe_("my_topic" ,  10, &MinimalSubscriber::topic3_callback);
//
//        subscription3_ = this->create_subscription<std_msgs::msg::String>(
//                    "topic", 10, std::bind(&MinimalSubscriber::topic3_callback, this, subclass_, std::placeholders::_1));
//
//
////        subscription3_ = subscribe_T_<std_msgs::msg::String>("my_topic" ,  10, &MinimalSubscriber::topic3_callback);
//
//
//
//    }
//
//private:
//
//    template<class MsgT>
//    struct msg_cb_help_t
//    {
//        typedef MsgT                                    msg_t;
//        typedef typename msg_t::SharedPtr                const_msg_shared_ptr;
//        typedef void (MinimalSubscriber::*msg_cb_perform_fun_t)(slamware_platform_t&, const const_msg_shared_ptr&);
//        typedef boost::function< void(const const_msg_shared_ptr&) >        ros_cb_fun_t; // callback function for ROS.
//    };
//
//    //////////////////////////////////////////////////////////////////////////
//
//    template<class MsgT>
//    void msgCbWrapperFun_T_(typename msg_cb_help_t<MsgT>::msg_cb_perform_fun_t mfpCbPerform
//            , const std::string& msgTopic
//            , const typename msg_cb_help_t<MsgT>::const_msg_shared_ptr & msg
//    )
//    {
//        BOOST_ASSERT(nullptr != mfpCbPerform);
//        auto pltfm = safeGetSlamwarePlatform_();
////        if (!pltfm)
////        {
////            RCLCPP_ERROR(this->get_logger() , "process msgTopic: %s, not connected.", msgTopic.c_str());
////            return;
////        }
//
//        try
//        {
//            (this->*mfpCbPerform)(pltfm, msg);
//        }
//        catch (const std::exception& excp)
//        {
//            RCLCPP_ERROR(this->get_logger() , "process msgTopic: %s, exception: %s.", msgTopic.c_str(), excp.what());
//        }
//        catch (...)
//        {
//            RCLCPP_ERROR(this->get_logger() , "process msgTopic: %s, unknown exception.", msgTopic.c_str());
//        }
//    }
//
//
//    rclcpp::Subscription<std_msgs::msg::String> subscribe_(const std::string& msgTopic
//            , std::uint32_t queueSize
//            , typename msg_cb_help_t<std_msgs::msg::String>::msg_cb_perform_fun_t mfpCbPerform
//    )
//    {
//        typedef msg_cb_help_t<std_msgs::msg::String>     TheMsgCbHelpT;
//
//        typename TheMsgCbHelpT::ros_cb_fun_t rosCbFun(
//                boost::bind(&MinimalSubscriber::msgCbWrapperFun_T_<std_msgs::msg::String>, this, mfpCbPerform, msgTopic, _1)
//        );
//        rclcpp::Subscription<std_msgs::msg::String::SharedPtr> subs = this->template create_subscription<std_msgs::msg::String>(msgTopic, queueSize, rosCbFun);
//        return subs;
//    }
//
//    template<class MsgT>
//    rclcpp::Subscription<MsgT> subscribe_T_(const std::string& msgTopic
//            , std::uint32_t queueSize
//            , typename msg_cb_help_t<MsgT>::msg_cb_perform_fun_t mfpCbPerform
//    )
//    {
//        typedef msg_cb_help_t<MsgT>     TheMsgCbHelpT;
//
//        typename TheMsgCbHelpT::ros_cb_fun_t rosCbFun(
//                boost::bind(&MinimalSubscriber::msgCbWrapperFun_T_<MsgT>, this, mfpCbPerform, msgTopic, _1)
//        );
//        return this->template create_subscription<MsgT>(msgTopic, queueSize, rosCbFun);
//    }
//
//
//
////    template<class MsgT>
////    rclcpp::Subscription<MsgT> subscribe_T_(const std::string& msgTopic
////            , std::uint32_t queueSize
////            , typename msg_cb_help_t<MsgT>::msg_cb_perform_fun_t mfpCbPerform
////    )
////    {
////        typedef msg_cb_help_t<MsgT>     TheMsgCbHelpT;
////
////        typename TheMsgCbHelpT::ros_cb_fun_t rosCbFun(
////                boost::bind(&MinimalSubscriber::msgCbWrapperFun_T_<MsgT>, this, mfpCbPerform, msgTopic, _1)
////        );
////        return this->create_subscription<MsgT>(msgTopic, queueSize, rosCbFun);
////    }
//
//    Subclass subclass_;
//
//    slamware_platform_t safeGetSlamwarePlatform_()
//    {
//        return subclass_;
//    }
//
//    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
//    {
//        std::string s = subclass_.hello(msg->data);
//        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", s.c_str());
//    }
//
//    void topic2_callback(const std_msgs::msg::String::SharedPtr msg) const
//    {
//        std::string s = subclass_.hello(msg->data);
//        RCLCPP_INFO(this->get_logger(), "222 I heard 2: '%s'", s.c_str());
//    }
//
//    void topic3_callback(slamware_platform_t& plt_form , const std_msgs::msg::String::SharedPtr msg) const
//    {
//        std::string s = subclass_.hello(msg->data);
//        RCLCPP_INFO(this->get_logger(), "333 I heard 2: '%s'", s.c_str());
//    }
//
//
//    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
//    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription2_;
//    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription3_;
//
//
//};
//
//
//int main(int argc, char *argv[])
//{
//    rclcpp::init(argc, argv);
//    rclcpp::spin(std::make_shared<MinimalSubscriber>());
//    rclcpp::shutdown();
//    return 0;
//}
