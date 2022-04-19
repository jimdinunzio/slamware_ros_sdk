
// to test from command line:
// ros2 topic pub /topic std_msgs/msg/String '{data: "Hello there"}' --once
// ros2 service call /add_two_ints example_interfaces/AddTwoInts "{a: 1, b: 2}"

#include <map>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include "example_interfaces/srv/add_two_ints.hpp"
#include <memory>

class Subclass
{
private:
    int i;
    std::string name;
public:

    Subclass(/* args */)
    {
        i = -99;
        name = 'initial';
    };

    ~Subclass()
    {};

    std::string hello(std::string s) const
    {
        return "Hello " + s;
    };

    void set_int(int i_)
    {
        i = i_;
    };

    int get_int()
    {
        return i;
    };

    void set_name(std::string name_)
    {
        name = name_;
    };

    std::string get_name()
    {
        return std::to_string(i) + "_" + name;
    }
};


typedef Subclass slamware_platform_t;
//typedef std::shared_ptr<Subclass>    p_slamware_platform_t;

class MinimalSubscriber
{
public:
    rclcpp::Node::SharedPtr nh_;
    slamware_platform_t subclass_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription1_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription2_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription3_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription4_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription5_;

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service;
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service1;
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service2;
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service3;


    template<class MsgT>
    struct msg_cb_help_t
    {
        typedef MsgT msg_t;
        typedef typename msg_t::ConstPtr const_msg_shared_ptr;

        typedef void (MinimalSubscriber::*msg_cb_perform_fun_t)(slamware_platform_t &, const const_msg_shared_ptr &);

        //Both the following are good
//        typedef std::function< void(const const_msg_shared_ptr) >        ros_cb_fun_t; // callback function for ROS.
        typedef std::function<void(std::shared_ptr<MsgT>)> ros_cb_fun_t; // callback function for ROS.
    };

    template<class MsgT>
    void msgCbWrapperFun_T_(typename msg_cb_help_t<MsgT>::msg_cb_perform_fun_t mfpCbPerform,
                            const std::string &msgTopic,
                            const typename msg_cb_help_t<MsgT>::const_msg_shared_ptr &msg
    )
    {
//        BOOST_ASSERT(nullptr != mfpCbPerform);
        auto pltfm = safeGetSlamwarePlatform_();
//        if (!pltfm)
//        {
//            RCLCPP_ERROR(rosNodeHandle_()->get_logger(), "process msgTopic: %s, not connected.", msgTopic.c_str());
//            return;
//        }

        try
        {
            (this->*mfpCbPerform)(pltfm, msg);
        }
        catch (const std::exception &excp)
        {
            RCLCPP_ERROR(nh_->get_logger(), "process msgTopic: %s, exception: %s.", msgTopic.c_str(),
                         excp.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(nh_->get_logger(), "process msgTopic: %s, unknown exception.", msgTopic.c_str());
        }
    }
//********************************************************** Service prep ****************************************
//    srv_cb_perform_fun_t must match &MinimalSubscriber::add1
//                  bool add1(slamware_platform_t &pltfm,
//                  const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
//                  std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>  response)

//    rosCbFun   ==> ros_cb_fun_t ==>          must match original ros call back (here in srv no &)
//                                       bool add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request ,
//                                       std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>  response)
// msgCbWrapperFun_T_  must match bind parms
//                                        this, mfpCbPerform, srvMsgTopic, std::placeholders::_1, std::placeholders::_2

    template<class SrvMsgT>
    struct srv_cb_help_t
    {
        typedef SrvMsgT srv_msg_t;
        typedef std::shared_ptr<typename srv_msg_t::Request> request_t;
        typedef std::shared_ptr<typename srv_msg_t::Response> response_t;

        typedef bool (MinimalSubscriber::*srv_cb_perform_fun_t)(slamware_platform_t &, const request_t, response_t);

        typedef std::function<bool(const request_t, response_t)> ros_cb_fun_t; // callback function for ROS.
    };

    template<class SrvMsgT>
    bool srvCbWrapperFun_T_(typename srv_cb_help_t<SrvMsgT>::srv_cb_perform_fun_t mfpCbPerform,
                            const std::string &srvMsgTopic,
                            const typename srv_cb_help_t<SrvMsgT>::request_t req,
                            typename srv_cb_help_t<SrvMsgT>::response_t resp
    )
    {
//        BOOST_ASSERT(nullptr != mfpCbPerform);
        auto pltfm = safeGetSlamwarePlatform_();
//        if (!pltfm)
//        {
//            RCLCPP_ERROR(nh_->get_logger(), "process request: %s, not connected.", srvMsgTopic.c_str());
//            return false;
//        }

        try
        {
            return (this->*mfpCbPerform)(pltfm, req, resp);
        }
        catch (const std::exception &excp)
        {
            RCLCPP_ERROR(nh_->get_logger(), "process request: %s, exception: %s.", srvMsgTopic.c_str(),
                         excp.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(nh_->get_logger(), "process request: %s, unknown exception.",
                         srvMsgTopic.c_str());
        }
        return false;
    }

    MinimalSubscriber()
    {
        nh_ = rclcpp::Node::make_shared("minimal_subscriber");
//        p_subclass_ = new slamware_platform_t();
        subscription_ = nh_->create_subscription<std_msgs::msg::String>(
                "topic", 1, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));

//        boost::bind(processImagecallback, _1, argc, argv)

        std::function<void(std::shared_ptr<std_msgs::msg::String>)> fnc;
//        void coco = &MinimalSubscriber::stringControlCallback;
        fnc = std::bind(&MinimalSubscriber::stringControlCallback, this, std::placeholders::_1,
                        safeGetSlamwarePlatform_());
        subscription_ = nh_->create_subscription<std_msgs::msg::String>("topic", 10, fnc);

        subclass_.set_name("subscription2_");
        subscription2_ = nh_->create_subscription<std_msgs::msg::String>("topic", 10,
                                                                         (std::function<void(
                                                                                 std::shared_ptr<std_msgs::msg::String>)>)
                                                                                 std::bind(
                                                                                         &MinimalSubscriber::stringControlCallback3,
                                                                                         this,
                                                                                         *p_safeGetSlamwarePlatform_(),
                                                                                         std::placeholders::_1));
        //Using templating but depends on struct msg_cb_help_t
        subclass_.set_name("subscription3_");
        subscription3_ = subscribe_T_<std_msgs::msg::String>("topic", 1, &MinimalSubscriber::stringControlCallback3);
        subclass_.set_name("subscription4_");
        subscription4_ = subscribe_T_4<std_msgs::msg::String>("topic", 1,
                                                              std::bind(&MinimalSubscriber::stringControlCallback3,
                                                                        this, safeGetSlamwarePlatform_(),
                                                                        std::placeholders::_1));

        subclass_.set_name("subscription1_ like the sdk");
        subscription1_ = subscribe_T_1<std_msgs::msg::String>("topic", 1, &MinimalSubscriber::stringControlCallback3);

//        subclass_.set_name("subscription5_");
//        std::function<void(const std_msgs::msg::String::SharedPtr)> callback = std::bind(&MinimalSubscriber::stringControlCallback3,
//                                                                                         this, safeGetSlamwarePlatform_(),std::placeholders::_1);
//        subscription5_ = nh_->create_subscription<std_msgs::msg::String>("topic", 1,callback);
//
        subclass_.set_name("subscription5_2_");
        std::function<void(const std_msgs::msg::String::SharedPtr msg)> bound_callback_func =
                std::bind(&MinimalSubscriber::stringControlCallback3, this, safeGetSlamwarePlatform_(),
                          std::placeholders::_1);
        subscription5_ = nh_->create_subscription<std_msgs::msg::String>("topic", 1, bound_callback_func);

//        subclass_.set_name("subscription5_Lambda_");
//        auto callback_factory = [](const slamware_platform_t& sClass) {
//            return [sClass](const std_msgs::msg::String::SharedPtr msg) -> void {
//                printf("topic_name: %s, data: %s\n", sClass.name.c_str(), msg->data.c_str());
//            };
//        };
//        subscription5_ = nh_->create_subscription<std_msgs::msg::String>("topic", 1, callback_factory(safeGetSlamwarePlatform_()));

//*********************************************** Services call backs ************************************************
//Standard calling , no pltfrm
        service = nh_->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints",
                                                                           std::bind(&MinimalSubscriber::add, this,
                                                                                     std::placeholders::_1,
                                                                                     std::placeholders::_2));

        //BAD call , with static pltfrm ???
        service1 = nh_->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints",
                                                                         (std::function<bool(std::shared_ptr<example_interfaces::srv::AddTwoInts::Request>,std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>)>)
                                                                                 std::bind(
                                                                                         &MinimalSubscriber::add1,
                                                                                         this,
                                                                                         *p_safeGetSlamwarePlatform_(),
                                                                                         std::placeholders::_1,
                                                                                         std::placeholders::_2));
        //BAD call , with static pltfrm watch i is not changed since initialization???
        std::function<bool(std::shared_ptr<example_interfaces::srv::AddTwoInts::Request>,std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>)> fnc1;
        fnc1 = std::bind(&MinimalSubscriber::add1, this,  safeGetSlamwarePlatform_(), std::placeholders::_1,std::placeholders::_2);
        service2 = nh_->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", fnc1);

        //good calling , with dynamic pltfrm
        service3 = advertiseService_T_<example_interfaces::srv::AddTwoInts>("add_two_ints", &MinimalSubscriber::add1);

        subclass_.set_name("Dynamic");

    }


private:
    //*********************************************** Services call backs ************************************************
    mutable std::mutex slamwarePltfmLock_;

    template<class SrvMsgT>
    typename rclcpp::Service<SrvMsgT>::SharedPtr advertiseService_T_(const std::string &srvMsgTopic,
                                                                     typename srv_cb_help_t<SrvMsgT>::srv_cb_perform_fun_t mfpCbPerform
    )
    {
        typedef srv_cb_help_t<SrvMsgT> TheSrvCbHelpT;

        typename TheSrvCbHelpT::ros_cb_fun_t rosCbFun(
                std::bind(&MinimalSubscriber::srvCbWrapperFun_T_<SrvMsgT>, this, mfpCbPerform, srvMsgTopic,
                          std::placeholders::_1, std::placeholders::_2)
        );

        std::function<void(std::shared_ptr<example_interfaces::srv::AddTwoInts>)> fnc;
//        void coco = &MinimalSubscriber::stringControlCallback;
//        fnc = std::bind(&MinimalSubscriber::add1, this,  safeGetSlamwarePlatform_(), std::placeholders::_1,std::placeholders::_2);
//        subscription_ = nh_->create_service<std_msgs::msg::String>(srvMsgTopic, fnc);


        return nh_->create_service<SrvMsgT>(srvMsgTopic, rosCbFun);
    }

//    template<class SrvMsgT>
//    typename rclcpp::Service<SrvMsgT>::SharedPtr
//    advertiseService_T_(const std::string &srvMsgTopic,
//                                              typename srv_cb_help_t<SrvMsgT>::srv_cb_perform_fun_t mfpCbPerform
//    )
//    {
//        typedef srv_cb_help_t<SrvMsgT> TheSrvCbHelpT;
//
//        typename TheSrvCbHelpT::ros_cb_fun_t rosCbFun(
//                std::bind(&MinimalSubscriber::srvCbWrapperFun_T_<SrvMsgT>, this, mfpCbPerform, srvMsgTopic, std::placeholders::_1, std::placeholders::_2)
//        );
//
//        return nh_->create_service<SrvMsgT>(srvMsgTopic, rosCbFun);
//    }

    bool add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
             std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
    {

        response->sum = request->a + request->b;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                    request->a, request->b);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int) response->sum);
        return true;
    }

    bool add1(slamware_platform_t &pltfm,
              const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
              std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
    {
        int i = pltfm.get_int();
        response->sum = request->a + request->b + i;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld i:%ld",
                    request->a, request->b, i);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int) response->sum);
        return true;
    }

    //*********************************************** message call backs ************************************************
    template<class MsgT>
    typename rclcpp::Subscription<MsgT>::SharedPtr
    subscribe_T_1(const std::string &msgTopic, std::uint32_t queueSize,
                  typename msg_cb_help_t<MsgT>::msg_cb_perform_fun_t mfpCbPerform
    )
    {
        typedef msg_cb_help_t<MsgT> TheMsgCbHelpT;
        typename TheMsgCbHelpT::ros_cb_fun_t rosCbFun(
                std::bind(&MinimalSubscriber::msgCbWrapperFun_T_<MsgT>, this, mfpCbPerform, msgTopic,
                          std::placeholders::_1)
        );
        return nh_->create_subscription<MsgT>(msgTopic, queueSize, rosCbFun);
    }

    template<typename T, typename R, typename... Args, typename... BArgs>
    void gbind_mem(std::function<R(Args...)> &fn, R(T::*mfn)(Args...),
                   T *instance, BArgs &&... args)
    {
        fn = std::bind(mfn, instance, std::forward<BArgs>(args)...);
    }

    template<class MsgT>
    typename rclcpp::Subscription<MsgT>::SharedPtr
    subscribe_T_(const std::string &msgTopic, std::uint32_t queueSize,
                 typename msg_cb_help_t<MsgT>::msg_cb_perform_fun_t mfpCbPerform
    )
    {
        return nh_->create_subscription<MsgT>(msgTopic, queueSize,
                                              (std::function<void(std::shared_ptr<MsgT>)>)
                                                      std::bind(mfpCbPerform, this, safeGetSlamwarePlatform_(),
                                                                std::placeholders::_1));
    }

    template<class MsgT>
    typename rclcpp::Subscription<MsgT>::SharedPtr
    subscribe_T_4(const std::string &msgTopic, std::uint32_t queueSize,
                  std::function<void(std::shared_ptr<MsgT>)> mfpCbPerform
    )
    {
        return nh_->create_subscription<MsgT>(msgTopic, queueSize, mfpCbPerform);
    }

    template<class MsgT>
    typename rclcpp::Subscription<MsgT>::SharedPtr
    subscribe_T_5(const std::string &msgTopic, std::uint32_t queueSize,
                  std::function<void> &mfpCbPerform
    )
    {

        return nh_->create_subscription<MsgT>(msgTopic, queueSize,
                                              std::bind(&mfpCbPerform, this, safeGetSlamwarePlatform_(),
                                                        std::placeholders::_1));
    }

    slamware_platform_t safeGetSlamwarePlatform_()
    {
        std::lock_guard<std::mutex> lkGuard(slamwarePltfmLock_);
        return subclass_;
    }

    slamware_platform_t *p_safeGetSlamwarePlatform_()
    {
        return &subclass_;
    }

    void stringControlCallback3(slamware_platform_t &ss,
                                const std_msgs::msg::String::ConstPtr &msg)
    {
        std::string s = ss.get_name() + " " + msg->data;
        RCLCPP_INFO(nh_->get_logger(), " stringControlCallback3 I heard: '%s'", s.c_str());
    }

    void stringControlCallback(const std_msgs::msg::String::SharedPtr &msg, slamware_platform_t &ss)
    {
        std::string s = subclass_.hello(msg->data);
        RCLCPP_INFO(nh_->get_logger(), " stringControlCallback I heard: '%s'", s.c_str());
    }

    void topic_callback(const std_msgs::msg::String::ConstSharedPtr msg) const
    {
        std::string s = subclass_.hello(msg->data);
        RCLCPP_INFO(nh_->get_logger(), "I heard: '%s'", s.c_str());
    }


};


int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);
    auto ms = std::make_shared<MinimalSubscriber>();
//    ms->drawCall(1,2);
    ms->subclass_.set_name("Dynamic");
    long i = 0;
    while (1 == 1)
    {
        i = i + 1;
        rclcpp::spin_some(ms->nh_);
        ms->subclass_.set_int(i);
//    ms->subclass_.set_name("Dynamic "+std::to_string(i));
    }
    rclcpp::spin(ms->nh_);
    rclcpp::shutdown();
    return 0;
}
