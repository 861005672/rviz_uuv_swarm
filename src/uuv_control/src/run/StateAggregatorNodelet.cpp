#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <uuv_interface/State3D.h>
#include <uuv_interface/State3DArray.h>
#include <boost/bind/bind.hpp>
#include <map>
#include <vector>
#include <string>
#include <mutex>

namespace uuv_control {

class StateAggregatorNodelet : public nodelet::Nodelet {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    std::vector<ros::Subscriber> subs_;
    ros::Publisher pub_;
    ros::Timer timer_;
    
    std::map<int, uuv_interface::State3D> state_cache_;
    std::mutex mutex_;

    std::string group_name_;
    int uuv_count_;
    int start_global_id_;

    void stateCallback(const uuv_interface::State3D::ConstPtr& msg, int uuv_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_cache_[uuv_id] = *msg;
    }

    void timerCallback(const ros::TimerEvent& event) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_cache_.empty()) return;

        uuv_interface::State3DArray array_msg;
        array_msg.header.stamp = ros::Time::now();
        array_msg.header.frame_id = "ned";

        for (const auto& pair : state_cache_) {
            array_msg.states.push_back(pair.second);
        }
        pub_.publish(array_msg);
    }

public:
    virtual void onInit() override {
        nh_ = getNodeHandle();
        pnh_ = getPrivateNodeHandle();

        // 核心变更：读取组名和数量
        pnh_.param<std::string>("group_name", group_name_, "groupA");
        pnh_.param<int>("uuv_count", uuv_count_, 0);
        pnh_.param<int>("start_global_id", start_global_id_, 0);

        if (uuv_count_ <= 0) {
            NODELET_WARN("[StateAggregatorNodelet] 'uuv_count' is 0. Aggregator is idle.");
            return;
        }

        // 自动根据数量批量生成订阅的 Topic： /groupA/0/state, /groupA/1/state ...
        for (int id = 0; id < uuv_count_; ++id) {
            int current_global_id = start_global_id_ + id;
            std::string topic = "/uuv_" + std::to_string(current_global_id) + "/state";
            subs_.push_back(nh_.subscribe<uuv_interface::State3D>(
                topic, 10, 
                boost::bind(&StateAggregatorNodelet::stateCallback, this, boost::placeholders::_1, id)
            ));
            NODELET_INFO("[StateAggregatorNodelet] Subscribed to %s for zero-copy transport.", topic.c_str());
        }

        // 聚合数据发布到组命名空间下： /groupA/aggregated_states
        pub_ = nh_.advertise<uuv_interface::State3DArray>("/" + group_name_ + "/aggregated_states", 10);
        
        timer_ = nh_.createTimer(ros::Duration(0.1), &StateAggregatorNodelet::timerCallback, this);
        NODELET_INFO("[StateAggregatorNodelet] Initialized for [%s] with %d UUVs.", group_name_.c_str(), uuv_count_);
    }
};

} // namespace uuv_control

PLUGINLIB_EXPORT_CLASS(uuv_control::StateAggregatorNodelet, nodelet::Nodelet)