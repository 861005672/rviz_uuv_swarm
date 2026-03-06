#ifndef UUV_INTERFACE_DECISION_BASE_H
#define UUV_INTERFACE_DECISION_BASE_H

#include <uuv_interface/ControlPluginBase.h>
#include <uuv_interface/TargetPoint3D.h>
#include <uuv_interface/TargetPoint3DArray.h>
#include <uuv_interface/State3D.h>
#include <uuv_interface/Neighbor3D.h>
#include <uuv_interface/utils/XmlParamReader.h>
#include <ros/ros.h>

namespace uuv_interface {

class DecisionBase : public ControlPluginBase {
protected:
    uuv_interface::TargetPoint3D latest_target_;
    
    ros::Subscriber target_sub_;
    uuv_interface::TargetPoint3DArray latest_mission_targets_;

    // 基类统一处理环境任务列表的订阅回调
    void targetArrayCallback(const uuv_interface::TargetPoint3DArray::ConstPtr& msg) {
        UUV_INFO << "[Decision] Decision layer received a new mission list. ";
        latest_mission_targets_ = *msg;
    }

public:
    DecisionBase() {
        latest_target_.id = -1; // 默认输出停止指令
    }
    virtual ~DecisionBase() {}

    // 基类提供的统一初始化接口（重写 ControlPluginBase）
    void initialize(ros::NodeHandle& nh, const std::string& plugin_xml, const std::string& self_level) override {
        ControlPluginBase::initialize(nh, plugin_xml, self_level);
        
        // 基类统一订阅全局任务列表，子类完全不需要管 ROS 订阅细节
        target_sub_ = get_nh().subscribe("/swarm/mission_targets", 1, &DecisionBase::targetArrayCallback, this);
        
        UUV_INFO << "[DecisionBase] DecisionBase Initialized. Update Rate: " << update_rate_ << " Hz";
    }

    // 主更新接口，带有严格的频率控制机制
    uuv_interface::TargetPoint3D update(const uuv_interface::State3D& state, const std::vector<uuv_interface::Neighbor3D>& neighbors, std::string& data_json) {
        ros::Time now = ros::Time::now();
        // 1. 调试发布拦截
        if (last_publish_debug_time_.isZero()) { last_publish_debug_time_ = now; }
            double dt_publish_debug = (now - last_publish_debug_time_).toSec();
            if (dt_publish_debug > (1.0 / this->publish_debug_rate_) * 0.95) {
                publishDebug(now);
                last_publish_debug_time_ = now;
        }

        if (last_update_time_.isZero()) {
            last_update_time_ = now;
            latest_target_.id = -1;
        }
        double dt_update = (now - last_update_time_).toSec();
        if (dt_update <= 0.0 || dt_update < (1.0/this->update_rate_)*0.95) return latest_target_; 
        
        last_update_time_ = now;
        latest_target_ = customUpdate(state, latest_mission_targets_, neighbors, data_json, dt_update);
        
        // 没到时间就直接返回上一次缓存的决策目标
        return latest_target_;
    }

    // 子类必须实现的接口
    virtual void initPlugin(ros::NodeHandle& gnh, const std::string& plugin_xml) = 0;
    
    void registerOverrideService() override {}

protected:
    // 自定义算法逻辑：纯粹的输入输出，无需操心频率和ROS通信
    virtual uuv_interface::TargetPoint3D customUpdate(const uuv_interface::State3D& state, 
        const uuv_interface::TargetPoint3DArray& mission_list,
        const std::vector<uuv_interface::Neighbor3D>& neighbors, 
        std::string& data_json, double dt) = 0;


};

} // namespace uuv_interface

#endif