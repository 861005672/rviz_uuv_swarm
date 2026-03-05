#ifndef UUV_INTERFACE_PLANNER_BASE_H
#define UUV_INTERFACE_PLANNER_BASE_H

#include <uuv_interface/ControlPluginBase.h>
#include <uuv_interface/TargetPoint3D.h>
#include <uuv_interface/State3D.h>
#include <uuv_interface/SetTargetPoint3D.h> // 【新增】引入服务消息头文件
#include <uuv_interface/utils/XmlParamReader.h>
#include <ros/ros.h>

namespace uuv_interface {

class PlannerBase : public ControlPluginBase {
protected:
    uuv_interface::TargetPoint3D latest_waypoint_;
    uuv_interface::TargetPoint3D latest_input_target_;
    
    // 【新增】控制权覆盖相关变量
    ros::ServiceServer override_srv_;
    uuv_interface::TargetPoint3D overridden_target_;

     // 实现基类的纯虚函数，注册本层专属的覆盖服务
     void registerOverrideService() override {
        override_srv_ = get_nh().advertiseService("set_planner_target", &PlannerBase::overrideCallback, this);
        UUV_WARN << "==================================================";
        UUV_WARN << "[PlannerBase] Planner Override Input Service is READY.";
        UUV_WARN << "==================================================";
    }

    // 覆盖服务的回调函数
    bool overrideCallback(uuv_interface::SetTargetPoint3D::Request& req, uuv_interface::SetTargetPoint3D::Response& res) {
        overridden_target_.n = req.target_n;
        overridden_target_.e = req.target_e;
        overridden_target_.d = req.target_d;
        res.success = true;
        return true;
    }

    // 留给子类实现的算法接口
    virtual uuv_interface::TargetPoint3D customUpdate(const uuv_interface::TargetPoint3D& global_target, const uuv_interface::State3D& state, double dt) = 0;

public:
    PlannerBase() {
        latest_waypoint_.id = -1; // 默认悬停
        overridden_target_.id = -1;
        latest_input_target_.id = -1;
    }
    virtual ~PlannerBase() {}

    // 主更新接口：接收全局目标，输出局部路径点
    uuv_interface::TargetPoint3D update(const uuv_interface::TargetPoint3D& global_target, const uuv_interface::State3D& state) {
        ros::Time now = ros::Time::now();
        
        // 1. 调试发布拦截
        if (last_publish_debug_time_.isZero()) { last_publish_debug_time_ = now; }
        double dt_publish_debug = (now - last_publish_debug_time_).toSec();
        if (this->publish_debug_rate_ > 0.0 && dt_publish_debug > (1.0 / this->publish_debug_rate_) * 0.95) {
            publishDebug(now);
            last_publish_debug_time_ = now;
        }

        // 2. 更新频率拦截
        if (last_update_time_.isZero()) {
            last_update_time_ = now;
            latest_waypoint_ = is_overridden_ ? overridden_target_ : global_target;
            return global_target;
        }
        double dt_update = (now - last_update_time_).toSec();
        
        if (dt_update <= 0.0 || (this->update_rate_ > 0.0 && dt_update < (1.0 / this->update_rate_) * 0.95)) {
            return latest_waypoint_;
        }
        
        last_update_time_ = now;

        // 【核心】：判断控制权是否被覆盖。如果是，则无视 Decision 传来的 target
        const uuv_interface::TargetPoint3D& actual_target = is_overridden_ ? overridden_target_ : global_target;

        // 将最终决定的输入目标传递给具体的规划算法子类
        latest_waypoint_ = customUpdate(actual_target, state, dt_update);
        
        return latest_waypoint_;
    }

};

} // namespace uuv_interface

#endif