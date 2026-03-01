#ifndef GUIDANCE_BASE_H
#define GUIDANCE_BASE_H

#include <ros/ros.h>
#include <uuv_interface/ControlPluginBase.h>
#include <uuv_interface/TargetPoint3D.h>
#include <uuv_interface/Cmd3D.h>
#include <uuv_interface/State3D.h>
#include <uuv_interface/SetTargetPoint3D.h>

namespace uuv_interface {

class GuidanceBase : public ControlPluginBase {
protected:
    // 层级拦截机制（核心）
    uuv_interface::TargetPoint3D override_input_;
    ros::ServiceServer override_srv_;

    uuv_interface::Cmd3D last_output_cmd_;

    // ROS 服务回调函数：成功截断上层输入
    bool overrideCallback(uuv_interface::SetTargetPoint3D::Request &req,
                          uuv_interface::SetTargetPoint3D::Response &res) {
        override_input_.n = req.target_n;
        override_input_.e = req.target_e;
        override_input_.d = req.target_d;
        res.success = true;
        return true;
    }

    void registerOverrideService() override {
        override_srv_ = nh_.advertiseService("set_guidance_target", &GuidanceBase::overrideCallback, this);
        UUV_WARN << "==================================================";
        UUV_WARN << "[GuidanceBase] Guidance Override Input Service is READY.";
        UUV_WARN << "==================================================";
    }

    virtual uuv_interface::Cmd3D customUpdate(const uuv_interface::TargetPoint3D& target, const uuv_interface::State3D& state, double dt) = 0;


public:
    virtual ~GuidanceBase() {}

    // 统一的纯虚 update 函数
    uuv_interface::Cmd3D update(const uuv_interface::TargetPoint3D& target, const uuv_interface::State3D& state) {
        ros::Time now = ros::Time::now();

        // 1. 频率控制与 dt 计算
        if (last_update_time_.isZero()) {
            last_update_time_ = now;
            // 初始第一帧时，指令维持静止，姿态对齐
            last_output_cmd_.target_u = 0.0;
            last_output_cmd_.target_pitch = state.pitch;
            last_output_cmd_.target_yaw = state.yaw;
            return last_output_cmd_;
        }
        double dt_update = (now - last_update_time_).toSec();
        if (dt_update <= 0.0 || dt_update < (1.0 / this->update_rate_) * 0.95) return last_output_cmd_;
        last_update_time_ = now;

        // 2. 调试发布拦截
        if (last_publish_debug_time_.isZero()) { last_publish_debug_time_ = now; }
        double dt_publish_debug = (now - last_publish_debug_time_).toSec();
        if (dt_publish_debug > (1.0 / this->publish_debug_rate_) * 0.95) {
            publishDebug(now);
            last_publish_debug_time_ = now;
        }

        // 3. 验证输入源 (是上层传进来的目标，还是玩家用手柄 / 服务设定的新目标？)
        const uuv_interface::TargetPoint3D& actual_target = is_overridden_ ? override_input_ : target;

        // 4. 执行纯粹的子类几何追踪算法
        last_output_cmd_ = customUpdate(actual_target, state, dt_update);
        return last_output_cmd_;
    }

};

} // namespace uuv_interface
#endif