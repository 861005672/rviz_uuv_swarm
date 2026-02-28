#ifndef GUIDANCE_BASE_H
#define GUIDANCE_BASE_H

#include <ros/ros.h>
#include <uuv_interface/PluginBase.h>
#include <uuv_interface/TargetPoint3D.h>
#include <uuv_interface/Cmd3D.h>
#include <uuv_interface/State3D.h>
#include <uuv_interface/SetTargetPoint3D.h>

namespace uuv_interface {

class GuidanceBase : public PluginBase {
protected:
    // 层级拦截机制（核心）
    bool is_overridden_ = false;
    uuv_interface::TargetPoint3D override_input_;
    ros::ServiceServer override_srv_;

    // ROS 服务回调函数：成功截断上层输入
    bool overrideCallback(uuv_interface::SetTargetPoint3D::Request &req,
                          uuv_interface::SetTargetPoint3D::Response &res) {
        override_input_.n = req.target_n;
        override_input_.e = req.target_e;
        override_input_.d = req.target_d;
        res.success = true;
        return true;
    }

public:
    virtual ~GuidanceBase() {}

    // 基类层级初始化（可由子类在 initialize 中调用）
    void initGuidanceLevel() {
        if (control_level_ == "guidance") {
            override_srv_ = nh_.advertiseService("set_guidance_input", &GuidanceBase::overrideCallback, this);
            is_overridden_ = true;
        }
    }

    // 解析当前层级实际应使用的输入
    uuv_interface::TargetPoint3D resolveInput(const uuv_interface::TargetPoint3D& upper_input) {
        return is_overridden_ ? override_input_ : upper_input;
    }

    // 统一的纯虚 update 函数
    virtual uuv_interface::Cmd3D update(const uuv_interface::TargetPoint3D& target, const uuv_interface::State3D& state) = 0;

};

} // namespace uuv_interface
#endif