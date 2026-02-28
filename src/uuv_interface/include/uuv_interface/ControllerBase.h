#ifndef CONTROLLOR_BASE_H
#define CONTROLLOR_BASE_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <sensor_msgs/Joy.h>
#include <uuv_interface/PluginBase.h>
#include <uuv_interface/Cmd3D.h>
#include <uuv_interface/State3D.h>
#include <uuv_interface/SetCmd3D.h>

namespace uuv_interface {

class ControllerBase : public PluginBase {
protected:
    // 层级拦截机制（核心）
    bool is_overridden_ = false;
    uuv_interface::Cmd3D override_input_;
    ros::ServiceServer override_srv_;

    // ROS 服务回调函数：成功截断上层输入
    bool overrideCallback(uuv_interface::SetCmd3D::Request &req,
                          uuv_interface::SetCmd3D::Response &res) {
        override_input_.target_u = req.target_u;
        override_input_.target_pitch = req.target_pitch;
        override_input_.target_yaw = req.target_yaw;
        res.success = true;
        return true;
    }

public:
    virtual ~ControllerBase() {}

    // 基类层级初始化（可由子类在 initialize 中调用）
    void initControllerLevel() {
        if (control_level_ == "controller") {
            override_srv_ = nh_.advertiseService("set_controller_input", &ControllerBase::overrideCallback, this);
            is_overridden_ = true;
        }
    }

    // 解析当前层级实际应使用的输入
    uuv_interface::Cmd3D resolveInput(const uuv_interface::Cmd3D& upper_input) {
        return is_overridden_ ? override_input_ : upper_input;
    }

    // 统一的纯虚 update 函数
    virtual Eigen::VectorXd update(const uuv_interface::Cmd3D& cmd, const uuv_interface::State3D& state) = 0;
};

} // namespace uuv_interface
#endif