#ifndef CONTROLLOR_BASE_H
#define CONTROLLOR_BASE_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <uuv_interface/ControlPluginBase.h>
#include <uuv_interface/Cmd3D.h>
#include <uuv_interface/State3D.h>
#include <uuv_interface/SetCmd3D.h>

namespace uuv_interface {

class ControllerBase : public ControlPluginBase {
protected:
    // 层级拦截机制（核心）
    uuv_interface::Cmd3D override_input_;
    ros::ServiceServer override_srv_;

    Eigen::VectorXd last_output_tau_ = Eigen::VectorXd::Zero(6);

    // ROS 服务回调函数：成功截断上层输入
    bool overrideCallback(uuv_interface::SetCmd3D::Request &req,
                          uuv_interface::SetCmd3D::Response &res) {
        override_input_.target_u = req.target_u;
        override_input_.target_pitch = req.target_pitch;
        override_input_.target_yaw = req.target_yaw;
        res.success = true;
        return true;
    }

    void registerOverrideService() override {
        override_srv_ = nh_.advertiseService("set_controller_input", &ControllerBase::overrideCallback, this);
        UUV_WARN << "==================================================";
        UUV_WARN << "[ControllerBase] Controller Override Input Service is READY.";
        UUV_WARN << "==================================================";
    }

    virtual Eigen::VectorXd customUpdate(const uuv_interface::Cmd3D& cmd, const uuv_interface::State3D& state, double dt) = 0;

public:
    virtual ~ControllerBase() {}

    // 统一的 update 函数 接口
    Eigen::VectorXd update(const uuv_interface::Cmd3D& cmd, const uuv_interface::State3D& state) {
        ros::Time now = ros::Time::now();
        // 1. 调试发布拦截
        if (last_publish_debug_time_.isZero()) { last_publish_debug_time_ = now; }
        double dt_publish_debug = (now - last_publish_debug_time_).toSec();
        if (dt_publish_debug > (1.0 / this->publish_debug_rate_) * 0.95) {
            publishDebug(now);
            last_publish_debug_time_ = now;
        }
        // 2. 频率控制与 dt 计算
        if (last_update_time_.isZero()) {
            last_update_time_ = now;
            return last_output_tau_; // 第一帧不对齐，返回 0
        }
        double dt_update = (now - last_update_time_).toSec();
        if (dt_update <= 0.0 || dt_update < (1.0 / this->update_rate_) * 0.95) return last_output_tau_;
        last_update_time_ = now;

        // 3. 验证输入源 (是上层制导传进来的，还是玩家用手柄接管的？)
        const uuv_interface::Cmd3D& actual_cmd = is_overridden_ ? override_input_ : cmd;

        // 4. 执行纯粹的子类数学算法
        last_output_tau_ = customUpdate(actual_cmd, state, dt_update);
        return last_output_tau_;

    }
};

} // namespace uuv_interface
#endif