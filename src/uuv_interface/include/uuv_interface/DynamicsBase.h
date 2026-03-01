#ifndef DYNAMICS_BASE_H
#define DYNAMICS_BASE_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <uuv_interface/State3D.h>
#include <uuv_interface/ControlPluginBase.h>
#include <uuv_interface/SetWrench3D.h>

namespace uuv_interface {

class DynamicsBase : public ControlPluginBase {
protected:
    // 层级拦截机制（核心）
    Eigen::VectorXd override_input_ = Eigen::VectorXd::Zero(6); // 默认6自由度力矩
    ros::ServiceServer override_srv_;
    uuv_interface::State3D state_; // 给外界的状态接口
    Eigen::VectorXd eta_; // 内部位置状态向量 [x, y, z, roll, pitch, yaw] (世界系位姿)
    Eigen::VectorXd nu_;  // 内部速度状态向量 [u, v, w, p, q, r] (机体系速度)
    Eigen::Quaterniond quat_;  // 内部状态向量 --- 四元数形式


    void registerOverrideService() override {
        // 注册动态推力接管服务
        override_srv_ = nh_.advertiseService("set_dynamics_input", &DynamicsBase::overrideCallback, this);
        UUV_WARN << "==================================================";
        UUV_WARN << "[DynamicsBase] Dynamics Override Input Service is READY.";
        UUV_WARN << "==================================================";
    }

    bool overrideCallback(uuv_interface::SetWrench3D::Request &req,
        uuv_interface::SetWrench3D::Response &res) {
        override_input_(0) = req.force_x;
        override_input_(1) = req.force_y;
        override_input_(2) = req.force_z;
        override_input_(3) = req.torque_x;
        override_input_(4) = req.torque_y;
        override_input_(5) = req.torque_z;

        is_overridden_ = true; // 收到外部调用，立刻激活接管标志
        res.success = true;
        return true;
    }

    virtual uuv_interface::State3D customUpdate(const Eigen::VectorXd& input_tau, const double& dt) = 0;

public:
    virtual ~DynamicsBase() {}

    uuv_interface::State3D getState() const {
        return state_;
    }

    // 状态设置函数
    void setState(const uuv_interface::State3D& state) {
        state_ = state;
        eta_(0) = state.x; eta_(1) = state.y; eta_(2) = state.z;
        eta_(3) = state.roll; eta_(4) = state.pitch; eta_(5) = state.yaw;
        nu_(0) = state.u; nu_(1) = state.v; nu_(2) = state.w;
        nu_(3) = state.p; nu_(4) = state.q; nu_(5) = state.r;
        // 3. 将初始欧拉角转换为四元数 quat_，防止 RK4 第一步积分时姿态错乱
        Eigen::AngleAxisd rollAngle(state.roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(state.pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(state.yaw, Eigen::Vector3d::UnitZ());
        quat_ = yawAngle * pitchAngle * rollAngle;
    }
    
    // 初始化内部状态
    void initInnerState() {
        eta_ = Eigen::VectorXd::Zero(6);
        nu_ = Eigen::VectorXd::Zero(6);
        quat_ = Eigen::Quaterniond::Identity(); // 【新增】初始化为单位四元数（0度旋转）
        state_.x = 0.0; state_.y = 0.0; state_.z = 0.0;
        state_.roll = 0.0; state_.pitch = 0.0; state_.yaw = 0.0;
        state_.u = 0.0; state_.v = 0.0; state_.w = 0.0;
        state_.p = 0.0; state_.q = 0.0; state_.r = 0.0;
    }


    uuv_interface::State3D update(const Eigen::VectorXd& input_tau) {
        // === 1. 更新频率控制
        ros::Time now = ros::Time::now();
        // 首次滴答对齐：如果时间为0，强制与主控节点的第一帧时间完美绑定
        if (last_update_time_.isZero()) {
            last_update_time_ = now;
            return state_; // 第一帧只对齐时钟，不积分
        }
        double dt_update = (now - last_update_time_).toSec();
        if (dt_update <= 0.0 || dt_update < (1.0 / this->update_rate_) * 0.95) return state_;
        last_update_time_ = now;
        // === 2. debug发布频率控制
        if (last_publish_debug_time_.isZero()) { last_publish_debug_time_ = now; }
        double dt_publish_debug = (now - last_publish_debug_time_).toSec();
        if (dt_publish_debug > (1.0 / this->publish_debug_rate_) * 0.95) {
            publishDebug(now);
            last_publish_debug_time_ = now;
        }
        // 检测是否覆写控制权，按照is_overridden，分配实际的控制输入
        const Eigen::VectorXd& actual_input_tau = is_overridden_? override_input_ : input_tau;
        return customUpdate(actual_input_tau, dt_update);
    }

};

} // namespace uuv_interface
#endif