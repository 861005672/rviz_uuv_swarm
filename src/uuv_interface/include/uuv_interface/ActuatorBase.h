#ifndef ACTUATORBASE_H
#define ACTUATORBASE_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <uuv_interface/ControlPluginBase.h>
#include <uuv_interface/SetWrench3D.h>
#include <uuv_interface/State3D.h>
#include <sensor_msgs/JointState.h>

namespace uuv_interface {

// ==========================================
// 1. 推进器类 (Thruster)
// ==========================================
class Thruster {
public:
    double rotor_constant; 
    double max_rpm;      
    double time_constant = 0.5; // 推进器时间常数
    std::string joint_name;
    double actual_rpm = 0.0;    // 当前真实转速
    double prop_angle = 0.0;    // 当前真实旋转角度
public:
    Thruster() = default;
    Thruster(std::string joint_name, double rotor, double max_r, double tc)
        : joint_name(joint_name), rotor_constant(rotor), max_rpm(max_r), time_constant(tc) {}

    double computeThrust(double cmd_rpm, double dt) {
        double clamped_cmd = std::max(-max_rpm, std::min(cmd_rpm, max_rpm));
        // 1. 内部平滑更新真实转速
        if (dt > 0.0) {
            double alpha = dt / (time_constant + dt);
            actual_rpm += alpha * (clamped_cmd - actual_rpm);
            prop_angle += actual_rpm * dt;
        }
        // 2. 根据真实转速计算推力
        double clamped_omega = std::max(-max_rpm, std::min(actual_rpm, max_rpm));
        return rotor_constant * clamped_omega * std::abs(clamped_omega);
    }

    void appendJointState(sensor_msgs::JointState& msg) const {
        if (!joint_name.empty()) {
            msg.name.push_back(joint_name);
            msg.position.push_back(prop_angle);
        }
    }
};
    
// ==========================================
// 2. 舵面类 (Fin)
// ==========================================
struct FinForces {
    double lift;
    double drag;
};
class Fin {
public:
    std::string joint_name;
    double multiplier;         // 决定该舵面的视觉动画是否反转 (1.0 或 -1.0)
    double fin_area;           
    double fluid_density;      
    double lift_coefficient;   
    double drag_coefficient;   
    double max_angle;          
    double time_constant = 0.2; 
    double actual_angle = 0.0;  
public:
    Fin() = default;
    Fin(std::string joint_name, double mult, double area, double density, 
        double lift_c, double drag_c, double max_ang, double tc)
        : joint_name(joint_name), multiplier(mult), fin_area(area), fluid_density(density), 
          lift_coefficient(lift_c), drag_coefficient(drag_c), max_angle(max_ang), time_constant(tc) {}

    // 对外接口：输入目标舵角、流速分量和时间，一次性返回升力和阻力
    FinForces computeForces(double cmd_angle, double u_velocity, double v_cross, double dt) {
        double clamped_cmd = std::max(-max_angle, std::min(cmd_angle, max_angle));
        // 1. 内部平滑更新真实舵角
        if (dt > 0.0) {
            double alpha_filter = dt / (time_constant + dt);
            actual_angle += alpha_filter * (cmd_angle - actual_angle);
        }

        // 2. 内部计算真实有效攻角
        double u_safe = std::max(std::abs(u_velocity), 0.1);
        double effective_alpha = actual_angle - std::atan2(v_cross, u_safe);
        double clamped_alpha = std::max(-max_angle, std::min(effective_alpha, max_angle));

        // 3. 计算并打包力和阻力
        FinForces forces;
        forces.lift = computeLiftForce(u_velocity, effective_alpha);
        forces.drag = computeDragForce(u_velocity, effective_alpha);
        
        return forces;
    }

    void appendJointState(sensor_msgs::JointState& msg) const {
        if (!joint_name.empty()) {
            msg.name.push_back(joint_name);
            msg.position.push_back(actual_angle * multiplier); // 只有视觉映射受 multiplier 影响
        }
    }
    
    double getLiftConstant() const {
        return 0.5 * fluid_density * fin_area * lift_coefficient;
    }

    double computeLiftForce(double u_velocity, double delta_angle) {
        double clamped_angle = std::max(-max_angle, std::min(delta_angle, max_angle));
        return getLiftConstant() * u_velocity * u_velocity * clamped_angle;
    }

    double getDragConstant() const {
        return 0.5 * fluid_density * fin_area * drag_coefficient;
    }

    double computeDragForce(double u_velocity, double delta_angle) {
        double clamped_angle = std::max(-max_angle, std::min(delta_angle, max_angle));
        return getDragConstant() * std::abs(u_velocity) * u_velocity * std::abs(clamped_angle);
    }


};

class ActuatorBase : public ControlPluginBase {

protected: 
    // 层级拦截机制（如果有人想强行接管执行器，比如注入推进器故障）
    Eigen::VectorXd override_input_ = Eigen::VectorXd::Zero(6); 
    ros::ServiceServer override_srv_;
    Eigen::VectorXd last_output_tau_ = Eigen::VectorXd::Zero(6); // 缓存实际推力

    bool overrideCallback(uuv_interface::SetWrench3D::Request &req,
        uuv_interface::SetWrench3D::Response &res) {
        override_input_ << req.force_x, req.force_y, req.force_z, 
                req.torque_x, req.torque_y, req.torque_z;
        is_overridden_ = true;
        res.success = true;
        return true;
    }
    void registerOverrideService() override {
        override_srv_ = nh_.advertiseService("set_actuator_input", &ActuatorBase::overrideCallback, this);
        UUV_WARN << "==================================================";
        UUV_WARN << "[ActuatorBase] Actuator Override Input Service is READY.";
        UUV_WARN << "==================================================";
    }
    
    virtual Eigen::VectorXd customUpdate(const Eigen::VectorXd& tau_cmd, const uuv_interface::State3D& state, double dt) = 0;

public:
    virtual ~ActuatorBase() = default;

    Eigen::VectorXd update(const Eigen::VectorXd& tau_cmd, const uuv_interface::State3D& state) {
        ros::Time now = ros::Time::now();
        // 1. 更新频率控制
        if (last_update_time_.isZero()) {
            last_update_time_ = now;
            return Eigen::VectorXd::Zero(6); 
        }
        double dt_update = (now - last_update_time_).toSec();
        if (dt_update <= 0.0 || dt_update < (1.0 / this->update_rate_) * 0.95) return last_output_tau_;
        last_update_time_ = now;

        // 2. 调试信息发布控制
        if (last_publish_debug_time_.isZero()) { last_publish_debug_time_ = now; }
        double dt_publish_debug = (now - last_publish_debug_time_).toSec();
        if (dt_publish_debug > (1.0 / this->publish_debug_rate_) * 0.95) {
            publishDebug(now);
            last_publish_debug_time_ = now;
        }
        // 3. 拦截器验证
        const Eigen::VectorXd& actual_input_cmd = is_overridden_ ? override_input_ : tau_cmd;

        // 4. 调用核心算法计算出实际输出的力，并缓存
        last_output_tau_ = customUpdate(actual_input_cmd, state, dt_update);
        return last_output_tau_;
    }

    // 2. 核心分配：输入大脑的期望力与当前流速，计算物理舵角/转速并保存在内部
    virtual void allocate(const Eigen::VectorXd& tau_cmd, const Eigen::VectorXd& nu) = 0;

    // 3. 物理反推：根据内部真实的舵角/转速，计算实际产生的物理 6DOF 力矩
    virtual Eigen::VectorXd computeActualTau(const Eigen::VectorXd& nu, double dt) = 0;

    // 4. 动画驱动：将内部状态转化为特定的 URDF 关节动画并发布出去！
    virtual void publishJointStates(const ros::Time& time) = 0;

};

} // namespace uuv_control
#endif