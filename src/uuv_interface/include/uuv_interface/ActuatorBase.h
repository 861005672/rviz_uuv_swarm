#ifndef ACTUATORBASE_H
#define ACTUATORBASE_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <uuv_interface/PluginBase.h>

namespace uuv_interface {

// ==========================================
// 1. 推进器类 (Thruster)
// ==========================================
class Thruster {
public:
    double rotor_constant; 
    double max_rpm;      
    double time_constant = 0.5; // 推进器时间常数
    double actual_rpm = 0.0;    // 当前真实转速

    double computeThrust(double cmd_rpm, double dt) {
        // 1. 内部平滑更新真实转速
        if (dt > 0.0) {
            double alpha = dt / (time_constant + dt);
            actual_rpm += alpha * (cmd_rpm - actual_rpm);
        }
        // 2. 根据真实转速计算推力
        double clamped_omega = std::max(-max_rpm, std::min(actual_rpm, max_rpm));
        return rotor_constant * clamped_omega * std::abs(clamped_omega);
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
    double fin_area;           
    double fluid_density;      
    double lift_coefficient;   
    double drag_coefficient;   
    double max_angle;          

    double time_constant = 0.2; 
    double actual_angle = 0.0;  // 保持 public 方便外部获取真实舵角用于发布动画

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

    // 唯一的对外接口：输入目标舵角、流速分量和时间，一次性返回升力和阻力
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
};

class ActuatorBase : public PluginBase {
public:
    virtual ~ActuatorBase() = default;

    // 2. 核心分配：输入大脑的期望力与当前流速，计算物理舵角/转速并保存在内部
    virtual void allocate(const Eigen::VectorXd& tau_cmd, const Eigen::VectorXd& nu) = 0;

    // 3. 物理反推：根据内部真实的舵角/转速，计算实际产生的物理 6DOF 力矩
    virtual Eigen::VectorXd computeActualTau(const Eigen::VectorXd& nu) = 0;

    // 4. 动画驱动：将内部状态转化为特定的 URDF 关节动画并发布出去！
    virtual void publishJointStates(const ros::Time& time, double dt) = 0;

    // 5. 发布调试的执行器状态
    virtual void publishActuatorStates(const ros::Time& time, double dt) = 0;
};

} // namespace uuv_control
#endif