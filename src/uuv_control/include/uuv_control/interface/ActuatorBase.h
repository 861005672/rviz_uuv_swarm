#ifndef UUV_CONTROL_ACTUATOR_BASE_H
#define UUV_CONTROL_ACTUATOR_BASE_H

#include <ros/ros.h>
#include <Eigen/Dense>

namespace uuv_control {


// ==========================================
// 1. 推进器类 (Thruster)
// ==========================================
class Thruster {
public:
    double rotor_constant; 
    double max_omega;      

    double computeThrust(double omega) {
        double clamped_omega = std::max(-max_omega, std::min(omega, max_omega));
        return rotor_constant * clamped_omega * std::abs(clamped_omega);
    }
};
    
// ==========================================
// 2. 舵面类 (Fin)
// ==========================================
class Fin {
public:
    double fin_area;           
    double fluid_density;      
    double lift_coefficient;   
    double drag_coefficient;   
    double max_angle;          

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

class ActuatorBase {
public:
    virtual ~ActuatorBase() = default;

    // 1. 初始化：读取执行器专有参数，并注册自己的 JointState 发布器
    virtual void initialize(ros::NodeHandle& nh) = 0;

    // 2. 核心分配：输入大脑的期望力与当前流速，计算物理舵角/转速并保存在内部
    virtual void allocate(const Eigen::VectorXd& tau_cmd, const Eigen::VectorXd& nu) = 0;

    // 3. 物理反推：根据内部真实的舵角/转速，计算实际产生的物理 6DOF 力矩
    virtual Eigen::VectorXd computeActualTau(const Eigen::VectorXd& nu) = 0;

    // 4. 动画驱动：将内部状态转化为特定的 URDF 关节动画并发布出去！
    virtual void publishJointStates(const ros::Time& time, double dt) = 0;
};

} // namespace uuv_control
#endif