#ifndef UUV_CONTROL_ACTUATOR_MODELS_H
#define UUV_CONTROL_ACTUATOR_MODELS_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <string>

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
};

// ==========================================
// 3. 推力/力矩分配器 (Control Allocator)
// ==========================================
class ControlAllocator {
public:
    Thruster thruster;
    Fin fin;
    double x_fin; 

    // [修改重点]：分离物理指令与 URDF 关节指令
    struct ActuatorCmd {
        double omega;     // 螺旋桨转速 (rad/s)
        
        // --- 物理计算用 (统一压心) ---
        double delta_r;   // 物理方向舵角
        double delta_s;   // 物理水平舵角
        
        // --- RViz/Gazebo 显示用 (已修复 URDF 坐标系翻转) ---
        double fin0_joint; 
        double fin1_joint;
        double fin2_joint;
        double fin3_joint;
    };

    void initialize(ros::NodeHandle& nh, const std::string& param_ns) {
        nh.param(param_ns + "rotor_constant", thruster.rotor_constant, 0.0002);
        nh.param(param_ns + "max_rpm", thruster.max_omega, 3000.0);
        
        nh.param(param_ns + "fin_area", fin.fin_area, 0.0064);
        nh.param(param_ns + "fluid_density", fin.fluid_density, 1028.0);
        nh.param(param_ns + "lift_coefficient", fin.lift_coefficient, 3.0);
        nh.param(param_ns + "drag_coefficient", fin.drag_coefficient, 1.98);
        nh.param(param_ns + "max_fin_angle", fin.max_angle, 1.57);
        nh.param(param_ns + "x_fin", x_fin, -0.4);

        ROS_INFO("[ControlAllocator] 执行器参数加载完成.");
    }

    ActuatorCmd allocate(const Eigen::VectorXd& tau_cmd, double u_velocity) {
        ActuatorCmd cmd;
        double u_safe = std::max(std::abs(u_velocity), 0.5);
        
        // 1. 推进器分配
        double tau_X = tau_cmd(0);
        double omega_sq = std::abs(tau_X) / thruster.rotor_constant;
        cmd.omega = std::copysign(std::sqrt(omega_sq), tau_X);
        cmd.omega = std::max(-thruster.max_omega, std::min(cmd.omega, thruster.max_omega));

        // 2. 物理舵面分配 (两片舵提供总升力)
        double total_lift_coeff = 2.0 * fin.getLiftConstant() * u_safe * u_safe;

        double tau_N = tau_cmd(5);
        double desired_Y = tau_N / x_fin; 
        cmd.delta_r = desired_Y / total_lift_coeff;
        cmd.delta_r = std::max(-fin.max_angle, std::min(cmd.delta_r, fin.max_angle));

        double tau_M = tau_cmd(4);
        double desired_Z = tau_M / (-x_fin);
        cmd.delta_s = desired_Z / total_lift_coeff;
        cmd.delta_s = std::max(-fin.max_angle, std::min(cmd.delta_s, fin.max_angle));

        // 3. ★ 修复 URDF 符号陷阱 ★
        // 虽然物理上是“同向转动”，但在由于 lauv_model.xacro 的 rpy 旋转，局部 Z 轴方向不同
        // Fin0 (Top, rpy="0 0 0") 和 Fin2 (Bottom, rpy="pi 0 0")
        cmd.fin0_joint = cmd.delta_r;
        cmd.fin2_joint = -cmd.delta_r; // 底舵 Z 轴倒置，发送相反的指令，视觉上才会同向偏转

        // Fin1 (Port, rpy="pi/2 0 0") 和 Fin3 (Stbd, rpy="-pi/2 0 0")
        cmd.fin1_joint = cmd.delta_s;
        cmd.fin3_joint = -cmd.delta_s; // 左右舵的 Z 轴互为反向，发送相反指令

        return cmd;
    }

    // 物理计算依然使用统一的 delta_r 和 delta_s
    Eigen::VectorXd computeActualTau(const ActuatorCmd& cmd, const Eigen::VectorXd& nu) {
        double u = nu(0);
        double v = nu(1);
        double w = nu(2);
        double p = nu(3);
        double q = nu(4);
        double r = nu(5);

        // 设定最小动压基准速度，防止 atan2 出现除以 0 的奇点
        double u_safe = std::max(std::abs(u), 0.1);

        // 1. 计算尾翼处的局部水流速度 (Local Cross-flow Velocity)
        // x_fin 通常为负值（例如 -0.4），表示在重心后方
        double v_fin = v + x_fin * r; // 尾部横向速度 (Sway + Yaw)
        double w_fin = w - x_fin * q; // 尾部垂向速度 (Heave + Pitch)

        // 2. 计算有效攻角 (Effective Angle of Attack)
        // 攻角 = 机械舵角 - 相对水流引起的侧偏角
        double alpha_r = cmd.delta_r - std::atan2(v_fin, u_safe);
        double alpha_s = cmd.delta_s - std::atan2(w_fin, u_safe);

        // 3. 计算真实升力 (使用真实的 u 作为系数，保证静止时力绝对为 0)
        double X_force = thruster.computeThrust(cmd.omega);
        double Y_force = 2.0 * fin.computeLiftForce(u, alpha_r);
        double Z_force = 2.0 * fin.computeLiftForce(u, alpha_s);

        // 4. 计算反馈到重心的力矩
        double M_torque = -x_fin * Z_force;
        double N_torque = x_fin * Y_force;

        Eigen::VectorXd tau_actual = Eigen::VectorXd::Zero(6);
        tau_actual << X_force, Y_force, Z_force, 0, M_torque, N_torque;
        
        return tau_actual;
    }
};

} // namespace uuv_control
#endif