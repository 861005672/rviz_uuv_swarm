#include <uuv_control/interface/ActuatorBase.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <algorithm>

namespace uuv_control {


class LauvActuator : public ActuatorBase {
private:
    ros::Publisher pub_joint_;
    
    Thruster thruster;
    Fin fin;
    double x_fin;

    double cmd_omega_ = 0.0;         // 推进器转速
    double cmd_delta_r_ = 0.0;       // 垂直舵角
    double cmd_delta_s_ = 0.0;       // 水平舵角
    double prop_angle_ = 0.0;        // 推进器角度

public:
    void initialize(ros::NodeHandle& nh) override {
        std::string ns = "actuators/";
        
        nh.param(ns+"thruster/rotor_constant", thruster.rotor_constant, 0.0002);
        nh.param(ns+"thruster/max_omega", thruster.max_omega, 3000.0);
        nh.param(ns+"fin/fluid_density", fin.fluid_density, 1028.0);
        nh.param(ns+"fin/fin_area", fin.fin_area, 0.0064);
        nh.param(ns+"fin/lift_coefficient", fin.lift_coefficient, 3.0);
        nh.param(ns+"fin/drag_coefficient", fin.drag_coefficient, 1.98);

        double max_deg = 30.0;
        nh.param(ns+"fin/max_angle_deg", max_deg, 30.0); 
        fin.max_angle = max_deg * M_PI / 180.0;
        
        nh.param(ns+"fin/x_fin", x_fin, -0.4);

        pub_joint_ = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

        ROS_INFO("[LauvActuator] Plugin Initialized. \n\
            thruster: rotor_constant:%.6f, max_omega:%.2f, fin: fluid_density:%.2f, fin_area:%.5f, lift_coefficient:%.2f, drag_coefficient:%.2f, max_angle_deg:%.2f, x_fin:%.2f"
            , thruster.rotor_constant, thruster.max_omega, fin.fluid_density, fin.fin_area, fin.lift_coefficient, fin.drag_coefficient, max_deg, x_fin);
    }

    // 完全等价于原版的 allocate，没有任何前馈补偿作弊
    void allocate(const Eigen::VectorXd& tau_cmd, const Eigen::VectorXd& nu) override {
        double u_safe = std::max(std::abs(nu(0)), 0.5);

        double tau_X = tau_cmd(0);
        double omega_sq = std::abs(tau_X) / thruster.rotor_constant;
        cmd_omega_ = std::copysign(std::sqrt(omega_sq), tau_X);
        cmd_omega_ = std::max(-thruster.max_omega, std::min(cmd_omega_, thruster.max_omega));

        double total_lift_coeff = 2.0 * fin.getLiftConstant() * u_safe * u_safe;
        
        double tau_N = tau_cmd(5);
        double desired_Y = tau_N / x_fin; 
        cmd_delta_r_ = desired_Y / total_lift_coeff;
        cmd_delta_r_ = std::max(-fin.max_angle, std::min(cmd_delta_r_, fin.max_angle));

        double tau_M = tau_cmd(4);
        double desired_Z = tau_M / (-x_fin);
        cmd_delta_s_ = desired_Z / total_lift_coeff;
        cmd_delta_s_ = std::max(-fin.max_angle, std::min(cmd_delta_s_, fin.max_angle));
    }

    // 完全等价于原版 computeActualTau 与 computeLiftForce 的物理法则
    Eigen::VectorXd computeActualTau(const Eigen::VectorXd& nu) override {
        double u = nu(0); double v = nu(1); double w = nu(2);
        double p = nu(3); double q = nu(4); double r = nu(5);
        
        // 1. 局部水流速度 (保护除零奇点)
        double u_safe = std::max(std::abs(u), 0.1);
        double v_fin = v + x_fin * r;
        double w_fin = w - x_fin * q;
        
        // 2. 真实有效攻角
        double alpha_r = cmd_delta_r_ - std::atan2(v_fin, u_safe); 
        double alpha_s = cmd_delta_s_ - std::atan2(w_fin, u_safe); 

        // 3. 计算真实升力 (使用真实的 u 作为系数，保证静止时力绝对为 0)
        double Y_force = 2.0 * fin.computeLiftForce(u, alpha_r);
        double Z_force = 2.0 * fin.computeLiftForce(u, alpha_s);

        double drag_r = 2.0 * fin.computeDragForce(u, alpha_r);
        double drag_s = 2.0 * fin.computeDragForce(u, alpha_s);
        double X_force = thruster.computeThrust(cmd_omega_) - (drag_r + drag_s);


        // 4. 计算反馈到重心的力矩
        double M_torque = -x_fin * Z_force;
        double N_torque = x_fin * Y_force;

        Eigen::VectorXd tau_actual = Eigen::VectorXd::Zero(6);
        tau_actual << X_force, Y_force, Z_force, 0, M_torque, N_torque;

        return tau_actual;
    }

    // URDF 动画映射逻辑，严格等价于原版
    void publishJointStates(const ros::Time& time, double dt) override {
        if (pub_joint_.getNumSubscribers() == 0) return;

        sensor_msgs::JointState msg;
        msg.header.stamp = time;
        
        msg.name = {
            "uuv_0/thruster_0_joint",
            "uuv_0/fin_0_joint", "uuv_0/fin_1_joint", 
            "uuv_0/fin_2_joint", "uuv_0/fin_3_joint"
        };

        prop_angle_ += cmd_omega_ * dt;
        msg.position = {
            prop_angle_, 
            cmd_delta_r_, cmd_delta_s_, -cmd_delta_r_, -cmd_delta_s_
        };

        pub_joint_.publish(msg);
    }
};

} // namespace uuv_control

PLUGINLIB_EXPORT_CLASS(uuv_control::LauvActuator, uuv_control::ActuatorBase)