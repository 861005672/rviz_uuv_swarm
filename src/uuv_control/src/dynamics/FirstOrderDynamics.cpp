#include <uuv_control/interface/DynamicsBase.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath> // 引入数学库

namespace uuv_control {

class FirstOrderDynamics : public DynamicsBase {
private:
    Eigen::VectorXd state_; // [x, y, z, phi, theta, psi, u, v, w, p, q, r]
    double mass_;
    double drag_;

public:
    void initialize(ros::NodeHandle& nh) override {
        state_ = Eigen::VectorXd::Zero(12);
        // 从参数服务器读取简单的物理参数
        nh.param("mass", mass_, 50.0); // 质量
        nh.param("drag", drag_, 10.0); // 阻力系数
        ROS_INFO("FirstOrderDynamics initialized with Mass=%.1f, Drag=%.1f", mass_, drag_);
    }

    uuv_control::State3D update(double dt, const Eigen::VectorXd& tau) override {
        // 1. 获取当前体坐标系速度 (Body Velocity)
        // vel = [u, v, w, p, q, r]
        Eigen::VectorXd vel = state_.segment(6, 6);

        // 2. 一阶惯性动力学公式 (仅更新速度)
        // F = ma + dv  =>  a = (F - dv) / m
        // 这里的 tau 是施加在【体坐标系】下的力/力矩
        Eigen::VectorXd acc = (tau - drag_ * vel) / mass_;

        // 3. 积分更新体坐标系速度
        vel += acc * dt;
        state_.segment(6, 6) = vel;

        // 4. 【关键修正】运动学更新 (Body Velocity -> World Position)
        // 使用完整的 3D 旋转矩阵将体坐标系速度转换为世界坐标系速度
        double phi = state_(3);   // Roll
        double theta = state_(4); // Pitch
        double psi = state_(5);   // Yaw
        
        double u = vel(0); // Surge
        double v = vel(1); // Sway
        double w = vel(2); // Heave

        // 预计算三角函数
        double c_phi = cos(phi); double s_phi = sin(phi);
        double c_theta = cos(theta); double s_theta = sin(theta);
        double c_psi = cos(psi); double s_psi = sin(psi);

        // === 核心修复开始：3D 旋转矩阵 (R_nb) ===
        // 将体坐标系速度 (u,v,w) 投影到 世界坐标系 (dx, dy, dz)
        
        // World X_dot
        double dx = u * (c_psi * c_theta) + 
                    v * (c_psi * s_theta * s_phi - s_psi * c_phi) + 
                    w * (c_psi * s_theta * c_phi + s_psi * s_phi);
        
        // World Y_dot
        double dy = u * (s_psi * c_theta) + 
                    v * (s_psi * s_theta * s_phi + c_psi * c_phi) + 
                    w * (s_psi * s_theta * c_phi - c_psi * s_phi);
        
        // World Z_dot
        double dz = u * (-s_theta) + 
                    v * (c_theta * s_phi) + 
                    w * (c_theta * c_phi);
        // === 核心修复结束 ===

        // 更新位置
        state_(0) += dx * dt;
        state_(1) += dy * dt;
        state_(2) += dz * dt;

        // 5. 姿态角更新
        // 注意：严格来说这里应该用欧拉角速率转换矩阵 (J2)，但对于简单仿真，
        // 且非大角度俯仰(接近90度)的情况下，直接积分角速度是可以接受的近似。
        // 为了体验更好，这里暂时保持直接积分，防止引入万向节死锁问题导致新手困惑。
        state_.segment(3, 3) += vel.segment(3, 3) * dt;

        return getState();
    }

    uuv_control::State3D getState() override {
        uuv_control::State3D msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "ned";
        msg.x = state_(0); msg.y = state_(1); msg.z = state_(2);
        msg.phi = state_(3); msg.theta = state_(4); msg.psi = state_(5);
        msg.u = state_(6); msg.v = state_(7); msg.w = state_(8);
        msg.p = state_(9); msg.q = state_(10); msg.r = state_(11);
        return msg;
    }
};

}

PLUGINLIB_EXPORT_CLASS(uuv_control::FirstOrderDynamics, uuv_control::DynamicsBase)