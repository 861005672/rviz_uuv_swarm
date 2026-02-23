#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <cmath>
#include <uuv_control/interface/DynamicsBase.h>
#include <uuv_control/State3D.h>
#include <pluginlib/class_list_macros.h>

namespace uuv_control {

// ==========================================
// 1. 类定义 (直接在CPP文件中声明)
// ==========================================
class FossenDynamics : public DynamicsBase {
public:
    FossenDynamics();
    virtual ~FossenDynamics() = default;

    virtual void initialize(ros::NodeHandle& nh) override;
    virtual uuv_control::State3D update(double dt, const Eigen::VectorXd& tau) override;
    virtual uuv_control::State3D getState() override;

private:
    // 辅助工具函数
    Eigen::MatrixXd loadMatrixFromParam(ros::NodeHandle& nh, const std::string& param_name, int rows, int cols);
    Eigen::Matrix3d skew(const Eigen::Vector3d& vec); // 计算三维向量的斜对称矩阵
    Eigen::Vector3d transformWorldToBody(const Eigen::Vector3d& vec_world, const Eigen::VectorXd& eta);    // 世界坐标系到体坐标系的转换

    // 核心动力学更新模块
    void updateCoriolisMatrix(const Eigen::VectorXd& nu);
    void updateDampingMatrix(const Eigen::VectorXd& nu);
    void updateRestoringForces(const Eigen::VectorXd& eta);

    // 状态向量
    Eigen::VectorXd eta_; // [x, y, z, phi, theta, psi] (世界系位姿)
    Eigen::VectorXd nu_;  // [u, v, w, p, q, r] (机体系速度)

    // Fossen 模型动力学矩阵
    Eigen::MatrixXd M_;         // 总质量矩阵 = M_RB + M_A
    Eigen::MatrixXd M_inv_;     // 质量矩阵的逆
    Eigen::MatrixXd C_;         // 总科里奥利与向心力矩阵 C(nu)
    Eigen::MatrixXd D_;         // 总阻尼矩阵 D(nu)
    Eigen::MatrixXd D_lin_;     // 线性阻尼系数矩阵
    Eigen::MatrixXd D_lin_forward_speed_;   // [新增] 前向速度耦合阻尼矩阵
    Eigen::MatrixXd D_quad_;    // 二次阻尼系数矩阵
    Eigen::VectorXd g_;         // 恢复力/力矩向量 g(eta)

    // 物理参数
    double mass_;
    double volume_;
    double fluid_density_;
    double gravity_;
    double W_; // 重力 (W = m * g)
    double B_; // 浮力 (B = rho * V * g)
    bool neutrally_buoyant_ = false;

    Eigen::Vector3d cog_; // 重心位置 (Center of Gravity)
    Eigen::Vector3d cob_; // 浮心位置 (Center of Buoyancy)

    uuv_control::State3D current_state_;
};

// ==========================================
// 2. 类实现
// ==========================================

FossenDynamics::FossenDynamics() {
    eta_ = Eigen::VectorXd::Zero(6);
    nu_ = Eigen::VectorXd::Zero(6);
    
    M_ = Eigen::MatrixXd::Identity(6, 6);
    M_inv_ = Eigen::MatrixXd::Identity(6, 6);
    C_ = Eigen::MatrixXd::Zero(6, 6);
    D_ = Eigen::MatrixXd::Zero(6, 6);
    g_ = Eigen::VectorXd::Zero(6);
}

// 三维向量的斜对称矩阵 (Skew-symmetric matrix)，用于计算叉乘和科氏力矩阵
Eigen::Matrix3d FossenDynamics::skew(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d S;
    S <<  0,      -vec(2),  vec(1),
          vec(2),  0,      -vec(0),
         -vec(1),  vec(0),  0;
    return S;
}

Eigen::MatrixXd FossenDynamics::loadMatrixFromParam(ros::NodeHandle& nh, const std::string& param_name, int rows, int cols) {
    std::vector<double> vec;
    Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(rows, cols);
    if (nh.getParam(param_name, vec) && vec.size() == rows * cols) {
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                mat(i, j) = vec[i * cols + j];
            }
        }
    } else {
        ROS_ERROR_STREAM("[FossenDynamics] 无法加载参数或维度不匹配: " << param_name);
    }
    return mat;
}

Eigen::Vector3d FossenDynamics::transformWorldToBody(const Eigen::Vector3d& vec_world, const Eigen::VectorXd& eta) {
    double phi = eta(3);   // 横滚 Roll
    double theta = eta(4); // 俯仰 Pitch
    double psi = eta(5);   // 偏航 Yaw

    double c_phi = cos(phi), s_phi = sin(phi);
    double c_theta = cos(theta), s_theta = sin(theta);
    double c_psi = cos(psi), s_psi = sin(psi);

    // 构建从机体系到世界系的旋转矩阵 R_b^n (Z-Y-X 顺序)
    Eigen::Matrix3d R_b_n;
    R_b_n << c_psi*c_theta, -s_psi*c_phi + c_psi*s_theta*s_phi,  s_psi*s_phi + c_psi*c_phi*s_theta,
             s_psi*c_theta,  c_psi*c_phi + s_phi*s_theta*s_psi, -c_psi*s_phi + s_theta*s_psi*c_phi,
            -s_theta,        c_theta*s_phi,                      c_theta*c_phi;

    // 世界系到机体系的转换矩阵是 R_b_n 的转置 (R_n^b)
    Eigen::Matrix3d R_n_b = R_b_n.transpose();

    // 进行坐标系转换
    return R_n_b * vec_world;
}

void FossenDynamics::initialize(ros::NodeHandle& nh) {
    std::string ns = "lauv_dynamics/";
    
    nh.param(ns + "mass", mass_, 18.0);
    nh.param(ns + "volume", volume_, 0.01755);
    nh.param(ns + "fluid_density", fluid_density_, 1028.0);
    nh.param(ns + "gravity", gravity_, 9.81);
    nh.param(ns + "neutrally_buoyant", neutrally_buoyant_, false);
    
    W_ = mass_ * gravity_;
    if (neutrally_buoyant_) {
        // 如果开启强制中性浮力，无视真实的 volume，直接让浮力等于重力
        B_ = W_;
        ROS_INFO("[FossenDynamics] 已开启强制中性浮力 (neutrally_buoyant=true)，重力与浮力已自动绝对配平！");
    } else {
        B_ = fluid_density_ * volume_ * gravity_;
        ROS_INFO("[FossenDynamics] 未开启中性浮力，真实浮力计算为: %.2f N", B_);
    }

    std::vector<double> cog_vec, cob_vec, inertia_vec;
    nh.getParam(ns + "cog", cog_vec);
    nh.getParam(ns + "cob", cob_vec);
    nh.getParam(ns + "inertia", inertia_vec);
    
    cog_ << cog_vec[0], cog_vec[1], cog_vec[2];
    cob_ << cob_vec[0], cob_vec[1], cob_vec[2];

    // 构建刚体质量矩阵 M_RB (包含质量和转动惯量)
    Eigen::MatrixXd M_rb = Eigen::MatrixXd::Zero(6, 6);
    M_rb.block<3,3>(0,0) = mass_ * Eigen::Matrix3d::Identity();
    // 假设质心与原点重合(cog=[0,0,0])的简化，如果有偏移需加上 -m*S(r_g) 等耦合项
    M_rb(3,3) = inertia_vec[0];
    M_rb(4,4) = inertia_vec[1];
    M_rb(5,5) = inertia_vec[2];
    
    Eigen::MatrixXd M_added = loadMatrixFromParam(nh, ns + "added_mass", 6, 6);
    D_lin_ = loadMatrixFromParam(nh, ns + "linear_damping", 6, 6);
    D_lin_forward_speed_ = loadMatrixFromParam(nh, ns + "linear_damping_forward_speed", 6, 6);
    D_quad_ = loadMatrixFromParam(nh, ns + "quadratic_damping", 6, 6);

    // 总质量矩阵 = 刚体质量矩阵 + 附加质量矩阵
    M_ = M_rb + M_added; 
    M_inv_ = M_.inverse();

    ROS_INFO("[FossenDynamics] 真实的 3D Fossen 模型初始化完成. 质量: %.2f, 浮力: %.2f", mass_, B_);
}

void FossenDynamics::updateCoriolisMatrix(const Eigen::VectorXd& nu) {
    // 严谨计算3D科氏力矩阵 C(nu)：包含 C_RB (刚体) 和 C_A (附加质量)
    // 根据 Fossen 理论，总科氏力可以由总质量矩阵 M 统一且优雅地求出
    Eigen::Vector3d nu_1 = nu.head<3>(); // 线速度 [u, v, w]
    Eigen::Vector3d nu_2 = nu.tail<3>(); // 角速度 [p, q, r]

    // 提取广义动量
    Eigen::Vector3d a_1 = M_.block<3,3>(0,0) * nu_1 + M_.block<3,3>(0,3) * nu_2;    // 系统在机体坐标系下的【总线动量】 包含由平移和旋转所引发的线动量变化
    Eigen::Vector3d a_2 = M_.block<3,3>(3,0) * nu_1 + M_.block<3,3>(3,3) * nu_2;    // 系统在机体坐标系下的【总角动量】 包含了平移和旋转所引发的角动量变化

    C_.setZero();
    C_.block<3,3>(0,3) = -skew(a_1);
    C_.block<3,3>(3,0) = -skew(a_1);
    C_.block<3,3>(3,3) = -skew(a_2);
}

void FossenDynamics::updateDampingMatrix(const Eigen::VectorXd& nu) {
    D_.setZero();
    // 获取当前前向速度绝对值 (通常只在前进 u>0 时尾翼升力最有效，这里取绝对值兼顾倒车)
    double u_abs = std::abs(nu(0)); 
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            // 【核心物理逻辑】：
            // 真实阻尼 = 常数线性阻尼 + (前向速度 * 耦合阻尼系数) + (绝对速度 * 二次阻尼系数)
            double dynamic_lin_damping = D_lin_(i, j) + u_abs * D_lin_forward_speed_(i, j);
            
            D_(i, j) = dynamic_lin_damping + D_quad_(i, j) * std::abs(nu(j));
        }
    }
}

void FossenDynamics::updateRestoringForces(const Eigen::VectorXd& eta) {
    // 世界系下的重力与浮力向量 (NED 坐标系，Z 轴向下)
    Eigen::Vector3d W_world(0.0, 0.0, W_);
    Eigen::Vector3d B_world(0.0, 0.0, -B_);

    // 显式调用：将世界系力转换到机体系
    Eigen::Vector3d fg = transformWorldToBody(W_world, eta);
    Eigen::Vector3d fb = transformWorldToBody(B_world, eta);
    
    g_.setZero();
    g_.head<3>() = -(fg + fb);
    g_.tail<3>() = -(skew(cog_) * fg + skew(cob_) * fb);
}

uuv_control::State3D FossenDynamics::update(double dt, const Eigen::VectorXd& tau) {
    // 1. 更新三大依赖状态的矩阵/向量
    updateCoriolisMatrix(nu_);
    updateDampingMatrix(nu_);
    updateRestoringForces(eta_);

    // 2. 动力学核心：求解线加速度与角加速度
    // Fossen 方程: M * nu_dot + C * nu + D_fossen * nu + g = tau
    // 因为我们的阻尼矩阵提取的参数带有负号 (即 D_ = -D_fossen)，所以我们在等号右边用加法： + D_ * nu
    Eigen::VectorXd coriolis_force = C_ * nu_;
    Eigen::VectorXd damping_force = D_ * nu_;
    
    Eigen::VectorXd nu_dot = M_inv_ * (tau - coriolis_force + damping_force - g_);

    // 3. 速度积分 (机体系下)
    nu_ += nu_dot * dt;

    // 4. 运动学：机体系速度向世界系速度的转移矩阵 J(eta)
    double phi = eta_(3), theta = eta_(4), psi = eta_(5);
    double c_phi = cos(phi), s_phi = sin(phi);
    double c_theta = cos(theta), s_theta = sin(theta), t_theta = tan(theta);
    double c_psi = cos(psi), s_psi = sin(psi);

    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 6);
    // J1: 旋转矩阵 R(eta)
    J.block<3,3>(0,0) << c_psi*c_theta, -s_psi*c_phi + c_psi*s_theta*s_phi, s_psi*s_phi + c_psi*c_phi*s_theta,
                         s_psi*c_theta,  c_psi*c_phi + s_phi*s_theta*s_psi, -c_psi*s_phi + s_theta*s_psi*c_phi,
                        -s_theta,        c_theta*s_phi,                     c_theta*c_phi;
                         
    // J2: 角速度转换矩阵 T(eta)
    J.block<3,3>(3,3) << 1.0, s_phi*t_theta, c_phi*t_theta,
                         0.0, c_phi,         -s_phi,
                         0.0, s_phi/c_theta, c_phi/c_theta;

    // 5. 位姿积分 (世界系下)
    Eigen::VectorXd eta_dot = J * nu_;
    eta_ += eta_dot * dt;

    // 6. 发布状态
    current_state_.x = eta_(0); current_state_.y = eta_(1); current_state_.z = eta_(2);
    current_state_.phi = eta_(3); current_state_.theta = eta_(4); current_state_.psi = eta_(5);
    current_state_.u = nu_(0); current_state_.v = nu_(1); current_state_.w = nu_(2);
    current_state_.p = nu_(3); current_state_.q = nu_(4); current_state_.r = nu_(5);

    return current_state_;
}

uuv_control::State3D FossenDynamics::getState() {
    return current_state_;
}

} // namespace uuv_control

PLUGINLIB_EXPORT_CLASS(uuv_control::FossenDynamics, uuv_control::DynamicsBase)