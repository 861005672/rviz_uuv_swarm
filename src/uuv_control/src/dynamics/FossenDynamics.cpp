#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <cmath>
#include <uuv_interface/DynamicsBase.h>
#include <uuv_interface/ActuatorBase.h>
#include <uuv_interface/State3D.h>
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>
#include <uuv_interface/utils/utils.h>
#include <uuv_interface/utils/XmlParamReader.h>
#include <geometry_msgs/WrenchStamped.h>

namespace uuv_control {


class FossenDynamics : public uuv_interface::DynamicsBase {

// =============== 子类实现特有的私有属性 =================
private:
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

    // Fossen 模型专属的受力缓存
    Eigen::VectorXd last_actuator_tau_ = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd last_coriolis_tau_ = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd last_damping_tau_ = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd last_restore_tau_ = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd last_total_tau_ = Eigen::VectorXd::Zero(6);
    // Fossen 模型专属的 ROS 调试力发布器
    ros::Publisher pub_actuator_wrench_;
    ros::Publisher pub_coriolis_wrench_;
    ros::Publisher pub_damping_wrench_;
    ros::Publisher pub_restoring_wrench_;
    ros::Publisher pub_total_wrench_;

private:

    // =============== 爷爷类PluginBase的纯虚函数重写 ================
    // 初始化自定义插件
    virtual void initPlugin(ros::NodeHandle& nh, const std::string& plugin_xml) override {
        // 初始化内部状态为零
        initInnerState();

        M_ = Eigen::MatrixXd::Identity(6, 6);
        M_inv_ = Eigen::MatrixXd::Identity(6, 6);
        C_ = Eigen::MatrixXd::Zero(6, 6);
        D_ = Eigen::MatrixXd::Zero(6, 6);
        g_ = Eigen::VectorXd::Zero(6);

        uuv_interface::XmlParamReader reader(plugin_xml);

        reader.param("mass", mass_, 18.0);
        reader.param("volume", volume_, 0.01755);
        reader.param("fluid_density", fluid_density_, 1028.0);
        reader.param("gravity", gravity_, 9.81);
        reader.param("neutrally_buoyant", neutrally_buoyant_, false);

        std::vector<double> cog_vec, cob_vec, inertia_vec;
        reader.param("cog", cog_vec, {0.0, 0.0, 0.0});
        reader.param("cob", cob_vec, {0.0, 0.0, 0.0});
        reader.param("inertia", inertia_vec, {0.1, 0.1, 0.1});

        Eigen::MatrixXd M_added = Eigen::MatrixXd::Zero(6,6);
        D_lin_ = Eigen::MatrixXd::Zero(6,6);
        D_lin_forward_speed_ = Eigen::MatrixXd::Zero(6,6);
        D_quad_ = Eigen::MatrixXd::Zero(6,6);
        reader.paramMatrix("added_mass", M_added, 6, 6);
        reader.paramMatrix("linear_damping", D_lin_, 6, 6);
        reader.paramMatrix("linear_damping_forward_speed", D_lin_forward_speed_, 6, 6);
        reader.paramMatrix("quadratic_damping", D_quad_, 6, 6);

        cog_ << cog_vec[0], cog_vec[1], cog_vec[2];
        cob_ << cob_vec[0], cob_vec[1], cob_vec[2];

        // 构建刚体质量矩阵 M_RB (包含质量和转动惯量)
        Eigen::MatrixXd M_rb = Eigen::MatrixXd::Zero(6, 6);
        M_rb.block<3,3>(0,0) = mass_ * Eigen::Matrix3d::Identity();
        // 假设质心与原点重合(cog=[0,0,0])的简化，如果有偏移需加上 -m*S(r_g) 等耦合项
        M_rb(3,3) = inertia_vec[0];
        M_rb(4,4) = inertia_vec[1];
        M_rb(5,5) = inertia_vec[2];

        // 总质量矩阵 = 刚体质量矩阵 + 附加质量矩阵
        M_ = M_rb + M_added; 
        M_inv_ = M_.inverse();
        // 计算重力和浮力
        W_ = mass_ * gravity_;
        B_ = neutrally_buoyant_ ? W_ : (fluid_density_ * volume_ * gravity_);

        UUV_INFO << "[FossenDynamics] XML Params loaded: \n mass=\n"<<mass_<<"\n volume=\n"<<volume_<<"\n fluid_density=\n"<<fluid_density_
        <<"\n gravity=\n"<<gravity_<<"\n neutrally_buoyant=\n"<<neutrally_buoyant_<<"\n cog=\n"<<cog_<<"\n cob=\n"<<cob_<<"\n M_rb=\n"
        <<M_rb<<"\n M_added=\n"<<M_added<<"\n D_lin=\n"<<D_lin_<<"\n D_lin_forward_speed=\n"<<D_lin_forward_speed_<<"\n D_quad=\n"<<D_quad_
        <<"\n M=\n"<<M_<<"\n W=\n"<<W_<<"\n B=\n"<<B_;
    }

    // 初始化调试发布器
    void initPublishDebug() override {
        pub_actuator_wrench_ = get_nh().advertise<geometry_msgs::WrenchStamped>("actuator_wrench", 10);
        pub_coriolis_wrench_ = get_nh().advertise<geometry_msgs::WrenchStamped>("coriolis_wrench", 10);
        pub_damping_wrench_ = get_nh().advertise<geometry_msgs::WrenchStamped>("damping_wrench", 10);
        pub_restoring_wrench_ = get_nh().advertise<geometry_msgs::WrenchStamped>("restoring_wrench", 10);
        pub_total_wrench_ = get_nh().advertise<geometry_msgs::WrenchStamped>("total_wrench", 10);
    }

    // 执行调试发布
    void publishDebug(const ros::Time& time) override {
        if (!publish_debug_) return;
        std::string frame = get_ns().empty() ? "base_link" : get_ns() + "/base_link";
        auto pubWrench = [&](ros::Publisher& pub, const Eigen::VectorXd& f) {
            geometry_msgs::WrenchStamped msg;
            msg.header.stamp = time;
            msg.header.frame_id = frame;
            msg.wrench.force.x = f(0); msg.wrench.force.y = f(1); msg.wrench.force.z = f(2);
            msg.wrench.torque.x = f(3); msg.wrench.torque.y = f(4); msg.wrench.torque.z = f(5);
            pub.publish(msg);
        };
        pubWrench(pub_actuator_wrench_, last_actuator_tau_);
        pubWrench(pub_coriolis_wrench_, last_coriolis_tau_);
        pubWrench(pub_damping_wrench_, last_damping_tau_);
        pubWrench(pub_restoring_wrench_, last_restore_tau_);
        pubWrench(pub_total_wrench_, last_total_tau_);
    }

    // =============== 爸爸类DynamicsBase的纯虚函数重写 ==============
    // 自定义插件的更新逻辑
    virtual uuv_interface::State3D customUpdate(const Eigen::VectorXd& input_tau, const double& dt) override {
        // 0. 记录上层执行器插件发来的外力
        last_actuator_tau_ = input_tau;

        // 1. 动力学核心：求解线加速度与角加速度
        rk4Step(last_actuator_tau_, dt);
        // 2. 记录几个用于调试显示的力
        last_coriolis_tau_ = -(computeCoriolisMatrix(nu_) * nu_);
        last_damping_tau_  = computeDampingMatrix(nu_) * nu_;
        last_restore_tau_  = -computeRestoringForces(quat_);
        last_total_tau_    = last_actuator_tau_ + last_coriolis_tau_ + last_damping_tau_ + last_restore_tau_;
        // 3. 从安全的四元数中提取出欧拉角，供其他模块或消息发布使用
        double sinr_cosp = 2.0 * (quat_.w() * quat_.x() + quat_.y() * quat_.z());
        double cosr_cosp = 1.0 - 2.0 * (quat_.x() * quat_.x() + quat_.y() * quat_.y());
        eta_(3) = std::atan2(sinr_cosp, cosr_cosp); // Roll
        double sinp = 2.0 * (quat_.w() * quat_.y() - quat_.z() * quat_.x());
        if (std::abs(sinp) >= 1.0)
            eta_(4) = std::copysign(M_PI / 2.0, sinp); // Pitch 极限保护 (严防超出 [-90, 90] 度)
        else
            eta_(4) = std::asin(sinp);
        double siny_cosp = 2.0 * (quat_.w() * quat_.z() + quat_.x() * quat_.y());
        double cosy_cosp = 1.0 - 2.0 * (quat_.y() * quat_.y() + quat_.z() * quat_.z());
        eta_(5) = std::atan2(siny_cosp, cosy_cosp); // Yaw
        // 5. 向外界发布状态
        state_.x = eta_(0); state_.y = eta_(1); state_.z = eta_(2);
        state_.roll = eta_(3); state_.pitch = eta_(4); state_.yaw = eta_(5);
        state_.u = nu_(0); state_.v = nu_(1); state_.w = nu_(2);
        state_.p = nu_(3); state_.q = nu_(4); state_.r = nu_(5);
        return state_;
    }

    // =============== 子类实现特有的功能函数 ========================
private:
    // 计算三维向量的斜对称矩阵
    Eigen::Matrix3d skew(const Eigen::Vector3d& vec) const {
        Eigen::Matrix3d S;
        S <<  0,      -vec(2),  vec(1),
              vec(2),  0,      -vec(0),
             -vec(1),  vec(0),  0;
        return S;
    }

    // 将世界坐标系下的向量转移到体坐标系
    Eigen::Vector3d transformWorldToBody(const Eigen::Vector3d& vec_world, const Eigen::Quaterniond& quat) const {
        return quat.inverse() * vec_world;
    }

    // 核心动力学更新模块
    Eigen::MatrixXd computeCoriolisMatrix(const Eigen::VectorXd& nu) const {
        Eigen::Vector3d nu_1 = nu.head<3>(); 
        Eigen::Vector3d nu_2 = nu.tail<3>(); 
        Eigen::Vector3d a_1 = M_.block<3,3>(0,0) * nu_1 + M_.block<3,3>(0,3) * nu_2;    
        Eigen::Vector3d a_2 = M_.block<3,3>(3,0) * nu_1 + M_.block<3,3>(3,3) * nu_2;    
        
        Eigen::MatrixXd C = Eigen::MatrixXd::Zero(6, 6);
        // 注意：如果在类的最前面加上了 #include <uuv_interface/utils/utils.h> ，这可以直接调 skew 
        // 如果 skew 是当前类的成员函数，建议把 skew 的声明也加上 const
        C.block<3,3>(0,3) = -skew(a_1);
        C.block<3,3>(3,0) = -skew(a_1);
        C.block<3,3>(3,3) = -skew(a_2);
        return C;
    }
    
    
    Eigen::MatrixXd computeDampingMatrix(const Eigen::VectorXd& nu) const {
        Eigen::MatrixXd D = Eigen::MatrixXd::Zero(6, 6);
        double u_abs = std::abs(nu(0)); 
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                D(i, j) = D_lin_(i, j) + u_abs * D_lin_forward_speed_(i, j) + D_quad_(i, j) * std::abs(nu(j));
            }
        }
        return D;
    }
    
    
    Eigen::VectorXd computeRestoringForces(const Eigen::Quaterniond& quat) const {
        Eigen::Vector3d W_world(0.0, 0.0, W_);
        Eigen::Vector3d B_world(0.0, 0.0, -B_);
        Eigen::Vector3d fg = transformWorldToBody(W_world, quat);
        Eigen::Vector3d fb = transformWorldToBody(B_world, quat);
        
        Eigen::VectorXd g = Eigen::VectorXd::Zero(6);
        g.head<3>() = -(fg + fb);
        g.tail<3>() = -(skew(cog_) * fg + skew(cob_) * fb);
        return g;
    }


    Eigen::VectorXd getDerivatives(const Eigen::VectorXd& math_state, const Eigen::VectorXd& tau) {
        Eigen::VectorXd dot = Eigen::VectorXd::Zero(13);

        // 1. 解包纯数学状态向量
        // math_state = [x(0), y(1), z(2), qw(3), qx(4), qy(5), qz(6), u(7), v(8), w(9), p(10), q(11), r(12)]
        Eigen::Quaterniond quat(math_state(3), math_state(4), math_state(5), math_state(6)); 
        quat.normalize(); // 保证计算导数时四元数的合法性
        Eigen::VectorXd nu = math_state.segment<6>(7);
    
        // =====================================
        // 2. 运动学：计算位置与姿态的导数
        // =====================================
        dot.segment<3>(0) = quat * nu.head<3>(); // 机器速度转世界速度
    
        // 四元数导数计算公式: q_dot = 0.5 * q \otimes [0, \omega]
        Eigen::Quaterniond omega(0, nu(3), nu(4), nu(5));
        Eigen::Quaterniond q_dot;
        q_dot.w() = 0.5 * (-quat.x()*omega.x() - quat.y()*omega.y() - quat.z()*omega.z());
        q_dot.x() = 0.5 * ( quat.w()*omega.x() + quat.y()*omega.z() - quat.z()*omega.y());
        q_dot.y() = 0.5 * ( quat.w()*omega.y() - quat.x()*omega.z() + quat.z()*omega.x());
        q_dot.z() = 0.5 * ( quat.w()*omega.z() + quat.x()*omega.y() - quat.y()*omega.x());
        dot(3) = q_dot.w(); dot(4) = q_dot.x(); dot(5) = q_dot.y(); dot(6) = q_dot.z();
    
        // =====================================
        // 3. 动力学：计算速度与角速度的导数 (nu_dot)
        // =====================================
        Eigen::MatrixXd C = computeCoriolisMatrix(nu);
        Eigen::MatrixXd D = computeDampingMatrix(nu);
        Eigen::VectorXd g = computeRestoringForces(quat);
    
        // 汇总求加速度: M^{-1} * (tau - C*nu + D*nu - g)
        dot.segment<6>(7) = M_inv_ * (tau - C * nu + D * nu - g);
    
        return dot;
    }


    void rk4Step(const Eigen::VectorXd& tau_cmd, double dt) {
        // 1. 打包：将当前的物理状态组装成 13 维数学向量
        // math_state = [x, y, z, qw, qx, qy, qz, u, v, w, p, q, r]
        Eigen::VectorXd y0 = Eigen::VectorXd::Zero(13);
        y0.segment<3>(0) = eta_.head<3>(); // 位置 x, y, z
        y0(3) = quat_.w(); y0(4) = quat_.x(); y0(5) = quat_.y(); y0(6) = quat_.z(); // 四元数
        y0.segment<6>(7) = nu_; // 机体系速度向量

        // 2. RK4 核心迭代求解 (四次采样)
        Eigen::VectorXd k1 = getDerivatives(y0, tau_cmd);
        Eigen::VectorXd k2 = getDerivatives(y0 + 0.5 * dt * k1, tau_cmd);
        Eigen::VectorXd k3 = getDerivatives(y0 + 0.5 * dt * k2, tau_cmd);
        Eigen::VectorXd k4 = getDerivatives(y0 + dt * k3, tau_cmd);

        // 加权平均求取下一时刻状态
        Eigen::VectorXd y_next = y0 + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

        // 3. 拆包：将算出的新一帧状态完美赋值回我们的物理变量
        eta_.head<3>() = y_next.segment<3>(0);
        quat_.w() = y_next(3); 
        quat_.x() = y_next(4); 
        quat_.y() = y_next(5); 
        quat_.z() = y_next(6);
        quat_.normalize(); // 极其重要：积分后的四元数必须重新归一化，防止变形
        nu_ = y_next.segment<6>(7);
    }


};


} // namespace uuv_control

PLUGINLIB_EXPORT_CLASS(uuv_control::FossenDynamics, uuv_interface::DynamicsBase)