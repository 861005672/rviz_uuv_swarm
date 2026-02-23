#include <uuv_control/interface/DynamicsBase.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath> 

namespace uuv_control {

class FirstOrderDynamics : public DynamicsBase {
private:
    Eigen::VectorXd state_; // [x, y, z, phi, theta, psi, u, v, w, p, q, r]
    double mass_;
    double drag_;

    // 记录诊断力矩
    Eigen::VectorXd last_cmd_force_ = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd last_damping_force_ = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd last_total_force_ = Eigen::VectorXd::Zero(6);

public:
    // ==========================================
    // 物理诊断接口
    // ==========================================
    virtual Eigen::VectorXd getCommandedForce() const override { return last_cmd_force_; }
    
    // 【修改核心】：实际执行力绝对、完美地等于大脑的期望力！没有任何硬件束缚！
    virtual Eigen::VectorXd getActuatorForce() const override { return last_cmd_force_; }
    
    virtual Eigen::VectorXd getDampingForce() const override { return last_damping_force_; }
    virtual Eigen::VectorXd getTotalForce() const override { return last_total_force_; }
    
    // 理想刚体没有舵机，返回全 0 的空指令
    virtual uuv_control::ControlAllocator::ActuatorCmd getActuatorCmd() const override { 
        return uuv_control::ControlAllocator::ActuatorCmd(); 
    }

    void initialize(ros::NodeHandle& nh) override {
        state_ = Eigen::VectorXd::Zero(12);
        
        nh.param("mass", mass_, 50.0); 
        nh.param("drag", drag_, 10.0); 
        
        ROS_INFO("FirstOrderDynamics initialized with Mass=%.1f, Drag=%.1f (Ideal Actuator Mode)", mass_, drag_);

        // 注册基类的可视化发布器
        initDebugPublishers(nh);
    }

    uuv_control::State3D update(double dt, const Eigen::VectorXd& tau_cmd) override {
        // 1. 获取当前体坐标系速度
        Eigen::VectorXd vel = state_.segment(6, 6);

        // 2. 纯粹的理想力赋值
        last_cmd_force_ = tau_cmd;

        // 3. 计算阻力 (简单的线性阻力)
        Eigen::VectorXd damping_force = drag_ * vel;
        last_damping_force_ = -damping_force; 
        
        // 4. 计算总净力矩 (期望推力 - 阻力)
        last_total_force_ = tau_cmd - damping_force; 

        // 5. 计算加速度 (a = F_net / m)
        Eigen::VectorXd acc = last_total_force_ / mass_;

        // 6. 速度积分
        vel += acc * dt;
        state_.segment(6, 6) = vel;

        // 7. 运动学解算 (机体速度 -> 世界系坐标)
        double phi = state_(3), theta = state_(4), psi = state_(5);
        double u = vel(0), v = vel(1), w = vel(2);

        double c_phi = cos(phi), s_phi = sin(phi);
        double c_theta = cos(theta), s_theta = sin(theta);
        double c_psi = cos(psi), s_psi = sin(psi);

        double dx = u * (c_psi * c_theta) + 
                    v * (c_psi * s_theta * s_phi - s_psi * c_phi) + 
                    w * (c_psi * s_theta * c_phi + s_psi * s_phi);
        
        double dy = u * (s_psi * c_theta) + 
                    v * (s_psi * s_theta * s_phi + c_psi * c_phi) + 
                    w * (s_psi * s_theta * c_phi - c_psi * s_phi);
        
        double dz = u * (-s_theta) + 
                    v * (c_theta * s_phi) + 
                    w * (c_theta * c_phi);

        state_(0) += dx * dt;
        state_(1) += dy * dt;
        state_(2) += dz * dt;

        // 8. 姿态角积分
        state_.segment(3, 3) += vel.segment(3, 3) * dt;

        // 9. 调用基类在 RViz 中绘制绚丽的受力对抗箭头！
        publishDebugWrenches(ros::Time::now());

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