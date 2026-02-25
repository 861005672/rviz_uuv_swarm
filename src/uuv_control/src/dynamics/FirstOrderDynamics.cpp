#include <uuv_control/interface/DynamicsBase.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath> 
#include <uuv_control/utils/utils.h>

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
        double roll = state_(3), pitch = state_(4), yaw = state_(5);
        double u = vel(0), v = vel(1), w = vel(2);

        double c_roll = cos(roll), s_roll = sin(roll);
        double c_pitch = cos(pitch), s_pitch = sin(pitch);
        double c_yaw = cos(yaw), s_yaw = sin(yaw);

        double dx = u * (c_yaw * c_pitch) + 
                    v * (c_yaw * s_pitch * s_roll - s_yaw * c_roll) + 
                    w * (c_yaw * s_pitch * c_roll + s_yaw * s_roll);
        
        double dy = u * (s_yaw * c_pitch) + 
                    v * (s_yaw * s_pitch * s_roll + c_yaw * c_roll) + 
                    w * (s_yaw * s_pitch * c_roll - c_yaw * s_roll);
        
        double dz = u * (-s_pitch) + 
                    v * (c_pitch * s_roll) + 
                    w * (c_pitch * c_roll);

        state_(0) += dx * dt;
        state_(1) += dy * dt;
        state_(2) += dz * dt;

        // 8. 姿态角积分
        state_.segment(3, 3) += vel.segment(3, 3) * dt;

        state_(3) = uuv_control::wrapAngle(state_(3));
        state_(4) = uuv_control::wrapAngle(state_(4));
        state_(5) = uuv_control::wrapAngle(state_(5));

        // 9. 调用基类在 RViz 中绘制绚丽的受力对抗箭头！
        publishDebugWrenches(ros::Time::now());

        return getState();
    }

    uuv_control::State3D getState() override {
        uuv_control::State3D msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "ned";
        msg.x = state_(0); msg.y = state_(1); msg.z = state_(2);
        msg.roll = state_(3); msg.pitch = state_(4); msg.yaw = state_(5);
        msg.u = state_(6); msg.v = state_(7); msg.w = state_(8);
        msg.p = state_(9); msg.q = state_(10); msg.r = state_(11);
        return msg;
    }
};

}

PLUGINLIB_EXPORT_CLASS(uuv_control::FirstOrderDynamics, uuv_control::DynamicsBase)