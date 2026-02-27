#ifndef DYNAMICS_BASE_H
#define DYNAMICS_BASE_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <uuv_interface/State3D.h>
#include <geometry_msgs/WrenchStamped.h>
#include <uuv_interface/PluginBase.h>

namespace uuv_interface {

class DynamicsBase : public PluginBase {
public:
    virtual ~DynamicsBase() {}

    // 初始化：加载参数
    virtual void initialize(ros::NodeHandle& gnh, const std::string& robot_description) = 0;

    // 核心步进函数：输入推力(tau)，输出下一时刻的状态
    // dt: 时间步长, tau: [Fx, Fy, Fz, Tx, Ty, Tz]
    virtual uuv_interface::State3D update(const Eigen::VectorXd& tau_cmd, const ros::Time& current_time) = 0;

    // 获取当前状态
    virtual uuv_interface::State3D getState() = 0;

    // 获取由上层控制器输出的期望受力/力矩
    virtual Eigen::VectorXd getCommandedForce() const { return Eigen::VectorXd::Zero(6); }
    // 获取执行器产生的实际力/力矩
    virtual Eigen::VectorXd getActuatorForce() const { return Eigen::VectorXd::Zero(6); }
    // 获取UUV受到的刚体科里奥利力/力矩（包含munk力矩）
    virtual Eigen::VectorXd getCoriolisForce() const { return Eigen::VectorXd::Zero(6); }
    // 获取UUV受到的水流阻力/力矩
    virtual Eigen::VectorXd getDampingForce() const { return Eigen::VectorXd::Zero(6); }
    // 获取UUV受到的恢复力/力矩
    virtual Eigen::VectorXd getRestoringForce() const { return Eigen::VectorXd::Zero(6); }
    // 获取UUV受到的合力/力矩
    virtual Eigen::VectorXd getTotalForce() const { return Eigen::VectorXd::Zero(6); }

    virtual void publishVisuals(const ros::Time& time) {}
    
protected:
    ros::Publisher pub_cmd_wrench_;
    ros::Publisher pub_actuator_wrench_;
    ros::Publisher pub_coriolis_wrench_;
    ros::Publisher pub_damping_wrench_;
    ros::Publisher pub_restoring_wrench_;
    ros::Publisher pub_total_wrench_;

    // 基类统一初始化发布器
    void initDebugPublishers(ros::NodeHandle& gnh) {

        pub_cmd_wrench_ = gnh.advertise<geometry_msgs::WrenchStamped>("cmd_wrench", 10);
        pub_actuator_wrench_ = gnh.advertise<geometry_msgs::WrenchStamped>("actuator_wrench", 10);
        pub_coriolis_wrench_ = gnh.advertise<geometry_msgs::WrenchStamped>("coriolis_wrench", 10);
        pub_damping_wrench_ = gnh.advertise<geometry_msgs::WrenchStamped>("damping_wrench", 10);
        pub_restoring_wrench_ = gnh.advertise<geometry_msgs::WrenchStamped>("restoring_wrench", 10);
        pub_total_wrench_ = gnh.advertise<geometry_msgs::WrenchStamped>("total_wrench", 10);
    }

    // 基类统一执行发布操作
    void publishDebugWrenches(const ros::Time& time, const std::string& frame_id = "uuv_0/base_link") {
        auto pubWrench = [&](ros::Publisher& pub, const Eigen::VectorXd& f) {
            geometry_msgs::WrenchStamped msg;
            msg.header.stamp = time;
            msg.header.frame_id = frame_id;
            msg.wrench.force.x = f(0); msg.wrench.force.y = f(1); msg.wrench.force.z = f(2);
            msg.wrench.torque.x = f(3); msg.wrench.torque.y = f(4); msg.wrench.torque.z = f(5);
            pub.publish(msg);
        };

        // 极其优雅：通过虚函数获取子类计算出的真实物理力
        pubWrench(pub_cmd_wrench_, getCommandedForce());
        pubWrench(pub_actuator_wrench_, getActuatorForce());
        pubWrench(pub_coriolis_wrench_, getCoriolisForce());
        pubWrench(pub_damping_wrench_, getDampingForce());
        pubWrench(pub_restoring_wrench_, getRestoringForce());
        pubWrench(pub_total_wrench_, getTotalForce());
    }
};

}
#endif