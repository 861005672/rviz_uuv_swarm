#ifndef DYNAMICS_BASE_H
#define DYNAMICS_BASE_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <memory>
#include <uuv_interface/State3D.h>
#include <pluginlib/class_loader.h>
#include <geometry_msgs/WrenchStamped.h>
#include <uuv_interface/PluginBase.h>
#include <uuv_interface/ActuatorBase.h> // 引入执行器基类，支持 publishActuator
// #include <uuv_interface/SetWrench.h> // 假设的用于设置力的服务

namespace uuv_interface {

class DynamicsBase : public PluginBase {
protected:
    // 层级拦截机制（核心）
    bool is_overridden_ = false;
    Eigen::VectorXd override_input_ = Eigen::VectorXd::Zero(6); // 默认6自由度力矩
    ros::ServiceServer override_srv_;

    // 基类托管的状态与执行器组件
    uuv_interface::State3D state_;
    boost::shared_ptr<uuv_interface::ActuatorBase> actuator_;
    pluginlib::ClassLoader<uuv_interface::ActuatorBase> actuator_loader_;

    // 发布配置标志
    bool publish_actuator_state_ = false;
    ros::Time last_actuator_pub_time_;

    // ROS 服务回调函数（伪代码演示：假设使用类似 SetWrench 的服务来拦截底层推力指令）
    // bool overrideCallback(uuv_interface::SetWrench::Request &req,
    //                       uuv_interface::SetWrench::Response &res) {
    //     override_input_(0) = req.wrench.force.x;  override_input_(1) = req.wrench.force.y;  override_input_(2) = req.wrench.force.z;
    //     override_input_(3) = req.wrench.torque.x; override_input_(4) = req.wrench.torque.y; override_input_(5) = req.wrench.torque.z;
    //     res.success = true;
    //     return true;
    // }


public:
    DynamicsBase() : actuator_loader_("uuv_interface", "uuv_interface::ActuatorBase") {}
    virtual ~DynamicsBase() {}

    // 基类层级初始化
    void initDynamicsLevel() {
        if (control_level_ == "dynamics") {
            // override_srv_ = nh_.advertiseService("set_dynamic_input", &DynamicsBase::overrideCallback, this);
            is_overridden_ = true;
        }
    }

    // 实现在基类的状态获取函数
    uuv_interface::State3D getState() const {
        return state_;
    }

    // 实现在基类的状态设置函数
    virtual void setState(const uuv_interface::State3D& state) {
        state_ = state;
    }

    // 实现在基类的执行器及可视化发布函数
    void publishActuator(const ros::Time& current_time) {
        if (actuator_) {
            // 时间差只管向后算，无需再做频率拦截，主控会严格把关
            double actuator_pub_dt = (current_time - last_actuator_pub_time_).toSec();
            if (actuator_pub_dt > 0.0) {
                actuator_->publishJointStates(current_time, actuator_pub_dt);
                if (publish_actuator_state_) {
                    actuator_->publishActuatorStates(current_time, actuator_pub_dt);
                }
                last_actuator_pub_time_ = current_time;
            }
        }
    }

    // 解析当前层级实际应使用的输入
    Eigen::VectorXd resolveInput(const Eigen::VectorXd& upper_input) {
        return is_overridden_ ? override_input_ : upper_input;
    }

    // 统一的初始化：加载参数
    virtual void initialize(ros::NodeHandle& gnh, const std::string& robot_description) = 0;

    // 严格按照需求定义的更新函数
    virtual uuv_interface::State3D update(const Eigen::VectorXd& tau_cmd) = 0;

};

} // namespace uuv_interface
#endif