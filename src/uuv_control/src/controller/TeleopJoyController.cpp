#include <uuv_control/interface/ControllerBase.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Joy.h>

namespace uuv_control {

class TeleopJoyController : public ControllerBase {
private:
    ros::Subscriber sub_joy_;
    Eigen::VectorXd current_tau_; // 存储当前计算出的力
    double max_thrust_;
    double max_torque_Z_;
    double max_torque_Y_;
    bool lt_initialized_ = false, rt_initialized_ = false;

public:
    void initialize(ros::NodeHandle& gnh) override {
        std::string ns = "/TeleopJoyController/";
        // 1. 读取参数
        gnh.param(ns + "max_thrust", max_thrust_, 100.0);
        gnh.param(ns + "max_torque_Z", max_torque_Z_, 60.0);
        gnh.param(ns + "max_torque_Y", max_torque_Y_, 60.0);

        // 2. 初始化输出
        current_tau_ = Eigen::VectorXd::Zero(6);

        // 3. 【关键】插件自己订阅消息！
        // 注意：这里用 boost::bind 或 lambda 来绑定回调
        sub_joy_ = gnh.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopJoyController::joyCallback, this);
        
        ROS_INFO_STREAM("[TeleopJoyController] Parameter Loaded: \nmax_thrust=\n"<<max_thrust_<<" \nmax_torque_Z=\n"<<max_torque_Z_
        <<" \nmax_torque_Y=\n"<<max_torque_Y_);
    }

    // 内部回调函数：收到手柄消息就更新 current_tau_
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
        if (!lt_initialized_ && msg->axes[2] != 0.0) lt_initialized_ = true;
        if (!rt_initialized_ && msg->axes[5] != 0.0) rt_initialized_ = true;
        // 左摇杆上下(axes 1) -> 前进力 X
        current_tau_(0) = msg->axes[1] * max_thrust_;
        // 右摇杆上下(axes 4) -> 俯仰力矩 Pitch (M)
        current_tau_(4) = msg->axes[4] * max_torque_Y_;
        // LT(axes 2),RT(axes 5) -> 偏航力矩 Yaw (N)
        double lt_val, rt_val;
        if (lt_initialized_) lt_val = (1.0-msg->axes[2]) / 2.0;
        else lt_val = 0.0;
        if (rt_initialized_) rt_val = (1.0-msg->axes[5]) / 2.0;
        else rt_val = 0.0;
        current_tau_(5) = (lt_val-rt_val)*max_torque_Z_;
    }

    // 主节点调用的接口：直接返回当前存的值
    Eigen::VectorXd compute() override {
        return current_tau_;
    }
};

}
PLUGINLIB_EXPORT_CLASS(uuv_control::TeleopJoyController, uuv_control::ControllerBase)