#include <uuv_control/interface/ControllerBase.h>
#include <pluginlib/class_list_macros.h>
#include <uuv_control/State3D.h>
#include <uuv_control/Cmd3D.h>  
#include <uuv_control/SetCmd3D.h>
#include <uuv_control/utils/PID.h> // 引入封装好的 PID 类和工具函数
#include <uuv_control/utils/utils.h>
#include <cmath>
#include <algorithm>
#include <uuv_control/utils/utils.h>
#include <dynamic_reconfigure/server.h>
#include <uuv_control/PidControllerConfig.h>


namespace uuv_control {

class PidController : public ControllerBase {
private:
    ros::Subscriber sub_state_;
    ros::ServiceServer srv_cmd_;
    ros::Publisher pub_cmd_;

    // 当前状态 (来自于 State3D)
    double current_u_ = 0.0;
    double current_pitch_ = 0.0;
    double current_q_ = 0.0;
    double current_yaw_ = 0.0;
    double current_r_ = 0.0;

    // 目标指令 (来自于 Cmd3D)
    double cmd_u_ = 0.0;
    double cmd_pitch_ = 0.0;
    double cmd_yaw_ = 0.0;

    ros::Time last_time_;

    // 使用 utils.h 中独立封装的 PID 类
    uuv_control::PID pid_u_;
    uuv_control::PID pid_pitch_;
    uuv_control::PID pid_q_;
    uuv_control::PID pid_yaw_;
    uuv_control::PID pid_r_;

    std::unique_ptr<dynamic_reconfigure::Server<uuv_control::PidControllerConfig>> dyn_server_;
    dynamic_reconfigure::Server<uuv_control::PidControllerConfig>::CallbackType dyn_f_;

    void reconfigCallback(uuv_control::PidControllerConfig &config, uint32_t level) {
        // 使用 utils.h 中新增的 setParams 方法平滑更新参数
        pid_u_.setParams("pid_u update", config.kp_u, config.ki_u, config.kd_u, config.max_output_u, config.max_integral_u);
        pid_pitch_.setParams("pid_pitch update", config.kp_pitch, config.ki_pitch, config.kd_pitch, config.max_output_pitch, config.max_integral_pitch);
        pid_q_.setParams("pid_q update", config.kp_q, config.ki_q, config.kd_q, config.max_output_q, config.max_integral_q);
        pid_yaw_.setParams("pid_yaw update", config.kp_yaw, config.ki_yaw, config.kd_yaw, config.max_output_yaw, config.max_integral_yaw);
        pid_r_.setParams("pid_r update", config.kp_r, config.ki_r, config.kd_r, config.max_output_r, config.max_integral_r);
        
        ROS_INFO("[PidController] PID Parameters dynamically updated!");
    }

public:
    void initialize(ros::NodeHandle& gnh) override {
        std::string ns = "/PidController/";

        // 临时变量用于读取参数
        double kp_u, ki_u, kd_u, max_output_u, max_integral_u;
        double kp_pitch, ki_pitch, kd_pitch, max_output_pitch, max_integral_pitch;
        double kp_q, ki_q, kd_q, max_output_q, max_integral_q;
        double kp_yaw, ki_yaw, kd_yaw, max_output_yaw, max_integral_yaw;
        double kp_r, ki_r, kd_r, max_output_r, max_integral_r;

        // 读取PID参数 (预设了一些经验值，可以在 launch 文件中覆写它们)
        gnh.param(ns+"kp_u", kp_u, 50.0);
        gnh.param(ns+"ki_u", ki_u, 5.0);
        gnh.param(ns+"kd_u", kd_u, 10.0);
        gnh.param(ns+"max_output_u", max_output_u, 100.0);
        gnh.param(ns+"max_integral_u", max_integral_u, 100.0);

        gnh.param(ns+"kp_pitch", kp_pitch, 100.0);
        gnh.param(ns+"ki_pitch", ki_pitch, 10.0);
        gnh.param(ns+"kd_pitch", kd_pitch, 20.0);
        gnh.param(ns+"max_output_pitch", max_output_pitch, 80.0);
        gnh.param(ns+"max_integral_pitch", max_integral_pitch, 80.0);

        gnh.param(ns+"kp_q", kp_q, 100.0);
        gnh.param(ns+"ki_q", ki_q, 10.0);
        gnh.param(ns+"kd_q", kd_q, 20.0);
        gnh.param(ns+"max_output_q", max_output_q, 20.0);
        gnh.param(ns+"max_integral_q", max_integral_q, 20.0);

        gnh.param(ns+"kp_yaw", kp_yaw, 100.0);
        gnh.param(ns+"ki_yaw", ki_yaw, 10.0);
        gnh.param(ns+"kd_yaw", kd_yaw, 20.0);
        gnh.param(ns+"max_output_yaw", max_output_yaw, 80.0);
        gnh.param(ns+"max_integral_yaw", max_integral_yaw, 80.0);

        gnh.param(ns+"kp_r", kp_r, 100.0);
        gnh.param(ns+"ki_r", ki_r, 10.0);
        gnh.param(ns+"kd_r", kd_r, 20.0);
        gnh.param(ns+"max_output_r", max_output_r, 20.0);
        gnh.param(ns+"max_integral_r", max_integral_r, 20.0);


        ROS_INFO_STREAM("[PidController] Parameter Loaded: \nkp_u=\n"<<kp_u<<" \nki_u=\n"<<ki_u<<" \nkd_u=\n"<<kd_u<<" \nmax_output_u=\n"<<max_output_u<<" \nmax_integral_u=\n"<<max_integral_u
            <<" \nkp_pitch=\n"<<kp_pitch<<" \nki_pitch=\n"<<ki_pitch<<" \nkd_pitch=\n"<<kd_pitch<<" \nmax_output_pitch=\n"<<max_output_pitch<<" \nmax_integral_pitch=\n"<<max_integral_pitch
            <<" \nkp_q=\n"<<kp_q<<" \nki_q=\n"<<ki_q<<" \nkd_q=\n"<<kd_q<<" \nmax_output_q=\n"<<max_output_q<<" \nmax_integral_q=\n"<<max_integral_q
            <<" \nkp_yaw=\n"<<kp_yaw<<" \nki_yaw=\n"<<ki_yaw<<" \nkd_yaw=\n"<<kd_yaw<<" \nmax_output_yaw=\n"<<max_output_yaw<<" \nmax_integral_yaw=\n"<<max_integral_yaw
            <<" \nkp_r=\n"<<kp_r<<" \nki_r=\n"<<ki_r<<" \nkd_r=\n"<<kd_r<<" \nmax_output_r=\n"<<max_output_r<<" \nmax_integral_r=\n"<<max_integral_r
        );
            

        // 初始化 PID 实例
        pid_u_.init(kp_u, ki_u, kd_u, max_output_u, max_integral_u);
        pid_pitch_.init(kp_pitch, ki_pitch, kd_pitch, max_output_pitch, max_integral_pitch);
        pid_q_.init(kp_q, ki_q, kd_q, max_output_q, max_integral_q);
        pid_yaw_.init(kp_yaw, ki_yaw, kd_yaw, max_output_yaw, max_integral_yaw);
        pid_r_.init(kp_r, ki_r, kd_r, max_output_r, max_integral_r);

        // 订阅状态与指令
        sub_state_ = gnh.subscribe("state", 1, &PidController::stateCallback, this);
        srv_cmd_ = gnh.advertiseService("setcmd", &PidController::cmdServiceCallback, this);
        pub_cmd_ = gnh.advertise<uuv_control::Cmd3D>("cmd", 10);
        ros::NodeHandle pid_nh("/PidController");
        dyn_server_ = std::make_unique<dynamic_reconfigure::Server<uuv_control::PidControllerConfig>>(pid_nh);
        dyn_f_ = boost::bind(&PidController::reconfigCallback, this, _1, _2);
        dyn_server_->setCallback(dyn_f_); // 绑定后会自动触发一次回调加载当前参数服务器上的值

        last_time_ = ros::Time::now();
    }

    void stateCallback(const uuv_control::State3D::ConstPtr& msg) {
        current_u_ = msg->u;
        current_pitch_ = msg->pitch;
        current_yaw_ = msg->yaw;
        current_q_ = msg->q;
        current_r_ = msg->r;
    }

    bool cmdServiceCallback(uuv_control::SetCmd3D::Request &req, uuv_control::SetCmd3D::Response &res) {
        cmd_u_ = req.target_u;
        cmd_pitch_ = uuv_control::wrapAngle(req.target_pitch);
        cmd_yaw_ = uuv_control::wrapAngle(req.target_yaw);

        // pid_u_.reset();
        // pid_pitch_.reset();
        // pid_q_.reset();
        // pid_yaw_.reset();
        // pid_r_.reset();
        
        res.success = true;
        ROS_INFO("[PidController] Received cmd3D service request! u=%.2f, pitch=%.2f, yaw=%.2f", cmd_u_, cmd_pitch_, cmd_yaw_);
        return true;
    }

    Eigen::VectorXd compute() override {
        Eigen::VectorXd tau = Eigen::VectorXd::Zero(6);
        
        ros::Time now = ros::Time::now();
        double dt = (now - last_time_).toSec();
        if (dt <= 0.0) return tau; 
        last_time_ = now;

        uuv_control::Cmd3D cmd_msg;
        cmd_msg.header.stamp = now;
        cmd_msg.target_u = cmd_u_;
        cmd_msg.target_pitch = cmd_pitch_;
        cmd_msg.target_yaw = cmd_yaw_;
        pub_cmd_.publish(cmd_msg);

        // ================= 1. 航速 PID (Surge u) =================
        double err_u = cmd_u_ - current_u_;
        double force_x = pid_u_.compute(err_u, dt);

        // ================= 2. 串级俯仰 PID (Pitch theta) =================
        // 外环: 计算期望俯仰角速度
        double err_pitch = uuv_control::wrapAngle(cmd_pitch_ - current_pitch_);
        double cmd_q = pid_pitch_.compute(err_pitch, dt);
        // 内环: 跟踪期望角速度,输出物理力矩
        double err_q = cmd_q - current_q_;
        double torque_y = pid_q_.compute(err_q, dt);

        // ================= 3. 串级偏航 PID (Yaw psi) =================
        // 外环: 计算期望偏航角速度
        double err_yaw = uuv_control::wrapAngle(cmd_yaw_ - current_yaw_);
        double cmd_r = pid_yaw_.compute(err_yaw, dt);
        // 内环: 跟踪期望偏航叫速度，输出物理力矩
        double err_r = cmd_r - current_r_;
        double torque_z = pid_r_.compute(err_r, dt);

        // 映射到 6 自由度推力向量
        tau(0) = force_x;   // 赋予 X 推力
        tau(4) = torque_y;  // 赋予 M 扭矩 (控制俯仰)
        tau(5) = torque_z;  // 赋予 N 扭矩 (控制偏航)

        return tau;
    }
};

} // namespace uuv_control

// 将其注册为 Controller 插件
PLUGINLIB_EXPORT_CLASS(uuv_control::PidController, uuv_control::ControllerBase)