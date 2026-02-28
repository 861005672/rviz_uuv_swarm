#include <uuv_interface/ControllerBase.h>
#include <pluginlib/class_list_macros.h>
#include <uuv_interface/State3D.h>
#include <uuv_interface/Cmd3D.h>  
#include <uuv_control/PID.h> // 引入封装好的 PID 类和工具函数
#include <uuv_interface/utils/utils.h>
#include <cmath>
#include <algorithm>
#include <dynamic_reconfigure/server.h>
#include <uuv_control/PidControllerConfig.h>
#include <uuv_interface/utils/XmlParamReader.h>


namespace uuv_control {

class PidController : public uuv_interface::ControllerBase {
private:

    ros::Time last_time_;
    Eigen::VectorXd last_tau_;

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
    void initialize(ros::NodeHandle& gnh, const std::string& plugin_xml) override {
        initializePlugin(gnh);
        initControllerLevel();

        uuv_interface::XmlParamReader reader(plugin_xml);
        
        bool enable_dynamic_reconfigure = false;
        reader.param("enable_dynamic_reconfigure", enable_dynamic_reconfigure, true);
        reader.param("update_rate", this->update_rate_, 10.0);

        // 临时变量与读取 (语法极简！)
        double kp_u, ki_u, kd_u, max_output_u, max_integral_u;
        reader.param("kp_u", kp_u, 50.0); reader.param("ki_u", ki_u, 5.0); reader.param("kd_u", kd_u, 10.0);
        reader.param("max_output_u", max_output_u, 100.0); reader.param("max_integral_u", max_integral_u, 100.0);

        double kp_pitch, ki_pitch, kd_pitch, max_output_pitch, max_integral_pitch;
        reader.param("kp_pitch", kp_pitch, 100.0); reader.param("ki_pitch", ki_pitch, 10.0); reader.param("kd_pitch", kd_pitch, 20.0);
        reader.param("max_output_pitch", max_output_pitch, 80.0); reader.param("max_integral_pitch", max_integral_pitch, 80.0);

        double kp_q, ki_q, kd_q, max_output_q, max_integral_q;
        reader.param("kp_q", kp_q, 100.0); reader.param("ki_q", ki_q, 10.0); reader.param("kd_q", kd_q, 20.0);
        reader.param("max_output_q", max_output_q, 20.0); reader.param("max_integral_q", max_integral_q, 20.0);

        double kp_yaw, ki_yaw, kd_yaw, max_output_yaw, max_integral_yaw;
        reader.param("kp_yaw", kp_yaw, 100.0); reader.param("ki_yaw", ki_yaw, 10.0); reader.param("kd_yaw", kd_yaw, 20.0);
        reader.param("max_output_yaw", max_output_yaw, 80.0); reader.param("max_integral_yaw", max_integral_yaw, 80.0);

        double kp_r, ki_r, kd_r, max_output_r, max_integral_r;
        reader.param("kp_r", kp_r, 100.0); reader.param("ki_r", ki_r, 10.0); reader.param("kd_r", kd_r, 20.0);
        reader.param("max_output_r", max_output_r, 20.0); reader.param("max_integral_r", max_integral_r, 20.0);
            

        // 初始化 PID 实例
        pid_u_.init(kp_u, ki_u, kd_u, max_output_u, max_integral_u);
        pid_pitch_.init(kp_pitch, ki_pitch, kd_pitch, max_output_pitch, max_integral_pitch);
        pid_q_.init(kp_q, ki_q, kd_q, max_output_q, max_integral_q);
        pid_yaw_.init(kp_yaw, ki_yaw, kd_yaw, max_output_yaw, max_integral_yaw);
        pid_r_.init(kp_r, ki_r, kd_r, max_output_r, max_integral_r);

        ROS_INFO_STREAM("[PidController] Parameter Loaded: \nenable_dynamic_reconfigure=\n"<<enable_dynamic_reconfigure<<"\nkp_u=\n"<<kp_u<<" \nki_u=\n"<<ki_u<<" \nkd_u=\n"<<kd_u<<" \nmax_output_u=\n"<<max_output_u<<" \nmax_integral_u=\n"<<max_integral_u
            <<" \nkp_pitch=\n"<<kp_pitch<<" \nki_pitch=\n"<<ki_pitch<<" \nkd_pitch=\n"<<kd_pitch<<" \nmax_output_pitch=\n"<<max_output_pitch<<" \nmax_integral_pitch=\n"<<max_integral_pitch
            <<" \nkp_q=\n"<<kp_q<<" \nki_q=\n"<<ki_q<<" \nkd_q=\n"<<kd_q<<" \nmax_output_q=\n"<<max_output_q<<" \nmax_integral_q=\n"<<max_integral_q
            <<" \nkp_yaw=\n"<<kp_yaw<<" \nki_yaw=\n"<<ki_yaw<<" \nkd_yaw=\n"<<kd_yaw<<" \nmax_output_yaw=\n"<<max_output_yaw<<" \nmax_integral_yaw=\n"<<max_integral_yaw
            <<" \nkp_r=\n"<<kp_r<<" \nki_r=\n"<<ki_r<<" \nkd_r=\n"<<kd_r<<" \nmax_output_r=\n"<<max_output_r<<" \nmax_integral_r=\n"<<max_integral_r
        );

        last_time_ = ros::Time(0);
        last_tau_ = Eigen::VectorXd::Zero(6);

        if (enable_dynamic_reconfigure) {
            ros::NodeHandle pid_nh(gnh, "PidController");
            dyn_server_ = std::make_unique<dynamic_reconfigure::Server<uuv_control::PidControllerConfig>>(pid_nh);
            dyn_f_ = boost::bind(&PidController::reconfigCallback, this, _1, _2);
            dyn_server_->setCallback(dyn_f_); // 绑定后会自动触发一次回调加载当前参数服务器上的值
            ROS_INFO("[PidController] Dynamic reconfigure ENABLED.");
        }
    }


    Eigen::VectorXd update(const uuv_interface::Cmd3D& cmd, const uuv_interface::State3D& state) override {        
        // 拦截器，解析出当前的输入是上层本地传输还是采用服务接收的输入
        auto actual_cmd = resolveInput(cmd);
        // 频率控制
        ros::Time now = ros::Time::now();
        if (last_time_.isZero()) {
            last_time_ = now;
            return last_tau_; // 第一帧只对齐时钟，不计算
        }
        double dt = (now - last_time_).toSec();
        if (dt <= 0.0 || dt < (1.0 / this->update_rate_) * 0.95) {
            return last_tau_; // 没到更新时间，直接返回上一次的计算结果
        }
        last_time_ = now;

        // ================= 1. 航速 PID (Surge u) =================
        double err_u = actual_cmd.target_u - state.u;
        double force_x = pid_u_.compute(err_u, dt);

        // ================= 2. 串级俯仰 PID (Pitch theta) =================
        // 外环: 计算期望俯仰角速度
        double err_pitch = uuv_interface::wrapAngle(actual_cmd.target_pitch - state.pitch);
        double cmd_q = pid_pitch_.compute(err_pitch, dt);
        // 内环: 跟踪期望角速度,输出物理力矩
        double err_q = cmd_q - state.q;
        double torque_y = pid_q_.compute(err_q, dt);

        // ================= 3. 串级偏航 PID (Yaw psi) =================
        // 外环: 计算期望偏航角速度
        double err_yaw = uuv_interface::wrapAngle(actual_cmd.target_yaw - state.yaw);
        double cmd_r = pid_yaw_.compute(err_yaw, dt);
        // 内环: 跟踪期望偏航叫速度，输出物理力矩
        double err_r = cmd_r - state.r;
        double torque_z = pid_r_.compute(err_r, dt);

        // 映射到 6 自由度推力向量
        Eigen::VectorXd tau = Eigen::VectorXd::Zero(6);
        tau(0) = force_x;   // 赋予 X 推力
        tau(4) = torque_y;  // 赋予 M 扭矩 (控制俯仰)
        tau(5) = torque_z;  // 赋予 N 扭矩 (控制偏航)
        last_tau_ = tau;
        
        return tau;
    }
};

} // namespace uuv_control

// 将其注册为 Controller 插件
PLUGINLIB_EXPORT_CLASS(uuv_control::PidController, uuv_interface::ControllerBase)