#include <uuv_interface/ActuatorBase.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/JointState.h>
#include <uuv_interface/LauvActuatorState.h>
#include <cmath>
#include <algorithm>
#include <uuv_interface/utils/XmlParamReader.h>

namespace uuv_control {


class LauvActuator : public uuv_interface::ActuatorBase {
private:
    ros::Publisher pub_joint_;
    ros::Publisher pub_actuator_;
    
    uuv_interface::Thruster thruster;
    uuv_interface::Fin fin_vertical;    // 垂直舵 (偏航)
    uuv_interface::Fin fin_horizontal;  // 水平舵 (俯仰)
    double x_fin;

    double cmd_omega_ = 0.0;         // 推进器转速
    double cmd_delta_v_ = 0.0;       // 垂直舵角
    double cmd_delta_h_ = 0.0;       // 水平舵角
    double prop_angle_ = 0.0;        // 推进器角度
    double prop_force_ = 0.0;        // 推进器推力

    ros::Time last_time_;
    bool is_first_run_ = true;
    std::string uuv_name_;

public:
    void initialize(ros::NodeHandle& gnh, const std::string& plugin_xml) override {

        std::string ns = gnh.getNamespace();
    
        // 剔除可能存在的开头斜杠，转换为纯净前缀 "uuv_0"
        if (!ns.empty() && ns.front() == '/') {
            uuv_name_ = ns.substr(1);
        } else {
            uuv_name_ = ns;
        }
            
        uuv_interface::XmlParamReader reader(plugin_xml);


        // 使用 getChild 获取内嵌标签读取器！
        uuv_interface::XmlParamReader t_reader = reader.getChild("thruster");
        t_reader.param("rotor_constant", thruster.rotor_constant, 0.0002);
        t_reader.param("max_rpm", thruster.max_rpm, 3000.0);
        t_reader.param("time_constant", thruster.time_constant, 0.6);

        double density, area, lift_c, drag_c, max_deg, fin_tc;
        uuv_interface::XmlParamReader f_reader = reader.getChild("fin");
        f_reader.param("fluid_density", density, 1028.0);
        f_reader.param("fin_area", area, 0.0064);
        f_reader.param("lift_coefficient", lift_c, 3.0);
        f_reader.param("drag_coefficient", drag_c, 1.98);
        f_reader.param("max_deg", max_deg, 30.0);
        f_reader.param("x_fin", x_fin, -0.4);
        f_reader.param("time_constant", fin_tc, 0.2);
        double max_rad = max_deg * M_PI / 180.0;
        
        fin_vertical.fluid_density = density; fin_vertical.fin_area = area;
        fin_vertical.lift_coefficient = lift_c; fin_vertical.drag_coefficient = drag_c;
        fin_vertical.max_angle = max_rad; fin_vertical.time_constant = fin_tc;
        fin_horizontal.fluid_density = density; fin_horizontal.fin_area = area;
        fin_horizontal.lift_coefficient = lift_c; fin_horizontal.drag_coefficient = drag_c;
        fin_horizontal.max_angle = max_rad; fin_horizontal.time_constant = fin_tc;

        ROS_INFO_STREAM("[LauvActuator] Xml Params Loaded: thruster/rotor_constant=\n"<<thruster.rotor_constant
            <<" \nthruster/max_rpm=\n"<<thruster.max_rpm<<" \nthruster/time_constant=\n"<<thruster.time_constant<<" \nfin/fluid_density=\n"<<density<<" \nfin/fin_area=\n"<<area
            <<" \nfin/lift_coefficient=\n"<<lift_c<<" \nfin/drag_coefficient=\n"<<drag_c<<" \nfin/max_deg=\n"<<max_deg
            <<" \nfin/x_fin=\n"<<x_fin<<" \nfin/time_constant=\n"<<fin_tc);

        // 创建 joint 关节状态发布器
        pub_joint_ = gnh.advertise<sensor_msgs::JointState>("joint_states", 10);
        pub_actuator_ = gnh.advertise<uuv_interface::LauvActuatorState>("actuator_states", 10);
        last_time_ = ros::Time::now();
    }

    // 分配器
    void allocate(const Eigen::VectorXd& tau_cmd, const Eigen::VectorXd& nu) override {
        double u_safe = std::max(std::abs(nu(0)), 0.5);

        double tau_X = tau_cmd(0);
        double omega_sq = std::abs(tau_X) / thruster.rotor_constant;
        cmd_omega_ = std::copysign(std::sqrt(omega_sq), tau_X);
        cmd_omega_ = std::max(-thruster.max_rpm, std::min(cmd_omega_, thruster.max_rpm));

        double total_lift_coeff = 2.0 * fin_vertical.getLiftConstant() * u_safe * u_safe;
        
        double tau_N = tau_cmd(5);
        double desired_Y = tau_N / x_fin; 
        cmd_delta_v_ = desired_Y / total_lift_coeff;
        cmd_delta_v_ = std::max(-fin_vertical.max_angle, std::min(cmd_delta_v_, fin_vertical.max_angle));

        double tau_M = tau_cmd(4);
        double desired_Z = tau_M / (-x_fin);
        cmd_delta_h_ = desired_Z / total_lift_coeff;
        cmd_delta_h_ = std::max(-fin_horizontal.max_angle, std::min(cmd_delta_h_, fin_horizontal.max_angle));
    }

    // 计算执行器产生的实际力/力矩
    Eigen::VectorXd computeActualTau(const Eigen::VectorXd& nu) override {
        ros::Time now = ros::Time::now();
        double dt = (now - last_time_).toSec();
        last_time_ = now;
        if (is_first_run_ || dt <= 0.0 || dt > 1.0) { dt = 0.0; is_first_run_ = false; }

        double u = nu(0); double v = nu(1); double w = nu(2);
        double p = nu(3); double q = nu(4); double r = nu(5);

        // 1. 获取平滑后的真实推进器推力 (封装好的极简接口)
        double X_force_prop = thruster.computeThrust(cmd_omega_, dt);
        // 2. 获取平滑后的真实舵面升力与阻力 (封装好的极简接口，内部自带有效攻角计算与限幅)
        uuv_interface::FinForces vert_forces = fin_vertical.computeForces(cmd_delta_v_, u, v+x_fin*r, dt);
        uuv_interface::FinForces horz_forces = fin_horizontal.computeForces(cmd_delta_h_, u, w-x_fin*q, dt);
        
        // 3. 计算合力 (上下左右共4个舵面，所以 * 2.0)
        double Y_force = 2.0 * vert_forces.lift;
        double Z_force = 2.0 * horz_forces.lift;

        double drag_r = 2.0 * vert_forces.drag;
        double drag_s = 2.0 * horz_forces.drag;

        double X_force = X_force_prop - (drag_r + drag_s);
        prop_force_ = X_force + (drag_r + drag_s); // 用于调试记录纯推力

        // 4. 计算反馈到重心的力矩
        double M_torque = -x_fin * Z_force;
        double N_torque = x_fin * Y_force;
        Eigen::VectorXd tau_actual = Eigen::VectorXd::Zero(6);
        tau_actual << X_force, Y_force, Z_force, 0, M_torque, N_torque;

        return tau_actual;
    }

    void publishActuatorStates(const ros::Time& time, double dt) override {
        if (pub_actuator_.getNumSubscribers() == 0) return;
        uuv_interface::LauvActuatorState msg;
        msg.header.stamp = time;
        msg.fin_horizontal_deg = fin_horizontal.actual_angle / M_PI*180.0;
        msg.fin_vertical_deg = fin_vertical.actual_angle / M_PI * 180.0;
        msg.thruster_rpm = thruster.actual_rpm;
        msg.thruster_force = prop_force_;
        pub_actuator_.publish(msg);
    }

    // URDF 动画映射逻辑，严格等价于原版
    void publishJointStates(const ros::Time& time, double dt) override {
        if (pub_joint_.getNumSubscribers() == 0) return;

        sensor_msgs::JointState msg;
        msg.header.stamp = time;
        
        msg.name = {
            uuv_name_+"/thruster_0_joint",
            uuv_name_+"/fin_0_joint", uuv_name_+"/fin_1_joint", 
            uuv_name_+"/fin_2_joint", uuv_name_+"/fin_3_joint"
        };

        prop_angle_ += thruster.actual_rpm * dt;
        msg.position = {
            prop_angle_, 
            fin_vertical.actual_angle, fin_horizontal.actual_angle, 
            -fin_vertical.actual_angle, -fin_horizontal.actual_angle
        };

        pub_joint_.publish(msg);
    }
};

} // namespace uuv_control

PLUGINLIB_EXPORT_CLASS(uuv_control::LauvActuator, uuv_interface::ActuatorBase)