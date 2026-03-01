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
    ros::Publisher pub_actuator_;
    ros::Publisher pub_joint_;
    
    uuv_interface::Thruster thruster;
    uuv_interface::Fin fin_0;  // 0号：上垂直舵 (偏航)
    uuv_interface::Fin fin_1;  // 1号：右水平舵 (俯仰)
    uuv_interface::Fin fin_2;  // 2号：下垂直舵 (偏航)
    uuv_interface::Fin fin_3;  // 3号：左水平舵 (俯仰)
    double x_fin_;

    double cmd_omega_ = 0.0;         // 指令推进器转速
    double cmd_delta_v_ = 0.0;       // 指令垂直舵角
    double cmd_delta_h_ = 0.0;       // 指令水平舵角
    double prop_force_ = 0.0;        // 推进器推力

protected:
    void initPlugin(ros::NodeHandle& nh, const std::string& plugin_xml) override {            
        uuv_interface::XmlParamReader reader(plugin_xml);

        // 使用 getChild 获取内嵌标签读取器！
        uuv_interface::XmlParamReader t_reader = reader.getChild("thruster");
        double rotor_const, max_rpm, t_tc;
        t_reader.param("rotor_constant", rotor_const, 0.0002);
        t_reader.param("max_rpm", max_rpm, 3000.0);
        t_reader.param("time_constant", t_tc, 0.6);

        double density, area, lift_c, drag_c, max_deg, fin_tc;
        uuv_interface::XmlParamReader f_reader = reader.getChild("fin");
        f_reader.param("fluid_density", density, 1028.0);
        f_reader.param("fin_area", area, 0.0064);
        f_reader.param("lift_coefficient", lift_c, 3.0);
        f_reader.param("drag_coefficient", drag_c, 1.98);
        f_reader.param("max_deg", max_deg, 30.0);
        f_reader.param("x_fin_", x_fin_, -0.4);
        f_reader.param("time_constant", fin_tc, 0.2);
        double max_rad = max_deg * M_PI / 180.0;

        thruster = uuv_interface::Thruster(get_ns() + "/thruster_0_joint", rotor_const, max_rpm, t_tc);
        // 垂直舵：0号正转，2号视觉反转
        fin_0 = uuv_interface::Fin(get_ns() + "/fin_0_joint",  1.0, area, density, lift_c, drag_c, max_rad, fin_tc);
        fin_2 = uuv_interface::Fin(get_ns() + "/fin_2_joint", -1.0, area, density, lift_c, drag_c, max_rad, fin_tc);
        // 水平舵：1号正转，3号视觉反转
        fin_1 = uuv_interface::Fin(get_ns() + "/fin_1_joint",  1.0, area, density, lift_c, drag_c, max_rad, fin_tc);
        fin_3 = uuv_interface::Fin(get_ns() + "/fin_3_joint", -1.0, area, density, lift_c, drag_c, max_rad, fin_tc);


        UUV_INFO << "[LauvActuator] Xml Params Loaded: \n thruster/rotor_constant=\n"<<thruster.rotor_constant
            <<"\n thruster/max_rpm=\n"<<thruster.max_rpm<<"\n thruster/time_constant=\n"<<thruster.time_constant<<"\n fin/fluid_density=\n"
            <<fin_0.fluid_density<<"\n fin/fin_area=\n"<<fin_0.fin_area
            <<"\n fin/lift_coefficient=\n"<<lift_c<<"\n fin/drag_coefficient=\n"<<drag_c<<"\n fin/max_deg=\n"<<max_deg
            <<"\n fin/x_fin_=\n"<<x_fin_<<"\n fin/time_constant=\n"<<fin_tc;
    }

    void initPublishDebug() override {
        pub_joint_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);
        pub_actuator_ = nh_.advertise<uuv_interface::LauvActuatorState>("actuator_states", 10);
    }

    // 分配器
    Eigen::VectorXd customUpdate(const Eigen::VectorXd& tau_cmd, const uuv_interface::State3D& state, double dt) override {
        Eigen::VectorXd nu = Eigen::VectorXd::Zero(6);
        nu << state.u, state.v, state.w, state.p, state.q, state.r;
        allocate(tau_cmd, nu);
        return computeActualTau(nu, dt);
    }

    void allocate(const Eigen::VectorXd& tau_cmd, const Eigen::VectorXd& nu) {
        double u_safe = std::max(std::abs(nu(0)), 0.5);
        double tau_X = tau_cmd(0);
        double omega_sq = std::abs(tau_X) / thruster.rotor_constant;
        cmd_omega_ = std::copysign(std::sqrt(omega_sq), tau_X);
        cmd_omega_ = std::max(-thruster.max_rpm, std::min(cmd_omega_, thruster.max_rpm));

        double lift_coeff_v = (fin_0.getLiftConstant() + fin_2.getLiftConstant()) * u_safe * u_safe;
        double tau_N = tau_cmd(5);
        double desired_Y = tau_N / x_fin_;
        cmd_delta_v_ = desired_Y / lift_coeff_v;
        cmd_delta_v_ = std::max(-fin_0.max_angle, std::min(cmd_delta_v_, fin_0.max_angle));

        double lift_coeff_h = (fin_1.getLiftConstant() + fin_3.getLiftConstant()) * u_safe * u_safe;
        double tau_M = tau_cmd(4);
        double desired_Z = tau_M / (-x_fin_);
        cmd_delta_h_ = desired_Z / lift_coeff_h;
        cmd_delta_h_ = std::max(-fin_1.max_angle, std::min(cmd_delta_h_, fin_1.max_angle));
    }

    // 计算执行器产生的实际力/力矩
    Eigen::VectorXd computeActualTau(const Eigen::VectorXd& nu, double dt) override {
        double u = nu(0); double v = nu(1); double w = nu(2);
        double p = nu(3); double q = nu(4); double r = nu(5);

        // 1. 获取平滑后的真实推进器推力 (封装好的极简接口)
        double X_force_prop = thruster.computeThrust(cmd_omega_, dt);
        // 2. 获取平滑后的真实舵面升力与阻力 (封装好的极简接口，内部自带有效攻角计算与限幅)
        uuv_interface::FinForces f0 = fin_0.computeForces(cmd_delta_v_, u, v+x_fin_*r, dt);
        uuv_interface::FinForces f2 = fin_2.computeForces(cmd_delta_v_, u, v+x_fin_*r, dt);
        uuv_interface::FinForces f1 = fin_1.computeForces(cmd_delta_h_, u, w-x_fin_*q, dt);
        uuv_interface::FinForces f3 = fin_3.computeForces(cmd_delta_h_, u, w-x_fin_*q, dt);
        
        // 3. 计算合力 (上下左右共4个舵面，所以 * 2.0)
        double Y_force = f0.lift + f2.lift;
        double Z_force = f1.lift + f3.lift;

        double drag_r = f0.drag + f2.drag;
        double drag_s = f1.drag + f3.drag;

        double X_force = X_force_prop - (drag_r + drag_s);
        prop_force_ = X_force + (drag_r + drag_s); // 用于调试记录纯推力

        // 4. 计算反馈到重心的力矩
        double M_torque = -x_fin_ * Z_force;
        double N_torque = x_fin_ * Y_force;
        Eigen::VectorXd tau_actual = Eigen::VectorXd::Zero(6);
        tau_actual << X_force, Y_force, Z_force, 0, M_torque, N_torque;

        return tau_actual;
    }

    void publishDebug(const ros::Time& time) override {
        if (!publish_debug_) return;
        if (pub_actuator_.getNumSubscribers()>0) {
            uuv_interface::LauvActuatorState msg;
            msg.header.stamp = time;
            msg.fin_horizontal_deg = fin_1.actual_angle / M_PI*180.0;
            msg.fin_vertical_deg = fin_0.actual_angle / M_PI * 180.0;
            msg.thruster_rpm = thruster.actual_rpm;
            msg.thruster_force = prop_force_;
            pub_actuator_.publish(msg);    
        }
        publishJointStates(time);
    }

    // URDF 动画映射逻辑，严格等价于原版
    void publishJointStates(const ros::Time& time) override {
        if (pub_joint_.getNumSubscribers() == 0) return;

        sensor_msgs::JointState msg;
        msg.header.stamp = time;
        
        // 让每一个物理实体自己把状态写进 msg 中！
        thruster.appendJointState(msg);
        fin_0.appendJointState(msg);
        fin_1.appendJointState(msg);
        fin_2.appendJointState(msg);
        fin_3.appendJointState(msg);
        pub_joint_.publish(msg);
    }
};

} // namespace uuv_control

PLUGINLIB_EXPORT_CLASS(uuv_control::LauvActuator, uuv_interface::ActuatorBase)