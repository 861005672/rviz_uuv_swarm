#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <uuv_control/interface/DynamicsBase.h>
#include <uuv_control/interface/ControllerBase.h>
#include <geometry_msgs/Wrench.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <uuv_control/actuator/ActuatorModels.h>

class UUVControlNode {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_; // 私有句柄用于读取参数
    
    // 插件加载器
    pluginlib::ClassLoader<uuv_control::DynamicsBase> dyn_loader_;
    pluginlib::ClassLoader<uuv_control::ControllerBase> ctrl_loader_;

    // 插件实例
    boost::shared_ptr<uuv_control::DynamicsBase> dynamics_;
    boost::shared_ptr<uuv_control::ControllerBase> controller_;

    // 发布与广播
    // 删除: ros::Publisher pub_marker_; <-- 不需要了
    ros::Publisher pub_wrench_; 
    ros::Publisher pub_state_;
    ros::Publisher pub_odom_;
    ros::Publisher pub_joint_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // 频率控制参数
    double dynamics_freq_;   // 动力学频率 (Hz)
    double controller_freq_; // 控制器频率 (Hz)
    
    Eigen::VectorXd current_tau_cmd_; // 当前由控制算法（如手柄或APF）期望输出的指令合力
    uuv_control::ControlAllocator allocator_;  // 硬件控制分配器

public:
    UUVControlNode() : 
        nh_(),
        pnh_("~"),
        dyn_loader_("uuv_control", "uuv_control::DynamicsBase"),
        ctrl_loader_("uuv_control", "uuv_control::ControllerBase")
    {
        // 1. 读取频率参数 (默认动力学100Hz, 控制器10Hz)
        pnh_.param("dynamics_frequency", dynamics_freq_, 100.0);
        pnh_.param("controller_frequency", controller_freq_, 10.0);

        loadPlugins();
        
        pub_wrench_ = nh_.advertise<geometry_msgs::Wrench>("debug_wrench", 1);
        pub_state_ = nh_.advertise<uuv_control::State3D>("state", 10);
        pub_odom_  = nh_.advertise<nav_msgs::Odometry>("odom", 10);
        pub_joint_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);
        // 初始化控制力为0
        current_tau_cmd_ = Eigen::VectorXd::Zero(6);
        // 初始化硬件分配器
        allocator_.initialize(nh_, "lauv_dynamics/actuators/");
    }

    void loadPlugins() {
        // 加载动力学插件
        std::string dyn_name;
        pnh_.param<std::string>("dynamics_plugin", dyn_name, "uuv_control/FirstOrderDynamics");
        try {
            dynamics_ = dyn_loader_.createInstance(dyn_name);
            dynamics_->initialize(nh_); 
            ROS_INFO("Dynamics Plugin Loaded: %s at %.1f Hz", dyn_name.c_str(), dynamics_freq_);
        } catch(pluginlib::PluginlibException& ex) {
            ROS_ERROR("Failed to load dynamics: %s", ex.what());
        }

        // 加载控制器插件
        std::string ctrl_name;
        pnh_.param<std::string>("controller_plugin", ctrl_name, "uuv_control/TeleopController");
        try {
            controller_ = ctrl_loader_.createInstance(ctrl_name);
            controller_->initialize(nh_); 
            ROS_INFO("Controller Plugin Loaded: %s at %.1f Hz", ctrl_name.c_str(), controller_freq_);
        } catch(pluginlib::PluginlibException& ex) {
            ROS_ERROR("Failed to load controller: %s", ex.what());
        }
    }

    void run() {
        if (!dynamics_ || !controller_) {
            ROS_ERROR("Plugins not loaded properly, node stopping.");
            return;
        }

        double loop_freq = std::max(dynamics_freq_, controller_freq_);
        ros::Rate rate(loop_freq); // 主循环以最高的动力学频率运行
        double dt_dynamics = 1.0 / dynamics_freq_;
        double dt_controller = 1.0 / controller_freq_;

        ROS_INFO("Simulation Loop running at %.1f Hz (Dynamics: %.1f Hz, Controller: %.1f Hz)", 
                 loop_freq, dynamics_freq_, controller_freq_);

        // 记录上一次执行控制的时间
        ros::Time last_control_time = ros::Time::now();
        ros::Time last_dynamics_time = ros::Time::now();

        while(ros::ok()) {
            ros::Time now = ros::Time::now();

            // === 1. 控制层 ===
            // 检查是否到了执行控制的时间
            if ((now - last_control_time).toSec() >= dt_controller) {
                // 计算新的控制力
                current_tau_cmd_ = controller_->compute();
                // 更新时间戳
                last_control_time = now;
                // 调试输出当前指令
                publishDebugWrench(current_tau_cmd_);
            }
            // === 2. 动力学层更新逻辑 ===
            // 检查是否达到了动力学的触发时间
            double dt_since_last_dyn = (now - last_dynamics_time).toSec();
            if (dt_since_last_dyn >= dt_dynamics) {
                // 2.1 获取当前真实航速和姿态 (用于动压计算)
                uuv_control::State3D state_curr = dynamics_->getState();
                double current_u = state_curr.u;
                
                // 将 6 个速度分量打包成 VectorXd
                Eigen::VectorXd current_nu(6);
                current_nu << state_curr.u, state_curr.v, state_curr.w, 
                              state_curr.p, state_curr.q, state_curr.r;
                // 2.2 ★硬件模拟：分配器将期望力映射为物理舵角与转速 (限幅在此发生)
                uuv_control::ControlAllocator::ActuatorCmd act_cmd = allocator_.allocate(current_tau_cmd_, current_u);
                // 2.3 ★硬件模拟：正向解算，根据此时的物理舵角和速度，产生真实的受力 tau_actual
                Eigen::VectorXd tau_actual = allocator_.computeActualTau(act_cmd, current_nu);
                // 可选：为了监控，发布真实输出的物理受力
                publishDebugWrench(tau_actual);
                // 2.4 ★物理引擎：将真实力矩喂入刚体动力学解算位置
                uuv_control::State3D state = dynamics_->update(dt_since_last_dyn, tau_actual);
                // 2.5 更新可视化
                publishTF(state);
                publishStateAndOdom(state);
                publishJointStates(act_cmd, dt_since_last_dyn); // RViz舵机旋转
                last_dynamics_time = now;
            }
            ros::spinOnce(); 
            rate.sleep();
        }
    }

    void publishJointStates(const uuv_control::ControlAllocator::ActuatorCmd& cmd, double dt) {
        sensor_msgs::JointState joint_msg;
        joint_msg.header.stamp = ros::Time::now();
        
        joint_msg.name = {
            "uuv_0/thruster_0_joint",
            "uuv_0/fin_0_joint",  // 顶舵
            "uuv_0/fin_1_joint",  // 左舵
            "uuv_0/fin_2_joint",  // 底舵
            "uuv_0/fin_3_joint"   // 右舵
        };

        static double prop_angle = 0;
        prop_angle += cmd.omega * dt;

        // 直接填入修复好 URDF 符号陷阱的独立关节角
        joint_msg.position = {
            prop_angle, 
            cmd.fin0_joint, 
            cmd.fin1_joint, 
            cmd.fin2_joint, 
            cmd.fin3_joint  
        };

        pub_joint_.publish(joint_msg);
    }

    void publishStateAndOdom(const uuv_control::State3D& state) {
        ros::Time current_time = ros::Time::now();

        // 1. 发布自定义 State3D 消息
        uuv_control::State3D state_msg = state; 
        pub_state_.publish(state_msg);

        // 2. 发布 Odometry 里程计消息
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "ned";             // 与你的 TF 保持一致的世界坐标系
        odom.child_frame_id = "uuv_0/base_link";  // UUV本体坐标系

        // 设置位置
        odom.pose.pose.position.x = state.x;
        odom.pose.pose.position.y = state.y;
        odom.pose.pose.position.z = state.z;

        // 设置姿态 (欧拉角转四元数)
        tf2::Quaternion q;
        q.setRPY(state.phi, state.theta, state.psi);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        // 设置速度 (机体系下的线速度和角速度)
        odom.twist.twist.linear.x = state.u;
        odom.twist.twist.linear.y = state.v;
        odom.twist.twist.linear.z = state.w;
        odom.twist.twist.angular.x = state.p;
        odom.twist.twist.angular.y = state.q;
        odom.twist.twist.angular.z = state.r;

        pub_odom_.publish(odom);
    }

    void publishTF(const uuv_control::State3D& state) {
        geometry_msgs::TransformStamped t;
        t.header.stamp = ros::Time::now();
        t.header.frame_id = "ned";            // 世界坐标系
        t.child_frame_id = "uuv_0/base_link"; // 机器人基体坐标系
        
        t.transform.translation.x = state.x;
        t.transform.translation.y = state.y;
        t.transform.translation.z = state.z;
        
        tf2::Quaternion q;
        q.setRPY(state.phi, state.theta, state.psi);
        t.transform.rotation.x = q.x(); t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z(); t.transform.rotation.w = q.w();
        
        tf_broadcaster_.sendTransform(t);
    }

    void publishDebugWrench(const Eigen::VectorXd& tau) {
        geometry_msgs::Wrench msg;
        msg.force.x = tau(0); msg.force.y = tau(1); msg.force.z = tau(2);
        msg.torque.x = tau(3); msg.torque.y = tau(4); msg.torque.z = tau(5);
        pub_wrench_.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "uuv_control_node");
    UUVControlNode node;
    node.run();
    return 0;
}