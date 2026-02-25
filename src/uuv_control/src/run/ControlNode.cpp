#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <uuv_control/interface/DynamicsBase.h>
#include <uuv_control/interface/ControllerBase.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <uuv_sensor/interface/SensorBase.h>

class UUVControlNode {
private:
    ros::NodeHandle gnh_;

    // 插件名
    std::string dynamics_plugin_name_;    // 动力学模型插件名称
    std::string controller_plugin_name_;  // 控制器插件名称
    std::vector<std::string> sensor_plugin_names_;    // 所有的传感器插件名称
    
    // 插件加载器
    pluginlib::ClassLoader<uuv_control::DynamicsBase> dyn_loader_;
    pluginlib::ClassLoader<uuv_control::ControllerBase> ctrl_loader_;
    pluginlib::ClassLoader<uuv_sensor::SensorBase> sensor_loader_;

    // 插件实例
    boost::shared_ptr<uuv_control::DynamicsBase> dynamics_;
    boost::shared_ptr<uuv_control::ControllerBase> controller_;
    std::vector<boost::shared_ptr<uuv_sensor::SensorBase>> sensors_;

    // 频率参数
    double dynamics_freq_;   // 动力学频率 (Hz)
    double controller_freq_; // 控制器频率 (Hz)

    // 发布与广播
    ros::Publisher pub_state_;
    ros::Publisher pub_odom_;
    ros::Publisher pub_joint_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // 当前由控制算法（如手柄或APF）期望输出的指令合力
    Eigen::VectorXd current_tau_cmd_; 

    uuv_control::State3D current_state_;

public:
    UUVControlNode() : 
        gnh_(),
        dyn_loader_("uuv_control", "uuv_control::DynamicsBase"),
        ctrl_loader_("uuv_control", "uuv_control::ControllerBase"),
        sensor_loader_("uuv_sensor", "uuv_sensor::SensorBase")
    {
        loadParameters();    // 载入参数
        loadPlugins();       // 载入插件
        
        pub_state_ = gnh_.advertise<uuv_control::State3D>("state", 10);
        pub_odom_  = gnh_.advertise<nav_msgs::Odometry>("odom", 10);
    
        current_tau_cmd_ = Eigen::VectorXd::Zero(6);    // 初始化控制力为0
    }

    void loadParameters() {
        std::string ns = "/";
        gnh_.param<std::string>(ns+"dynamics_plugin", dynamics_plugin_name_, "uuv_control/FossenDynamics");
        gnh_.param<std::string>(ns+"controller_plugin", controller_plugin_name_, "uuv_control/PidController");
        XmlRpc::XmlRpcValue sensor_list;
        if (gnh_.getParam(ns+"sensor_plugins", sensor_list) && sensor_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for (int i = 0; i < sensor_list.size(); ++i) {
                sensor_plugin_names_.push_back(static_cast<std::string>(sensor_list[i]));
            }
        }
        gnh_.param(ns+"dynamics_frequency", dynamics_freq_, 100.0);
        gnh_.param(ns+"controller_frequency", controller_freq_, 10.0);
        ROS_INFO_STREAM("[ControlNode] Parameter Loaded: dynamics_plugin=\n"<<dynamics_plugin_name_<<" \ncontroller_plugin=\n"<<controller_plugin_name_
        <<" \ndynamics_frequency=\n"<<dynamics_freq_<<" \ncontroller_frequency=\n"<<controller_freq_);
    }

    void loadPlugins() {
        // 加载动力学插件
        try {
            dynamics_ = dyn_loader_.createInstance(dynamics_plugin_name_);
            dynamics_->initialize(gnh_); 
            ROS_INFO("[ControlNode] Dynamics Plugin Loaded.");
        } catch(pluginlib::PluginlibException& ex) {
            ROS_ERROR("[ControlNode] Failed to load dynamics: %s", ex.what());
        }
        // 加载控制器插件
        try {
            controller_ = ctrl_loader_.createInstance(controller_plugin_name_);
            controller_->initialize(gnh_); 
            ROS_INFO("[ControlNode] Controller Plugin Loaded.");
        } catch(pluginlib::PluginlibException& ex) {
            ROS_ERROR("[ControlNode] Failed to load controller: %s", ex.what());
        }
        // 加载传感器插件
        for (const auto& plugin_name : sensor_plugin_names_) {
            try {
                auto sensor = sensor_loader_.createInstance(plugin_name);
                sensor->initialize(gnh_); 
                sensors_.push_back(sensor);
                ROS_INFO("[ControlNode] Sensor Plugin Loaded: %s", plugin_name.c_str());
            } catch(pluginlib::PluginlibException& ex) {
                ROS_ERROR("[ControlNode] Failed to load sensor plugin %s: %s", plugin_name.c_str(), ex.what());
            }
        }
    }

    void run() {
        if (!dynamics_ || !controller_) {
            ROS_ERROR("[ControlNode] Plugins not loaded properly, node stopping.");
            return;
        }

        double loop_freq = std::max(dynamics_freq_, controller_freq_);
        ros::Rate rate(loop_freq); // 主循环以最高的动力学频率运行
        double dt_dynamics = 1.0 / dynamics_freq_;
        double dt_controller = 1.0 / controller_freq_;

        ROS_INFO("[ControlNode] Simulation Loop running at %.1f Hz (Dynamics: %.1f Hz, Controller: %.1f Hz)", 
                 loop_freq, dynamics_freq_, controller_freq_);

        // 记录上一次执行控制的时间
        ros::Time last_control_time = ros::Time::now();
        ros::Time last_dynamics_time = ros::Time::now();

        while(ros::ok()) {
            ros::Time now = ros::Time::now();

            // === 1. 控制层 ===
            // 检查是否到了执行控制的时间
            double dt_since_last_control = (now - last_control_time).toSec();
            if (dt_since_last_control >= dt_controller*0.95) {
                // 计算新的控制力
                current_tau_cmd_ = controller_->compute();
                // 更新时间戳
                last_control_time = now;
            }
            // === 2. 动力学层更新逻辑 ===
            // 检查是否达到了动力学的触发时间
            double dt_since_last_dyn = (now - last_dynamics_time).toSec();
            if (dt_since_last_dyn >= dt_dynamics*0.95) {
                // 将期望力/力矩（控制输入）喂入Dynamics层，由动力学模型和执行器模型解算位置
                current_state_ = dynamics_->update(dt_since_last_dyn, current_tau_cmd_);
                last_dynamics_time = now;
            }
            // === 3. 更新传感器插件 === 
            updateSensor();
            
            // === 4. 更新可视化 ===Z
            publishTF(current_state_);
            publishStateAndOdom(current_state_);

            ros::spinOnce(); 
            rate.sleep();
        }
    }

    void updateSensor() {
        uuv_sensor::SensorState s_state;
        s_state.x = current_state_.x;
        s_state.y = current_state_.y;
        s_state.z = current_state_.z;
        s_state.roll = current_state_.roll;
        s_state.pitch = current_state_.pitch;
        s_state.yaw = current_state_.yaw;
        s_state.u = current_state_.u;
        s_state.v = current_state_.v;
        s_state.w = current_state_.w;
        s_state.p = current_state_.p;
        s_state.q = current_state_.q;
        s_state.r = current_state_.r;
        for (auto& sensor : sensors_) {
            sensor->update(s_state);
        }
    }

    void publishStateAndOdom(const uuv_control::State3D& state) {
        ros::Time current_time = ros::Time::now();

        // 1. 发布自定义 State3D 消息
        uuv_control::State3D state_msg = state;
        state_msg.header.stamp = current_time; 
        state_msg.header.frame_id = "ned";
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
        q.setRPY(state.roll, state.pitch, state.yaw);
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
        t.header.stamp = ros::Time::now()-ros::Duration(0.02);    // 减小0.02s，用于补偿UUV主体的TF发布太快，导致主体和舵、推进器不同步问题
        t.header.frame_id = "ned";            // 世界坐标系
        t.child_frame_id = "uuv_0/base_link"; // 机器人基体坐标系
        
        t.transform.translation.x = state.x;
        t.transform.translation.y = state.y;
        t.transform.translation.z = state.z;
        
        tf2::Quaternion q;
        q.setRPY(state.roll, state.pitch, state.yaw);
        t.transform.rotation.x = q.x(); t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z(); t.transform.rotation.w = q.w();
        
        tf_broadcaster_.sendTransform(t);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "uuv_control_node");
    UUVControlNode node;
    node.run();
    return 0;
}