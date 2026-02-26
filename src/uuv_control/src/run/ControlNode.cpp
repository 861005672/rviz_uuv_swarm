#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <uuv_interface/DynamicsBase.h>
#include <uuv_interface/ControllerBase.h>
#include <uuv_interface/GuidanceBase.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <uuv_interface/SensorBase.h>
#include <uuv_interface/Cmd3D.h>
#include <tinyxml.h>

class UUVControlNode {
private:
    ros::NodeHandle gnh_;
    std::string robot_description_;
    
    // 插件加载器
    pluginlib::ClassLoader<uuv_interface::DynamicsBase> dyn_loader_;
    pluginlib::ClassLoader<uuv_interface::ControllerBase> ctrl_loader_;
    pluginlib::ClassLoader<uuv_interface::SensorBase> sensor_loader_;
    pluginlib::ClassLoader<uuv_interface::GuidanceBase> guidance_loader_;

    // 插件实例
    boost::shared_ptr<uuv_interface::DynamicsBase> dynamics_;
    boost::shared_ptr<uuv_interface::ControllerBase> controller_;
    boost::shared_ptr<uuv_interface::GuidanceBase> guidance_;
    std::vector<boost::shared_ptr<uuv_interface::SensorBase>> sensors_;

    // 频率参数
    double dynamics_freq_;   // 动力学频率 (Hz)
    double controller_freq_; // 控制器频率 (Hz)
    double guidance_freq_ = 10.0;

    // 发布与广播
    ros::Publisher pub_state_;
    ros::Publisher pub_odom_;
    ros::Publisher pub_joint_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // 当前由控制算法（如手柄或APF）期望输出的指令合力
    Eigen::VectorXd current_tau_cmd_; 

    uuv_interface::State3D current_state_;

public:
    UUVControlNode() : 
        gnh_(),
        dyn_loader_("uuv_interface", "uuv_interface::DynamicsBase"),
        ctrl_loader_("uuv_interface", "uuv_interface::ControllerBase"),
        sensor_loader_("uuv_interface", "uuv_interface::SensorBase"),
        guidance_loader_("uuv_interface", "uuv_interface::GuidanceBase")
    {
        loadPluginsFromXML();
        
        pub_state_ = gnh_.advertise<uuv_interface::State3D>("state", 10);
        pub_odom_  = gnh_.advertise<nav_msgs::Odometry>("odom", 10);
        current_tau_cmd_ = Eigen::VectorXd::Zero(6);    // 初始化控制力为0
    }

    void loadPluginsFromXML() {
        if (!gnh_.getParam("robot_description", robot_description_)) {
            ROS_ERROR("[ControlNode] Cannot find 'robot_description' parameter! Node stopping.");
            return;
        }

        TiXmlDocument doc;
        doc.Parse(robot_description_.c_str());
        if (doc.Error()) { ROS_ERROR("[ControlNode] Failed to parse robot_description XML!"); return; }

        TiXmlElement* root = doc.RootElement();
        if (!root) { ROS_ERROR("[ControlNode] Not found <root> in robot_description XML!"); return; }

        // 统一遍历所有名叫 <plugin> 的标签
        for (TiXmlElement* elem = root->FirstChildElement("plugin"); 
            elem != nullptr; 
            elem = elem->NextSiblingElement("plugin")) {
             
            // 1. 获取 layer, name 和 type 三大核心属性
            std::string layer = elem->Attribute("layer") ? elem->Attribute("layer") : "";
            std::string name  = elem->Attribute("name")  ? elem->Attribute("name")  : "unknown_name";
            std::string type  = elem->Attribute("type")  ? elem->Attribute("type")  : "";
            if (layer.empty() || type.empty()) {
                ROS_WARN("[ControlNode] Plugin '%s' is missing 'layer' or 'type'. Skipping.", name.c_str());
                continue;
            }
            // 2. 提取当前标签专属的 XML 切片
            TiXmlPrinter printer;
            elem->Accept(&printer);
            std::string snippet = printer.CStr();

            // 3. 根据 layer 路由到对应的加载器，下发切片并完成实例化！
            try {
                if (layer == "dynamics") {
                    dynamics_ = dyn_loader_.createInstance(type);
                    dynamics_->initialize(gnh_, snippet);  
                    dynamics_freq_ = dynamics_->get_rate();
                    ROS_INFO("[ControlNode] Loaded Dynamics : '%s' [%s] @ %.1f Hz", name.c_str(), type.c_str(), dynamics_freq_);
                } 
                else if (layer == "controller") {
                    controller_ = ctrl_loader_.createInstance(type);
                    controller_->initialize(gnh_, snippet);
                    controller_freq_ = controller_->get_rate();
                    ROS_INFO("[ControlNode] Loaded Controller: '%s' [%s] @ %.1f Hz", name.c_str(), type.c_str(), controller_freq_);
                } 
                else if (layer == "guidance") { // 3. 增加制导层的 XML 解析分支
                    guidance_ = guidance_loader_.createInstance(type);
                    guidance_->initialize(gnh_, snippet);
                    guidance_freq_ = guidance_->get_rate();
                    ROS_INFO("[ControlNode] Loaded Guidance  : '%s' [%s] @ %.1f Hz", name.c_str(), type.c_str(), guidance_freq_);
                }
                else if (layer == "sensor") {
                    auto sensor = sensor_loader_.createInstance(type);
                    sensor->initialize(gnh_, snippet);
                    sensors_.push_back(sensor);
                    ROS_INFO("[ControlNode] Loaded Sensor    : '%s' [%s]", name.c_str(), type.c_str());
                } 
                else {
                    ROS_WARN("[ControlNode] Unknown layer '%s' for plugin '%s'", layer.c_str(), name.c_str());
                }
            } catch(pluginlib::PluginlibException& ex) {
                ROS_ERROR("[ControlNode] Failed to load plugin '%s': %s", name.c_str(), ex.what());
            }
        }
    }

    void run() {
        if (!dynamics_ || !controller_ || !guidance_) {
            ROS_ERROR("[ControlNode] Plugins not loaded properly, node stopping.");
            return;
        }

        double loop_freq = std::max({dynamics_freq_, controller_freq_, guidance_freq_, 100.0});
        ros::Rate rate(loop_freq);

        ROS_INFO("[ControlNode] ControlNode Loop running at %.1f Hz (Plugins will self-regulate)", loop_freq);

        while(ros::ok()) {
            // === 1. 制导层 ===
            uuv_interface::Cmd3D g_out = guidance_->compute();
            controller_->setCommand(g_out);
            // === 2. 控制层 ===
            current_tau_cmd_ = controller_->compute();
            // === 3. 动力学层更新逻辑 ===
            current_state_ = dynamics_->update(current_tau_cmd_);
            // === 4. 更新传感器插件 === 
            updateSensor();
            // === 4. 更新可视化 ===Z
            publishTF(current_state_);
            publishStateAndOdom(current_state_);
            ros::spinOnce(); 
            rate.sleep();
        }
    }

    void updateSensor() {
        uuv_interface::SensorState s_state;
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

    void publishStateAndOdom(const uuv_interface::State3D& state) {
        ros::Time current_time = ros::Time::now();

        // 1. 发布自定义 State3D 消息
        uuv_interface::State3D state_msg = state;
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

    void publishTF(const uuv_interface::State3D& state) {
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