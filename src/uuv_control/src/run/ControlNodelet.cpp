#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
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
#include <memory>

namespace uuv_control {

class UUVControlNodelet : public nodelet::Nodelet {
private:
    ros::NodeHandle gnh_;
    ros::NodeHandle pnh_;
    std::string robot_description_;
    
    // 插件加载器 (使用智能指针延迟实例化，避免静态销毁问题)
    std::unique_ptr<pluginlib::ClassLoader<uuv_interface::DynamicsBase>> dyn_loader_;
    std::unique_ptr<pluginlib::ClassLoader<uuv_interface::ControllerBase>> ctrl_loader_;
    std::unique_ptr<pluginlib::ClassLoader<uuv_interface::SensorBase>> sensor_loader_;
    std::unique_ptr<pluginlib::ClassLoader<uuv_interface::GuidanceBase>> guidance_loader_;

    // 插件实例
    boost::shared_ptr<uuv_interface::DynamicsBase> dynamics_;
    boost::shared_ptr<uuv_interface::ControllerBase> controller_;
    boost::shared_ptr<uuv_interface::GuidanceBase> guidance_;
    std::vector<boost::shared_ptr<uuv_interface::SensorBase>> sensors_;

    // 频率与定时器
    double dynamics_freq_;   
    double controller_freq_; 
    double guidance_freq_ = 10.0;
    ros::Timer control_timer_; // 替代原有的 while 循环

    // 发布与广播
    ros::Publisher pub_state_;
    ros::Publisher pub_odom_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 命名空间与坐标系参数
    std::string uuv_ns_;
    std::string base_frame_;
    std::string world_frame_;
    Eigen::VectorXd current_tau_cmd_; 
    uuv_interface::State3D current_state_;

    double visual_rate_;
    double visual_period_;
    ros::Time last_visual_time_;

public:
    UUVControlNodelet() {}

    virtual void onInit() override {
        gnh_ = getNodeHandle();
        pnh_ = getPrivateNodeHandle();

        // 读取组名和id
        std::string ns = gnh_.getNamespace();
        if (!ns.empty() && ns.front() == '/') {
            uuv_ns_ = ns.substr(1); // 剔除开头的斜杠，得到 "groupA/0"
        } else {
            uuv_ns_ = ns;
        }
        // 允许单独配置世界坐标系，默认保持为 "ned"
        pnh_.param<std::string>("world_frame", world_frame_, "ned");
        double init_x = 0.0, init_y = 0.0, init_z = 0.0, init_yaw = 0.0;
        pnh_.param("init_x", init_x, 0.0);
        pnh_.param("init_y", init_y, 0.0);
        pnh_.param("init_z", init_z, 0.0);
        pnh_.param("init_yaw", init_yaw, 0.0);

        pnh_.param<double>("visual_rate", visual_rate_, 20.0);
        // 组合UUV的命名空间
        // 自动推导该 UUV 的基座标系
        base_frame_ = uuv_ns_ + "/base_link";

        visual_period_ = 1.0 / visual_rate_;
        last_visual_time_ = ros::Time(0);

        // 实例化加载器和广播器
        dyn_loader_.reset(new pluginlib::ClassLoader<uuv_interface::DynamicsBase>("uuv_interface", "uuv_interface::DynamicsBase"));
        ctrl_loader_.reset(new pluginlib::ClassLoader<uuv_interface::ControllerBase>("uuv_interface", "uuv_interface::ControllerBase"));
        sensor_loader_.reset(new pluginlib::ClassLoader<uuv_interface::SensorBase>("uuv_interface", "uuv_interface::SensorBase"));
        guidance_loader_.reset(new pluginlib::ClassLoader<uuv_interface::GuidanceBase>("uuv_interface", "uuv_interface::GuidanceBase"));
        tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster());
        current_tau_cmd_ = Eigen::VectorXd::Zero(6);
        
        pub_state_ = gnh_.advertise<uuv_interface::State3D>("state", 10);
        pub_odom_  = gnh_.advertise<nav_msgs::Odometry>("odom", 10);

        loadPluginsFromXML();

        if (!dynamics_ || !controller_ || !guidance_) {
            NODELET_ERROR("[ControlNodelet] Plugins not loaded properly, nodelet stopping.");
            return;
        }

        double loop_freq = std::max({dynamics_freq_, controller_freq_, guidance_freq_, 100.0});
        NODELET_INFO("[ControlNodelet] Loop running at %.1f Hz via Timer", loop_freq);

        // 使用 Timer 替代原有的阻塞式死循环
        control_timer_ = gnh_.createTimer(ros::Duration(1.0 / loop_freq), &UUVControlNodelet::timerCallback, this);

        // 解析初始状态
        uuv_interface::State3D init_state;
        init_state.x = init_x; init_state.y = init_y; init_state.z = init_z;
        init_state.roll = 0.0; init_state.pitch = 0.0; init_state.yaw = init_yaw;
        init_state.u = 0.0; init_state.v = 0.0; init_state.w = 0.0;
        init_state.p = 0.0; init_state.q = 0.0; init_state.r = 0.0;
        dynamics_->setState(init_state);
        current_state_ = dynamics_->getState(); // 同步状态
    }

    void loadPluginsFromXML() {
        if (!gnh_.getParam("robot_description", robot_description_)) {
            NODELET_ERROR("[ControlNodelet] Cannot find 'robot_description' parameter!");
            return;
        }

        TiXmlDocument doc;
        doc.Parse(robot_description_.c_str());
        if (doc.Error()) { NODELET_ERROR("[ControlNodelet] Failed to parse robot_description XML!"); return; }

        TiXmlElement* root = doc.RootElement();
        if (!root) { NODELET_ERROR("[ControlNodelet] Not found <root> in robot_description XML!"); return; }

        for (TiXmlElement* elem = root->FirstChildElement("plugin"); elem != nullptr; elem = elem->NextSiblingElement("plugin")) {
            std::string layer = elem->Attribute("layer") ? elem->Attribute("layer") : "";
            std::string name  = elem->Attribute("name")  ? elem->Attribute("name")  : "unknown_name";
            std::string type  = elem->Attribute("type")  ? elem->Attribute("type")  : "";
            
            if (layer.empty() || type.empty()) {
                NODELET_WARN("[ControlNodelet] Plugin '%s' is missing 'layer' or 'type'. Skipping.", name.c_str());
                continue;
            }

            TiXmlPrinter printer;
            elem->Accept(&printer);
            std::string snippet = printer.CStr();

            try {
                if (layer == "dynamics") {
                    dynamics_ = dyn_loader_->createInstance(type);
                    dynamics_->initialize(gnh_, snippet);  
                    dynamics_freq_ = dynamics_->get_rate();
                    NODELET_INFO("[ControlNodelet] Loaded Dynamics : '%s' [%s] @ %.1f Hz", name.c_str(), type.c_str(), dynamics_freq_);
                } 
                else if (layer == "controller") {
                    controller_ = ctrl_loader_->createInstance(type);
                    controller_->initialize(gnh_, snippet);
                    controller_freq_ = controller_->get_rate();
                    NODELET_INFO("[ControlNodelet] Loaded Controller: '%s' [%s] @ %.1f Hz", name.c_str(), type.c_str(), controller_freq_);
                } 
                else if (layer == "guidance") {
                    guidance_ = guidance_loader_->createInstance(type);
                    guidance_->initialize(gnh_, snippet);
                    guidance_freq_ = guidance_->get_rate();
                    NODELET_INFO("[ControlNodelet] Loaded Guidance  : '%s' [%s] @ %.1f Hz", name.c_str(), type.c_str(), guidance_freq_);
                }
                else if (layer == "sensor") {
                    auto sensor = sensor_loader_->createInstance(type);
                    sensor->initialize(gnh_, snippet);
                    sensors_.push_back(sensor);
                    NODELET_INFO("[ControlNodelet] Loaded Sensor    : '%s' [%s]", name.c_str(), type.c_str());
                } 
                else {
                    NODELET_WARN("[ControlNodelet] Unknown layer '%s' for plugin '%s'", layer.c_str(), name.c_str());
                }
            } catch(pluginlib::PluginlibException& ex) {
                NODELET_ERROR("[ControlNodelet] Failed to load plugin '%s': %s", name.c_str(), ex.what());
            }
        }
    }

    // 定时器回调函数（替代原有的 run() 循环）
    void timerCallback(const ros::TimerEvent& event) {
        ros::Time current_time = ros::Time::now();
        if (last_visual_time_.isZero()) {
            last_visual_time_ = current_time;
        }
        // === 1. 制导层 ===
        uuv_interface::Cmd3D g_out = guidance_->compute();
        controller_->setCommand(g_out);
        // === 2. 控制层 ===
        current_tau_cmd_ = controller_->compute();
        // === 3. 动力学层 ===
        current_state_ = dynamics_->update(current_tau_cmd_, current_time);
        // === 4. 更新传感器 === 
        updateSensor();

        publishState(current_state_, current_time);
        // === 5. 更新可视化 ===
        if ((current_time - last_visual_time_).toSec() >= visual_period_ * 0.95) {
            publishTF(current_state_, current_time);
            publishOdom(current_state_, current_time);
            dynamics_->publishVisuals(current_time);
            last_visual_time_ = current_time;
        }
    }

    void updateSensor() {
        uuv_interface::SensorState s_state;
        s_state.x = current_state_.x; s_state.y = current_state_.y; s_state.z = current_state_.z;
        s_state.roll = current_state_.roll; s_state.pitch = current_state_.pitch; s_state.yaw = current_state_.yaw;
        s_state.u = current_state_.u; s_state.v = current_state_.v; s_state.w = current_state_.w;
        s_state.p = current_state_.p; s_state.q = current_state_.q; s_state.r = current_state_.r;
        for (auto& sensor : sensors_) {
            sensor->update(s_state);
        }
    }

    void publishState(const uuv_interface::State3D& state, const ros::Time& time) {
        uuv_interface::State3D state_msg = state;
        state_msg.header.stamp = time; 
        state_msg.header.frame_id = world_frame_;
        pub_state_.publish(state_msg);
    }

    void publishOdom(const uuv_interface::State3D& state, const ros::Time& time) {
        nav_msgs::Odometry odom;
        odom.header.stamp = time;
        odom.header.frame_id = world_frame_;             
        odom.child_frame_id = base_frame_;  

        odom.pose.pose.position.x = state.x;
        odom.pose.pose.position.y = state.y;
        odom.pose.pose.position.z = state.z;

        tf2::Quaternion q;
        q.setRPY(state.roll, state.pitch, state.yaw);
        odom.pose.pose.orientation.x = q.x(); odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z(); odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x = state.u; odom.twist.twist.linear.y = state.v; odom.twist.twist.linear.z = state.w;
        odom.twist.twist.angular.x = state.p; odom.twist.twist.angular.y = state.q; odom.twist.twist.angular.z = state.r;

        pub_odom_.publish(odom);
    }

    void publishTF(const uuv_interface::State3D& state, const ros::Time& time) {
        geometry_msgs::TransformStamped t;
        t.header.stamp = time - ros::Duration(visual_period_); 
        t.header.frame_id = world_frame_;            
        t.child_frame_id = base_frame_; 
        
        t.transform.translation.x = state.x;
        t.transform.translation.y = state.y;
        t.transform.translation.z = state.z;
        
        tf2::Quaternion q;
        q.setRPY(state.roll, state.pitch, state.yaw);
        t.transform.rotation.x = q.x(); t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z(); t.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(t);
    }
};

} // namespace uuv_control

// 导出 Nodelet 插件
PLUGINLIB_EXPORT_CLASS(uuv_control::UUVControlNodelet, nodelet::Nodelet)