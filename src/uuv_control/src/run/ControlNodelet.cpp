#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>
#include <uuv_interface/DynamicsBase.h>
#include <uuv_interface/ControllerBase.h>
#include <uuv_interface/GuidanceBase.h>
#include <uuv_interface/ActuatorBase.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <uuv_interface/SensorPluginBase.h>
#include <uuv_interface/Cmd3D.h>
#include <uuv_interface/TargetPoint3D.h>
#include <tinyxml.h>
#include <memory>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <uuv_interface/utils/UUVLogger.h>

namespace uuv_control {

class UUVControlNodelet : public nodelet::Nodelet {
private:
    ros::NodeHandle gnh_;
    std::string robot_description_;
    std::shared_ptr<uuv_interface::UUVLogger> uuv_logger_;
    
    // 插件加载器 (使用智能指针延迟实例化，避免静态销毁问题)
    std::unique_ptr<pluginlib::ClassLoader<uuv_interface::DynamicsBase>> dyn_loader_;
    std::unique_ptr<pluginlib::ClassLoader<uuv_interface::ActuatorBase>> act_loader_;
    std::unique_ptr<pluginlib::ClassLoader<uuv_interface::ControllerBase>> ctrl_loader_;
    std::unique_ptr<pluginlib::ClassLoader<uuv_interface::GuidanceBase>> guidance_loader_;
    std::unique_ptr<pluginlib::ClassLoader<uuv_interface::SensorPluginBase>> sensor_loader_;

    // 插件实例
    boost::shared_ptr<uuv_interface::DynamicsBase> dynamics_;
    boost::shared_ptr<uuv_interface::ActuatorBase> actuator_;
    boost::shared_ptr<uuv_interface::ControllerBase> controller_;
    boost::shared_ptr<uuv_interface::GuidanceBase> guidance_;
    std::vector<boost::shared_ptr<uuv_interface::SensorPluginBase>> sensors_;

    // 频率与定时器
    double dynamics_freq_;   
    double actuator_freq_;   
    double controller_freq_; 
    double guidance_freq_ = 10.0;
    ros::Timer control_timer_; // 替代原有的 while 循环

    // 发布与广播
    ros::Publisher pub_state_;
    ros::Publisher pub_trajectory_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 命名空间与坐标系参数
    std::string uuv_ns_;
    std::string base_frame_;
    std::string world_frame_;
    uuv_interface::State3D current_state_;

    nav_msgs::Path uuv_trajectory_;
    int max_trajectory_length_ =1000;
    double trajectory_seg_length_ = 1;

    double visual_rate_;
    double visual_period_;
    ros::Time last_visual_time_;

public:
    UUVControlNodelet() {}

    virtual void onInit() override {
        gnh_ = getNodeHandle();

        // 读取组名和id
        std::string ns = gnh_.getNamespace();
        if (!ns.empty() && ns.front() == '/') {
            uuv_ns_ = ns.substr(1); // 剔除开头的斜杠，得到 "groupA/0"
        } else {
            uuv_ns_ = ns;
        }

        std::string log_dir;
        gnh_.param<std::string>("log_dir", log_dir, "/tmp");
        std::string log_filename = log_dir + "/" + uuv_ns_ + ".log";
        uuv_logger_ = std::make_shared<uuv_interface::UUVLogger>(log_filename);
        UUV_INFO << "===========================================";
        UUV_INFO << uuv_ns_ << " Control Nodelet Initializing...";
        UUV_INFO << "===========================================";
        // 允许单独配置世界坐标系，默认保持为 "ned"
        gnh_.param<std::string>("world_frame", world_frame_, "ned");
        double init_x = 0.0, init_y = 0.0, init_z = 0.0, init_yaw = 0.0;
        gnh_.param("init_x", init_x, 0.0);
        gnh_.param("init_y", init_y, 0.0);
        gnh_.param("init_z", init_z, 0.0);
        gnh_.param("init_yaw", init_yaw, 0.0);
        gnh_.param("max_trajectory_length", max_trajectory_length_, 1000);
        gnh_.param("trajectory_seg_length", trajectory_seg_length_, 1.0);
        gnh_.param<double>("visual_rate", visual_rate_, 20.0);
        // 组合UUV的命名空间
        // 自动推导该 UUV 的基座标系
        base_frame_ = uuv_ns_ + "/base_link";

        visual_period_ = 1.0 / visual_rate_;
        last_visual_time_ = ros::Time(0);

        // 实例化加载器和广播器
        dyn_loader_.reset(new pluginlib::ClassLoader<uuv_interface::DynamicsBase>("uuv_interface", "uuv_interface::DynamicsBase"));
        act_loader_.reset(new pluginlib::ClassLoader<uuv_interface::ActuatorBase>("uuv_interface", "uuv_interface::ActuatorBase"));
        ctrl_loader_.reset(new pluginlib::ClassLoader<uuv_interface::ControllerBase>("uuv_interface", "uuv_interface::ControllerBase"));
        sensor_loader_.reset(new pluginlib::ClassLoader<uuv_interface::SensorPluginBase>("uuv_interface", "uuv_interface::SensorPluginBase"));
        guidance_loader_.reset(new pluginlib::ClassLoader<uuv_interface::GuidanceBase>("uuv_interface", "uuv_interface::GuidanceBase"));
        tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster());
        
        pub_state_ = gnh_.advertise<uuv_interface::State3D>("state", 10);
        pub_trajectory_  = gnh_.advertise<nav_msgs::Path>("trajectory", 10);
        uuv_trajectory_.header.frame_id = "ned";

        loadPluginsFromXML();

        // if (!dynamics_ || !controller_ || !guidance_) {
        //     UUV_ERROR << "[ControlNodelet] Plugins not loaded properly, nodelet stopping.";
        //     return;
        // }

        double loop_freq = std::max({dynamics_freq_, controller_freq_, guidance_freq_, actuator_freq_, 100.0});
        if (loop_freq > 100) loop_freq = 100.0;
        UUV_INFO << "[ControlNodelet] Loop running at " << loop_freq << " Hz via Timer";

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
            ROS_ERROR("[ControlNodelet] CRITICAL: Cannot find 'robot_description' parameter! Check your Xacro!");
            UUV_ERROR << "[ControlNodelet] Cannot find 'robot_description' parameter!";
            return;
        }

        TiXmlDocument doc;
        doc.Parse(robot_description_.c_str());
        if (doc.Error()) { UUV_ERROR << "[ControlNodelet] Failed to parse robot_description XML!"; return; }

        TiXmlElement* root = doc.RootElement();
        if (!root) { UUV_ERROR << "[ControlNodelet] Not found <root> in robot_description XML!"; return; }

        for (TiXmlElement* elem = root->FirstChildElement("plugin"); elem != nullptr; elem = elem->NextSiblingElement("plugin")) {
            std::string layer = elem->Attribute("layer") ? elem->Attribute("layer") : "";
            std::string name  = elem->Attribute("name")  ? elem->Attribute("name")  : "unknown_name";
            std::string type  = elem->Attribute("type")  ? elem->Attribute("type")  : "";
            
            if (layer.empty() || type.empty()) {
                UUV_WARN << "[ControlNodelet] Plugin "<< name.c_str() <<" is missing 'layer' or 'type'. Skipping.";
                continue;
            }

            TiXmlPrinter printer;
            elem->Accept(&printer);
            std::string snippet = printer.CStr();

            try {
                if (layer == "dynamics") {
                    dynamics_ = dyn_loader_->createInstance(type);
                    dynamics_->setLogger(uuv_logger_);
                    dynamics_->initialize(gnh_, snippet, "dynamics");  
                    dynamics_freq_ = dynamics_->get_rate();
                    UUV_INFO << "[ControlNodelet] Loaded Dynamics : "<< name.c_str() <<  "[" << type.c_str() << "] @ " << dynamics_freq_ << " Hz";
                } else if (layer == "actuator") {
                    actuator_ = act_loader_->createInstance(type);
                    actuator_->setLogger(uuv_logger_);
                    actuator_->initialize(gnh_, snippet, "actuator");  
                    actuator_freq_ = actuator_->get_rate();
                    UUV_INFO << "[ControlNodelet] Loaded Actuator : "<< name.c_str() <<  "[" << type.c_str() << "] @ " << actuator_freq_ << " Hz";
                }
                else if (layer == "controller") {
                    controller_ = ctrl_loader_->createInstance(type);
                    controller_->setLogger(uuv_logger_);
                    controller_->initialize(gnh_, snippet, "controller");
                    controller_freq_ = controller_->get_rate();
                    UUV_INFO << "[ControlNodelet] Loaded Controller : "<< name.c_str() <<  "[" << type.c_str() << "] @ " << controller_freq_ << " Hz";
                } 
                else if (layer == "guidance") {
                    guidance_ = guidance_loader_->createInstance(type);
                    guidance_->setLogger(uuv_logger_);
                    guidance_->initialize(gnh_, snippet, "guidance");
                    guidance_freq_ = guidance_->get_rate();
                    UUV_INFO << "[ControlNodelet] Loaded Guidance : "<< name.c_str() <<  "[" << type.c_str() << "] @ " << guidance_freq_ << " Hz";
                }
                else if (layer == "sensor") {
                    auto sensor = sensor_loader_->createInstance(type);
                    sensor->setLogger(uuv_logger_);
                    sensor->initialize(gnh_, snippet);
                    sensors_.push_back(sensor);
                    double sensor_freq = sensor->get_rate();
                    UUV_INFO << "[ControlNodelet] Loaded Sensor : "<< name.c_str() <<  "[" << type.c_str() << "] @ " << sensor_freq << " Hz";
                } 
                else {
                    UUV_INFO << "[ControlNodelet] Unknown layer : "<< layer.c_str() <<  " for plugin " << name.c_str();

                }
            } catch(pluginlib::PluginlibException& ex) {
                ROS_ERROR_STREAM("[ControlNodelet] Failed to load plugin " << name << " of type " << type << ". ERROR: " << ex.what());
                UUV_ERROR << "[ControlNodelet] Failed to load plugin " << name.c_str() << ": " << ex.what();
            }
        }
    }

    // 定时器回调函数（替代原有的 run() 循环）
    void timerCallback(const ros::TimerEvent& event) {
        ros::Time current_time = ros::Time::now();
        if (last_visual_time_.isZero()) {
            last_visual_time_ = current_time;
        }
        
        // 目标点默认为自身位置，相当于默认不行动
        uuv_interface::TargetPoint3D dummy_tgt;
        // dummy_tgt.n = current_state_.x;
        // dummy_tgt.e = current_state_.y;
        // dummy_tgt.d = current_state_.z;
        dummy_tgt.n = 100;
        dummy_tgt.e = 0;
        dummy_tgt.d = 0;


        // === 1. 制导层 ===
        uuv_interface::Cmd3D g_out = guidance_->update(dummy_tgt, current_state_);
        // === 2. 控制层 ===
        Eigen::VectorXd tau_cmd = controller_->update(g_out, current_state_);
        // === 3. 执行器层 ===
        Eigen::VectorXd actuator_tau = actuator_->update(tau_cmd, current_state_);
        // === 4. 动力学层 ===
        current_state_ = dynamics_->update(actuator_tau);
        // === 5. 更新传感器 === 
        updateSensor();
        // === 6. 发布状态话题 === 
        publishState(current_state_, current_time);
        // === 7. 更新可视化(TF和轨迹) ===
        if ((current_time - last_visual_time_).toSec() >= visual_period_ * 0.95) {
            publishTF(current_state_, current_time);
            publishTrajectory(current_state_, current_time);
            last_visual_time_ = current_time;
        }
    }

    void updateSensor() {
        for (auto& sensor : sensors_) {
            sensor->update(current_state_);
        }
    }

    void publishState(const uuv_interface::State3D& state, const ros::Time& time) {
        uuv_interface::State3D state_msg = state;
        state_msg.header.stamp = time; 
        state_msg.header.frame_id = world_frame_;
        pub_state_.publish(state_msg);
    }

    // 轨迹发布函数
    void publishTrajectory(const uuv_interface::State3D& state, const ros::Time& time) {
        // 1. 距离校验：如果轨迹数组不为空，计算当前点与上一个记录点的 3D 欧氏距离
        if (!uuv_trajectory_.poses.empty()) {
            const auto& last_pos = uuv_trajectory_.poses.back().pose.position;
            double dx = state.x - last_pos.x;
            double dy = state.y - last_pos.y;
            double dz = state.z - last_pos.z;
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            // 如果移动距离小于设定的阈值，直接返回，不记录也不发布！(极大节省计算和通信资源)
            if (dist < trajectory_seg_length_) {
                return; 
            }
        }

        // 2. 距离达标（或是第一个点），开始记录新点
        geometry_msgs::PoseStamped current_pose;
        current_pose.header.stamp = time;
        current_pose.header.frame_id = uuv_trajectory_.header.frame_id;

        current_pose.pose.position.x = state.x;
        current_pose.pose.position.y = state.y;
        current_pose.pose.position.z = state.z;

        tf2::Quaternion q;
        q.setRPY(state.roll, state.pitch, state.yaw);
        current_pose.pose.orientation.x = q.x();
        current_pose.pose.orientation.y = q.y();
        current_pose.pose.orientation.z = q.z();
        current_pose.pose.orientation.w = q.w();

        uuv_trajectory_.poses.push_back(current_pose);

        // 3. 维持滑动窗口
        if (uuv_trajectory_.poses.size() > max_trajectory_length_) {
            uuv_trajectory_.poses.erase(uuv_trajectory_.poses.begin());
        }

        // 4. 更新时间戳并发布
        uuv_trajectory_.header.stamp = time;
        pub_trajectory_.publish(uuv_trajectory_);
    }

    void publishTF(const uuv_interface::State3D& state, const ros::Time& time) {
        geometry_msgs::TransformStamped t;
        t.header.stamp = time - ros::Duration(2*visual_period_); 
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