// src/uuv_control/src/guidance/ToGoalGuidance.cpp
#include <uuv_interface/GuidanceBase.h>
#include <pluginlib/class_list_macros.h>
#include <uuv_interface/State3D.h>
#include <uuv_interface/TargetPoint3D.h>
#include <uuv_interface/SetTargetPoint3D.h>
#include <uuv_interface/utils/XmlParamReader.h>
#include <geometry_msgs/Point.h>
#include <cmath>

namespace uuv_control {

class ToGoalGuidance : public uuv_interface::GuidanceBase {
private:
    ros::Subscriber sub_state_;
    ros::ServiceServer srv_wp_;
    
    // 模式切换标志
    bool use_local_targetpoint_ = false;

    // 目标位置
    uuv_interface::TargetPoint3D target_point_;

    double max_pitch_deg_ = 40.0;
    double max_pitch_rad_ = 0.2;

    // 当前状态 (NED坐标系)
    geometry_msgs::Point current_point_;

    // 制导参数
    double cruise_speed_ = 7.7;     // 默认高速巡航速度 (约 15 节)
    double acceptance_radius_ = 2.0; // 到达目标点的容忍半径

    ros::Time last_time_;
    uuv_interface::Cmd3D last_output_;

public:
    void initialize(ros::NodeHandle& gnh, const std::string& plugin_xml) override {
        uuv_interface::XmlParamReader reader(plugin_xml);
        
        reader.param("update_rate", this->update_rate_, 10.0);
        reader.param("use_local_targetpoint", use_local_targetpoint_, false);
        reader.param("cruise_speed", cruise_speed_, 7.7); 
        reader.param("acceptance_radius", acceptance_radius_, 2.0);
        reader.param("max_pitch_deg", max_pitch_deg_, 30.0);

        max_pitch_rad_ = max_pitch_deg_/180.0*M_PI;

        // 订阅当前位姿
        sub_state_ = gnh.subscribe("state", 1, &ToGoalGuidance::stateCallback, this);

        ROS_INFO_STREAM("[ToGoalGuidance] Parameter Loaded: \n update_rate=\n"<<update_rate_<<"\n use_local_waypoint=\n"<<use_local_targetpoint_
            <<" \n cruise_speed=\n"<<cruise_speed_<<" \n acceptance_radius=\n"<<acceptance_radius_
            <<" \n max_pitch_deg=\n"<<max_pitch_deg_
        );

        if (!use_local_targetpoint_) {
            srv_wp_ = gnh.advertiseService("set_target_point", &ToGoalGuidance::targetPointServiceCallback, this);
            ROS_INFO("[ToGoalGuidance] TargetPoint Set Mode: SERVICE (Advertised 'set_target_point')");
        } else {
            ROS_INFO("[ToGoalGuidance] TargetPoint Set Mode: LOCAL MEMORY (Managed by Planning Layer)");
        }

        last_time_ = ros::Time::now();
    }

    // 供规划层直接调用的本地接口
    void setTargetPoint(const uuv_interface::TargetPoint3D& target) override {
        if (use_local_targetpoint_) {
            target_point_ = target;
        }
    }

    void stateCallback(const uuv_interface::State3D::ConstPtr& msg) {
        current_point_.x = msg->x;
        current_point_.y = msg->y;
        current_point_.z = msg->z;
    }

    bool targetPointServiceCallback(uuv_interface::SetTargetPoint3D::Request &req, uuv_interface::SetTargetPoint3D::Response &res) {
        target_point_.n = req.target_n;
        target_point_.e = req.target_e;
        target_point_.d = req.target_d;
        res.success = true;
        ROS_INFO("[ToGoalGuidance] Target Updated! X:%.1f, Y:%.1f, Z:%.1f", target_point_.n, target_point_.e, target_point_.d);
        return true;
    }

    uuv_interface::Cmd3D compute() override {
        ros::Time now = ros::Time::now();
        double dt = (now - last_time_).toSec();
        if (dt <= 0.0 || dt < (1.0 / this->update_rate_) * 0.95) return last_output_;
        last_time_ = now;

        double dx = target_point_.n - current_point_.x;
        double dy = target_point_.e - current_point_.y;
        double dz = target_point_.d - current_point_.z; // NED中 z 朝下，目标更深则 dz > 0
        
        double dist_xy = std::sqrt(dx * dx + dy * dy);
        double distance = std::sqrt(dist_xy * dist_xy + dz * dz);

        uuv_interface::Cmd3D out;

        if (distance < acceptance_radius_) {
            // 已抵达目标半径内，航速设为0，保持最后一次的偏航角
            out.target_u = 0.0;
            out.target_pitch = 0.0;
            out.target_yaw = last_output_.target_yaw; 
        } else {
            // 标准 3D 视线法 (LOS) 制导
            out.target_u = cruise_speed_;
            out.target_yaw = std::atan2(dy, dx);
            
            // 俯仰角：由于 NED 坐标系下 z 轴向下，Pitch 正值代表“低头”(即朝向 Z 轴正方向)。
            // 如果 dz > 0（需要往下潜），此时 atan2(dz, dist_xy) 结果为正，完美符合右手法则。
            out.target_pitch = std::atan2(-dz, dist_xy); 
            out.target_pitch = std::max(-max_pitch_rad_, std::min(out.target_pitch, max_pitch_rad_));
        }
        last_output_ = out;
        return out;
    }
};

} // namespace uuv_control

// 注册为 GuidanceBase 插件
PLUGINLIB_EXPORT_CLASS(uuv_control::ToGoalGuidance, uuv_interface::GuidanceBase)