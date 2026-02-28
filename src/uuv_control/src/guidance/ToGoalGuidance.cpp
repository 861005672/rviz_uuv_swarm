#include <uuv_interface/GuidanceBase.h>
#include <pluginlib/class_list_macros.h>
#include <uuv_interface/State3D.h>
#include <uuv_interface/TargetPoint3D.h>
#include <uuv_interface/Cmd3D.h>
#include <uuv_interface/utils/XmlParamReader.h>
#include <cmath>

namespace uuv_control {

class ToGoalGuidance : public uuv_interface::GuidanceBase {
private:
    double max_pitch_deg_ = 40.0;
    double max_pitch_rad_ = 0.2;
    double cruise_speed_ = 7.7;     // 默认高速巡航速度 (约 15 节)
    double acceptance_radius_ = 2.0; // 到达目标点的容忍半径

public:
    void initialize(ros::NodeHandle& gnh, const std::string& plugin_xml) override {
        // 1. 调用大基类层级初始化开启拦截服务
        initializePlugin(gnh);
        initGuidanceLevel();

        // 2. 纯粹读取算法专属参数
        uuv_interface::XmlParamReader reader(plugin_xml);
        reader.param("update_rate", this->update_rate_, 10.0);
        reader.param("cruise_speed", cruise_speed_, 7.7); 
        reader.param("acceptance_radius", acceptance_radius_, 2.0);
        reader.param("max_pitch_deg", max_pitch_deg_, 30.0);

        max_pitch_rad_ = max_pitch_deg_ / 180.0 * M_PI;

        ROS_INFO_STREAM("[ToGoalGuidance] Pure Math Model Loaded: \n update_rate="<<update_rate_
            <<" \n cruise_speed="<<cruise_speed_<<" \n acceptance_radius="<<acceptance_radius_
            <<" \n max_pitch_deg="<<max_pitch_deg_);
    }

    // 核心黑盒运算，无任何内部状态缓存，只依赖传入参数
    uuv_interface::Cmd3D update(const uuv_interface::TargetPoint3D& target, const uuv_interface::State3D& state) override {
        // 严格按照要求的第一行：智能拦截器
        auto actual_target = resolveInput(target);
        
        uuv_interface::Cmd3D out;

        // 使用 actual_target 和传入的 state 进行纯数学计算
        double dx = actual_target.n - state.x;
        double dy = actual_target.e - state.y;
        double dz = actual_target.d - state.z; 
        
        double dist_xy = std::sqrt(dx * dx + dy * dy);
        double distance = std::sqrt(dist_xy * dist_xy + dz * dz);

        if (distance < acceptance_radius_) {
            out.target_u = 0.0;
            out.target_pitch = 0.0;
            out.target_yaw = state.yaw; // 到达目标附近，保持当前偏航角
        } else {
            out.target_u = cruise_speed_;
            out.target_yaw = std::atan2(dy, dx);
            out.target_pitch = std::atan2(-dz, dist_xy); 
            out.target_pitch = std::max(-max_pitch_rad_, std::min(out.target_pitch, max_pitch_rad_));
        }

        return out;
    }
};

} // namespace uuv_control

PLUGINLIB_EXPORT_CLASS(uuv_control::ToGoalGuidance, uuv_interface::GuidanceBase)