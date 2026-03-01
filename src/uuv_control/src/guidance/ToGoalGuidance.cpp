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
    void initPlugin(ros::NodeHandle& gnh, const std::string& plugin_xml) override {
        // 1. 纯粹读取算法专属参数
        uuv_interface::XmlParamReader reader(plugin_xml);
        reader.param("cruise_speed", cruise_speed_, 7.7); 
        reader.param("acceptance_radius", acceptance_radius_, 2.0);
        reader.param("max_pitch_deg", max_pitch_deg_, 30.0);

        max_pitch_rad_ = max_pitch_deg_ / 180.0 * M_PI;

        UUV_INFO << "[ToGoalGuidance] Pure Math Model Loaded: \n"
            <<"\n cruise_speed=\n"<<cruise_speed_<<"\n acceptance_radius=\n"<<acceptance_radius_
            <<"\n max_pitch_deg=\n"<<max_pitch_deg_;

        double init_x, init_y, init_z;
        gnh.param("init_x", init_x, 0.0);
        gnh.param("init_y", init_y, 0.0);
        gnh.param("init_z", init_z, 0.0);
        override_input_.n = init_x;
        override_input_.e = init_y;
        override_input_.d = init_z;
    }

    virtual void initPublishDebug() {}

    void publishDebug(const ros::Time& time) override {
        if (!publish_debug_) return;
        // 制导层可以留空，后续如有需要可以把当前的 LOS 误差通过话题发出来
    }

    // 核心黑盒运算，无任何内部状态缓存，只依赖传入参数
    uuv_interface::Cmd3D customUpdate(const uuv_interface::TargetPoint3D& target, const uuv_interface::State3D& state, double dt) override {
        uuv_interface::Cmd3D out;

        // 使用 target 和传入的 state 进行纯数学计算
        double dx = target.n - state.x;
        double dy = target.e - state.y;
        double dz = target.d - state.z; 
        
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