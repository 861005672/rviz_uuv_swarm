#include <uuv_interface/PlannerBase.h>
#include <pluginlib/class_list_macros.h>

namespace uuv_control {

class SimplePlanner : public uuv_interface::PlannerBase {
protected:
    void initPlugin(ros::NodeHandle& gnh, const std::string& plugin_xml) override {
        UUV_INFO << "[SimplePlanner] SimplePlanner Layer Ready. Passing input targets directly to guidance.";
    }

    uuv_interface::TargetPoint3D customUpdate(const uuv_interface::TargetPoint3D& target_input, const uuv_interface::State3D& state, double dt) override {
        // 【零逻辑规划】：将最终目标，直接原封不动地发给制导层
        if (target_input.id != latest_input_target_.id) {
            UUV_INFO << "[SIMPLE_PLANNER] Received a new input-target [" << latest_input_target_.id << "->" << target_input.id << "]";
            latest_input_target_ = target_input;
        }
        return target_input;
    }

    void initPublishDebug() override {}
    void publishDebug(const ros::Time& time) override {}
};

} // namespace uuv_control

PLUGINLIB_EXPORT_CLASS(uuv_control::SimplePlanner, uuv_interface::PlannerBase)