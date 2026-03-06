#include <uuv_interface/DecisionBase.h>
#include <pluginlib/class_list_macros.h>

namespace uuv_control {

class VanguardDecision : public uuv_interface::DecisionBase {
protected:
    void initPlugin(ros::NodeHandle& gnh, const std::string& plugin_xml) override {
        // 基类已经处理了参数读取和话题订阅，这里只需要打个招呼
        UUV_INFO << "[VanguardDecision] VanguardDecision Plugin Loaded. Awaiting vanguard missions...";
    }

    uuv_interface::TargetPoint3D customUpdate(const uuv_interface::State3D& state, 
                                 const uuv_interface::TargetPoint3DArray& mission_list, 
                                 const std::vector<uuv_interface::Neighbor3D>& neighbors,
                                 std::string& data_json, 
                                 double dt) override {
        uuv_interface::TargetPoint3D target;
        
        // 【先锋逻辑】：永远只看任务列表里的第一个任务
        if (!mission_list.targets.empty()) {
            target = mission_list.targets[0];
        } else {
            // 指挥部没有下达任务，或者任务列表为空，原地待命 (id = -1 会被底层的 Guidance 识别为停止信号)
            target.id = -1; 
            target.n = state.x; 
            target.e = state.y; 
            target.d = state.z;
        }    
        return target;
    }

public:

    void initPublishDebug() override {}
    void publishDebug(const ros::Time& time) override {}
};

} // namespace uuv_control

// 向 pluginlib 注册该插件
PLUGINLIB_EXPORT_CLASS(uuv_control::VanguardDecision, uuv_interface::DecisionBase)