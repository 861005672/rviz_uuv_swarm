#ifndef PLUGIN_BASE_H
#define PLUGIN_BASE_H

#include <ros/ros.h>
#include <string>

namespace uuv_interface {

class PluginBase {
protected:
    double update_rate_ = 10.0; // 提供一个受保护的默认运行频率 (Hz)

public:
    virtual ~PluginBase() = default;

    // 统一的初始化接口：传入节点句柄和完整的 URDF XML 字符串
    virtual void initialize(ros::NodeHandle& gnh, const std::string& robot_description) = 0;

    // 统一的获取频率接口
    virtual double get_rate() const {
        return update_rate_;
    }
};

} // namespace uuv_control
#endif