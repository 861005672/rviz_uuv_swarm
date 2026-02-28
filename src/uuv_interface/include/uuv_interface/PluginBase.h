#ifndef PLUGIN_BASE_H
#define PLUGIN_BASE_H

#include <ros/ros.h>
#include <string>

namespace uuv_interface {

class PluginBase {
protected:
    double update_rate_ = 10.0; // 提供一个受保护的默认运行频率 (Hz)
    
    // 大基类新增的节点句柄与层级标识
    ros::NodeHandle nh_;
    std::string control_level_;
    bool publish_debug_ = false;

public:
    virtual ~PluginBase() = default;

    // 大基类托管的基础初始化函数
    virtual void initializePlugin(ros::NodeHandle& nh) {
        nh_ = nh;
        nh_.param<std::string>("control_level", control_level_, "guidance");
    }

    // 发布调试信息的虚函数，子类可以继承这个函数来发布所有调试信息
    virtual void publishDebug(const ros::Time& time) {}

    // 统一的初始化接口：传入节点句柄和完整的 URDF XML 字符串
    virtual void initialize(ros::NodeHandle& gnh, const std::string& robot_description) = 0;

    // 统一的获取频率接口
    virtual double get_rate() const {
        return update_rate_;
    }
};

} // namespace uuv_interface
#endif