#ifndef CONTROL_PLUGIN_BASE_H
#define CONTROL_PLUGIN_BASE_H

#include <ros/ros.h>
#include <string>
#include <uuv_interface/utils/XmlParamReader.h>
#include <uuv_interface/utils/UUVLogger.h>


namespace uuv_interface {

class ControlPluginBase {
protected:
    double update_rate_ = 10.0;  
    double publish_debug_rate_ = 1.0;  
    bool publish_debug_ = false;
    ros::Time last_update_time_;
    ros::Time last_publish_debug_time_;
    
    // 大基类新增的节点句柄与层级标识
    ros::NodeHandle nh_;
    std::string ns_;
    std::string control_level_;
    std::string self_level_;
    bool is_overridden_ = false;
    std::shared_ptr<uuv_interface::UUVLogger> uuv_logger_;

public:
    virtual ~ControlPluginBase() = default;

    // 大基类托管的基础初始化函数,所有插件初始化时都需要调用这个函数！！！
    virtual void initialize(ros::NodeHandle& nh, const std::string& plugin_xml, const std::string& self_level) {
        uuv_interface::XmlParamReader reader(plugin_xml);
        reader.param("update_rate", update_rate_, 20.0);
        reader.param("publish_debug", publish_debug_, 10.0);
        reader.param("publish_debug_rate", publish_debug_rate_, 10.0);
        nh_ = nh;
        ns_ = nh_.getNamespace();
        if (!ns_.empty() && ns_[0] == '/') {
            ns_ = ns_.substr(1); // 剥离掉最前面的 '/'，防止 tf 报错
        }
        self_level_ = self_level;
        last_update_time_ =  ros::Time(0);
        last_publish_debug_time_ = ros::Time(0);

        initControlLevel();
        initPlugin(nh, plugin_xml);
        initPublishDebug();

        UUV_INFO << "[ControlPluginBase] Base Param loaded:\n self_level=\n"<<self_level
            <<"\n control_level=\n"<<control_level_<<"\n update_rate=\n"<<update_rate_<<"\n publish_debug=\n"
            <<publish_debug_<<"\n publish_debug_rate=\n"<<publish_debug_rate_;
    }

    // 统一读取接口
    virtual double get_rate() const {
        return update_rate_;
    }
    std::string& get_ns() { return ns_; }
    ros::NodeHandle& get_nh() { return nh_; }

    void setLogger(std::shared_ptr<uuv_interface::UUVLogger> logger) {
        uuv_logger_ = logger;
    }

protected:
    // 统一的初始化接口：传入节点句柄和完整的 URDF XML 字符串, 留给各层级的实现子类重写
    virtual void initPlugin(ros::NodeHandle& nh, const std::string& plugin_xml) = 0;
    // 留给层级基类（GuidanceBase等）去注册各自专属的消息类型服务
    virtual void registerOverrideService() = 0;
    // 发布调试信息的虚函数，子类可以继承这个函数来发布所有调试信息,留给各层级的实现子类
    virtual void publishDebug(const ros::Time& time) = 0;
    // 初始化调试信息发布
    virtual void initPublishDebug() = 0;
    // 初始化控制层级
    void initControlLevel() {
        nh_.param<std::string>("control_level", control_level_, "guidance");
        if (control_level_ == self_level_) {
            registerOverrideService();    // 召唤子类注册对应的 ROS Service
            is_overridden_ = true;
        }
    }


};

} // namespace uuv_interface
#endif