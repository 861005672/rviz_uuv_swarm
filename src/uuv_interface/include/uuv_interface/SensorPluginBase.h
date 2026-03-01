#ifndef SENSOR_PLUGIN_BASE_H
#define SENSOR_PLUGIN_BASE_H

#include <ros/ros.h>
#include <string>
#include <uuv_interface/utils/XmlParamReader.h>
#include <uuv_interface/State3D.h> 
#include <uuv_interface/utils/UUVLogger.h>

namespace uuv_interface {

class SensorPluginBase {
protected:
    double update_rate_ = 10.0;  
    double publish_debug_rate_ = 1.0;  
    bool publish_debug_ = false;
    ros::Time last_update_time_;
    ros::Time last_publish_debug_time_;
    
    ros::NodeHandle nh_;
    std::string ns_;

    std::shared_ptr<uuv_interface::UUVLogger> uuv_logger_;

public:
    virtual ~SensorPluginBase() = default;

    void initialize(ros::NodeHandle& nh, const std::string& plugin_xml) {
        uuv_interface::XmlParamReader reader(plugin_xml);
        reader.param("update_rate", update_rate_, 10.0);
        reader.param("publish_debug", publish_debug_, false);
        reader.param("publish_debug_rate", publish_debug_rate_, 1.0);
        
        nh_ = nh;
        ns_ = nh_.getNamespace();
        if (!ns_.empty() && ns_[0] == '/') {
            ns_ = ns_.substr(1); 
        }
        last_update_time_ =  ros::Time(0);
        last_publish_debug_time_ = ros::Time(0);

        initPlugin(nh, plugin_xml);
        initPublishDebug();

        UUV_INFO << "[SensorPluginBase] Param loaded:\n update_rate=\n" << update_rate_ << "\n publish_debug=\n" << publish_debug_ << "\n publish_debug_rate=\n" << publish_debug_rate_;
    }

    double get_rate() const { return update_rate_; }
    std::string get_ns() { return ns_; }
    ros::NodeHandle& get_nh() { return nh_; }

    // 由基类代为完成更新频率控制和 Debug 发布控制
    void update(const uuv_interface::State3D& state) {
        ros::Time now = ros::Time::now();
        
        // 1. 更新频率控制
        if (last_update_time_.isZero()) {
            last_update_time_ = now;
            return; 
        }
        double dt_update = (now - last_update_time_).toSec();
        if (dt_update <= 0.0 || dt_update < (1.0 / this->update_rate_) * 0.95) return;
        last_update_time_ = now;

        // 2. 调试信息发布控制
        if (last_publish_debug_time_.isZero()) { last_publish_debug_time_ = now; }
        double dt_publish_debug = (now - last_publish_debug_time_).toSec();
        if (dt_publish_debug > (1.0 / this->publish_debug_rate_) * 0.95) {
            publishDebug(now);
            last_publish_debug_time_ = now;
        }

        // 3. 执行纯粹的传感器算法
        customUpdate(state, dt_update);
    }

    void setLogger(std::shared_ptr<uuv_interface::UUVLogger> logger) {
        uuv_logger_ = logger;
    }


protected:
    virtual void initPlugin(ros::NodeHandle& nh, const std::string& plugin_xml) = 0;
    virtual void publishDebug(const ros::Time& time) = 0;
    virtual void initPublishDebug() = 0;
    
    // 【新增】子类只需关心拿到最新的 state 和 dt 后，怎么计算传感数据
    virtual void customUpdate(const uuv_interface::State3D& state, double dt) = 0;
};

} // namespace uuv_interface
#endif