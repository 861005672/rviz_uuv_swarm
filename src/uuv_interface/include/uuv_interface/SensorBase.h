#pragma once
#include <ros/ros.h>
#include <string>
#include <uuv_interface/PluginBase.h>


namespace uuv_interface {

struct SensorState {
    double x, y, z;
    double roll, pitch, yaw;
    double u, v, w;
    double p, q, r;
};

class SensorBase : public PluginBase {
protected:
    double update_rate_ = 10.0;       // 默认传感器频率 10Hz
    ros::Time last_update_time_;      // 上次更新的时间戳
    std::string uuv_name_;            // UUV 的命名空间

    // 纯虚函数：交由具体的传感器子类去实现其独特的物理模拟逻辑
    virtual void generateData(const SensorState& current_state) = 0;
public:
    virtual ~SensorBase() = default;

    // 初始化接口
    virtual void initialize(ros::NodeHandle& nh, const std::string& robot_description) = 0;

    // 统一的对外更新接口（非虚函数），在这里进行全局的频率控制！
    void update(const SensorState& current_state) {
        ros::Time now = ros::Time::now();
        
        // 如果 update_rate_ <= 0，或者还没到下一次更新的时间，则直接跳过
        if (update_rate_ > 0.0 && (now - last_update_time_).toSec() >= (1.0 / update_rate_)*0.95) {
            // 时间到了，调用子类的具体物理计算逻辑
            generateData(current_state);
            // 刷新时间戳
            last_update_time_ = now;
        }
    }
};

} // namespace uuv_sensor