#ifndef CONTROLLOR_BASE_H
#define CONTROLLOR_BASE_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <sensor_msgs/Joy.h>
#include <uuv_interface/PluginBase.h>
#include <uuv_interface/Cmd3D.h>

namespace uuv_interface {

class ControllerBase : public PluginBase {
public:
    virtual ~ControllerBase() {}

    // 用于接收制导层在本地内存传递的指令
    // 提供默认空实现，避免影响不使用该接口的控制器 (如 TeleopJoyController)
    virtual void setCommand(const uuv_interface::Cmd3D& cmd) {}

    // 核心计算函数：输出控制力/力矩
    virtual Eigen::VectorXd compute() = 0;
};

}
#endif