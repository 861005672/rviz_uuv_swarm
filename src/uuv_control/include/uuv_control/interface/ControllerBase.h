#ifndef CONTROLLOR_BASE_H
#define CONTROLLOR_BASE_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <sensor_msgs/Joy.h>

namespace uuv_control {

class ControllerBase {
public:
    virtual ~ControllerBase() {}

    // 初始化
    virtual void initialize(ros::NodeHandle& nh) = 0;

    // 核心计算函数：输出控制力/力矩
    virtual Eigen::VectorXd compute() = 0;
};

}
#endif