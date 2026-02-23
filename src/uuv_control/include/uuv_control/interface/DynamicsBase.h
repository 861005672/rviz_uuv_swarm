#ifndef DYNAMICS_BASE_H
#define DYNAMICS_BASE_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <uuv_control/State3D.h>

namespace uuv_control {

class DynamicsBase {
public:
    virtual ~DynamicsBase() {}

    // 初始化：加载参数
    virtual void initialize(ros::NodeHandle& nh) = 0;

    // 核心步进函数：输入推力(tau)，输出下一时刻的状态
    // dt: 时间步长, tau: [Fx, Fy, Fz, Tx, Ty, Tz]
    virtual uuv_control::State3D update(double dt, const Eigen::VectorXd& tau) = 0;

    // 获取当前状态
    virtual uuv_control::State3D getState() = 0;
};

}
#endif