// src/uuv_interface/include/uuv_interface/GuidanceBase.h
#ifndef GUIDANCE_BASE_H
#define GUIDANCE_BASE_H

#include <ros/ros.h>
#include <uuv_interface/PluginBase.h>
#include <uuv_interface/TargetPoint3D.h>
#include <uuv_interface/Cmd3D.h>

namespace uuv_interface {

class GuidanceBase : public PluginBase {
public:
    virtual ~GuidanceBase() {}

    // 供未来的【规划层】在本地内存直接设置目标点，代替服务通信
    virtual void setTargetPoint(const uuv_interface::TargetPoint3D& target) {}

    // 核心计算函数：根据当前位姿计算期望航速与姿态
    virtual uuv_interface::Cmd3D compute() = 0;
};

}
#endif