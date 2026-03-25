#ifndef GUIDANCE_BASE_H
#define GUIDANCE_BASE_H

#include <ros/ros.h>
#include <uuv_interface/ControlPluginBase.h>
#include <uuv_interface/TargetPoint3D.h>
#include <uuv_interface/Cmd3D.h>
#include <uuv_interface/State3D.h>
#include <uuv_interface/SetTargetPoint3D.h>
#include <sensor_msgs/LaserScan.h>
#include <uuv_interface/Neighborhood3D.h>
#include <Eigen/Dense>

namespace uuv_interface {

class GuidanceBase : public ControlPluginBase {
protected:
    // 层级拦截机制（核心）
    ros::ServiceServer override_srv_;
    ros::Subscriber sonar_sub_;
    uuv_interface::Cmd3D latest_output_cmd_;
    sensor_msgs::LaserScan latest_sonar_;
    uuv_interface::Neighborhood3D latest_neighbors_;
    uuv_interface::TargetPoint3D latest_target_;
    uuv_interface::TargetPoint3D override_input_;



    // ROS 服务回调函数：成功截断上层输入
    bool overrideCallback(uuv_interface::SetTargetPoint3D::Request &req,
                          uuv_interface::SetTargetPoint3D::Response &res) {
        override_input_.id = latest_target_.id+1;
        override_input_.n = req.target_n;
        override_input_.e = req.target_e;
        override_input_.d = req.target_d;
        res.success = true;
        return true;
    }

    void registerOverrideService() override {
        override_srv_ = nh_.advertiseService("set_guidance_target", &GuidanceBase::overrideCallback, this);
        UUV_WARN << "==================================================";
        UUV_WARN << "[GuidanceBase] Guidance Override Input Service is READY.";
        UUV_WARN << "==================================================";
    }

    void sonarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        latest_sonar_ = *msg;
    }
    void neighborCallback(const uuv_interface::Neighborhood3D::ConstPtr& msg) {
        latest_neighbors_ = *msg;
    }

    // 注册传感器订阅的公共方法，供 ControlPluginBase 统一调用或在初始化时调用
    void registerSensorSubscribers() {
        sonar_sub_ = nh_.subscribe("sonar_scan", 1, &GuidanceBase::sonarCallback, this);
        UUV_INFO << "[GuidanceBase] Sensor Subscriptions (Sonar & Neighborhood) Registered.";
    }

    virtual uuv_interface::Cmd3D customUpdate(const uuv_interface::TargetPoint3D& target, 
                                              const uuv_interface::State3D& state, 
                                              const std::vector<uuv_interface::Neighbor3D>& neighbors,
                                              double dt) = 0;

public:

    GuidanceBase() {
        latest_target_.id = -1;
        override_input_ = latest_target_;
    }
    virtual ~GuidanceBase() {}
    
    virtual Eigen::Vector3d getTargetDir() const { return Eigen::Vector3d(1, 0, 0); }

    // 统一的纯虚 update 函数
    uuv_interface::Cmd3D update(const uuv_interface::TargetPoint3D& target, 
                                const uuv_interface::State3D& state,
                                const std::vector<uuv_interface::Neighbor3D>& neighbors) {
        ros::Time now = ros::Time::now();

        // 1. 调试发布拦截
        if (last_publish_debug_time_.isZero()) { last_publish_debug_time_ = now; }
            double dt_publish_debug = (now - last_publish_debug_time_).toSec();
            if (dt_publish_debug > (1.0 / this->publish_debug_rate_) * 0.95) {
                publishDebug(now);
                last_publish_debug_time_ = now;
        }

        // 2. 频率控制与 dt 计算
        if (last_update_time_.isZero()) {
            last_update_time_ = now;
            // 初始第一帧时，指令维持静止，姿态对齐
            latest_output_cmd_.target_u = 0.0;
            latest_output_cmd_.target_pitch = state.pitch;
            latest_output_cmd_.target_yaw = state.yaw;
            return latest_output_cmd_;
        }
        double dt_update = (now - last_update_time_).toSec();
        if (dt_update <= 0.0 || dt_update < (1.0 / this->update_rate_) * 0.95) return latest_output_cmd_;
        last_update_time_ = now;



        // 3. 验证输入源 (是上层传进来的目标，还是玩家用手柄 / 服务设定的新目标？)
        const uuv_interface::TargetPoint3D& actual_target = is_overridden_ ? override_input_ : target;

        // 4. 执行纯粹的子类几何追踪算法
        latest_output_cmd_ = customUpdate(actual_target, state, neighbors, dt_update);
        return latest_output_cmd_;
    }

    bool isSamePoint(const uuv_interface::TargetPoint3D& p1, const uuv_interface::TargetPoint3D& p2) {
        if (p1.id!=p2.id) return false;
        else if (std::abs(p1.n - p2.n) < 1e-3 && 
                 std::abs(p1.e - p2.e) < 1e-3 && 
                 std::abs(p1.d - p2.d) < 1e-3) return true;
        else return false;
    }

};

} // namespace uuv_interface
#endif