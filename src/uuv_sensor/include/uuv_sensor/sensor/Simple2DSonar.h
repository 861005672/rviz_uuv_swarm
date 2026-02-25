#pragma once
#include <uuv_sensor/interface/SensorBase.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Dense>
#include <vector>
#include <string>

namespace uuv_sensor {

struct Obstacle {
    std::string name;
    std::string type;
    Eigen::Vector3d pos;
    Eigen::Vector3d scale;
};

class Simple2DSonar : public SensorBase {
private:
    ros::Publisher pub_scan_;
    
    // 声纳参数
    int beams_;
    double fov_;
    double max_range_;
    
    // 环境障碍物容器
    std::vector<Obstacle> obstacles_;

    void loadEnvironment(ros::NodeHandle& nh);
    double checkIntersection(const Eigen::Vector3d& ray_origin, const Eigen::Vector3d& ray_dir);

protected:
    void generateData(const SensorState& current_state) override;

public:
    Simple2DSonar() = default;
    ~Simple2DSonar() override = default;

    void initialize(ros::NodeHandle& nh) override;
};

} // namespace uuv_sensor