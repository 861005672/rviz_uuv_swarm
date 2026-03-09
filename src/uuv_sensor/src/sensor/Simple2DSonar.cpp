#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <XmlRpcValue.h>
#include <algorithm>
#include <uuv_interface/SensorPluginBase.h>
#include <uuv_interface/utils/XmlParamReader.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <random>

namespace uuv_sensor {

struct Obstacle {
    std::string name;
    std::string type;
    Eigen::Vector3d pos;
    Eigen::Vector3d scale;
};


class Simple2DSonar : public uuv_interface::SensorPluginBase {
private:
    ros::Publisher pub_scan_;
    
    // 声纳参数
    int beams_;
    double fov_;
    double max_range_;

    // 弱观测参数
    double noise_std_base_;
    double noise_vel_factor_;
    double miss_prob_;
    double false_prob_;
    
    // 环境障碍物容器
    std::vector<Obstacle> obstacles_;

    // 随机数生成器
    std::default_random_engine generator_;
    std::uniform_real_distribution<double> uniform_dist_{0.0, 1.0};

    void loadEnvironment(ros::NodeHandle& nh) {
        // 从参数服务器的 /env_spawner_node/obstacles 读取环境列表
        XmlRpc::XmlRpcValue obs_list;
        if (nh.getParam("/env_spawner_node/obstacles", obs_list)) {
            if (obs_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
                for (int i = 0; i < obs_list.size(); ++i) {
                    XmlRpc::XmlRpcValue& obs = obs_list[i];
                    Obstacle o;
                    o.name = static_cast<std::string>(obs["name"]);
                    o.type = static_cast<std::string>(obs["type"]);
                    
                    // 解析位置
                    XmlRpc::XmlRpcValue& pos = obs["position"];
                    o.pos << static_cast<double>(pos[0]), static_cast<double>(pos[1]), static_cast<double>(pos[2]);
                    
                    // 解析尺寸
                    XmlRpc::XmlRpcValue& scale = obs["scale"];
                    o.scale << static_cast<double>(scale[0]), static_cast<double>(scale[1]), static_cast<double>(scale[2]);
                    
                    obstacles_.push_back(o);
                }
                UUV_INFO << "[Simple2DSonar] Successfully loaded " << obstacles_.size() <<" obstacles from ROS Parameter Server.";
            }
        } else {
            UUV_WARN << "[Simple2DSonar] Failed to get obstacles from parameter server! Ensure env_spawner is running.";
        }
    }
    
    double checkIntersection(const Eigen::Vector3d& ray_origin, const Eigen::Vector3d& ray_dir) {
        double min_dist = 1e9;

        for (const auto& obs : obstacles_) {
            if (obs.type == "sphere") {
                // 球体求交
                double radius = obs.scale.x() / 2.0;
                Eigen::Vector3d L = obs.pos - ray_origin;
                double tca = L.dot(ray_dir);
                if (tca < 0) continue; 
                double d2 = L.dot(L) - tca * tca;
                if (d2 > radius * radius) continue; 
                double thc = sqrt(radius * radius - d2);
                double t0 = tca - thc;
                if (t0 > 0.0 && t0 < min_dist) min_dist = t0;
    
            } else if (obs.type == "cylinder") {
                // 圆柱体求交（垂直于XY面）
                double radius = obs.scale.x() / 2.0;
                double half_height = obs.scale.z() / 2.0;
                
                Eigen::Vector2d O2d(ray_origin.x(), ray_origin.y());
                Eigen::Vector2d D2d(ray_dir.x(), ray_dir.y());
                Eigen::Vector2d C2d(obs.pos.x(), obs.pos.y());
                
                double A = D2d.dot(D2d);
                if (A < 1e-6) continue;
    
                Eigen::Vector2d L2d = O2d - C2d;
                double B = 2.0 * L2d.dot(D2d);
                double C = L2d.dot(L2d) - radius * radius;
                
                double discriminant = B * B - 4 * A * C;
                if (discriminant >= 0) {
                    double t0 = (-B - sqrt(discriminant)) / (2.0 * A);
                    if (t0 > 0) {
                        double hit_z = ray_origin.z() + t0 * ray_dir.z();
                        if (hit_z >= (obs.pos.z() - half_height) && hit_z <= (obs.pos.z() + half_height)) {
                            if (t0 < min_dist) min_dist = t0;
                        }
                    }
                }
    
            } else if (obs.type == "box") {
                // 长方体 AABB (Axis-Aligned Bounding Box) 求交 (Slab Method)
                Eigen::Vector3d min_bound = obs.pos - obs.scale / 2.0;
                Eigen::Vector3d max_bound = obs.pos + obs.scale / 2.0;
    
                // 为了防止除以0，C++ IEEE754标准下1.0/0.0为Inf，可以安全计算
                Eigen::Vector3d inv_dir(1.0 / ray_dir.x(), 1.0 / ray_dir.y(), 1.0 / ray_dir.z());
    
                double tx1 = (min_bound.x() - ray_origin.x()) * inv_dir.x();
                double tx2 = (max_bound.x() - ray_origin.x()) * inv_dir.x();
                double tmin = std::min(tx1, tx2);
                double tmax = std::max(tx1, tx2);
    
                double ty1 = (min_bound.y() - ray_origin.y()) * inv_dir.y();
                double ty2 = (max_bound.y() - ray_origin.y()) * inv_dir.y();
                tmin = std::max(tmin, std::min(ty1, ty2));
                tmax = std::min(tmax, std::max(ty1, ty2));
    
                double tz1 = (min_bound.z() - ray_origin.z()) * inv_dir.z();
                double tz2 = (max_bound.z() - ray_origin.z()) * inv_dir.z();
                tmin = std::max(tmin, std::min(tz1, tz2));
                tmax = std::min(tmax, std::max(tz1, tz2));
    
                // tmax >= tmin 说明射线贯穿了包围盒，tmin > 0 说明盒子在射线正前方
                if (tmax >= tmin && tmin > 0) {
                    if (tmin < min_dist) min_dist = tmin;
                }
            }
        }
        return min_dist;
    }

protected:
    void customUpdate(const uuv_interface::State3D& state, double dt) override {
        sensor_msgs::LaserScan scan;
        scan.header.stamp = ros::Time::now();
        scan.header.frame_id = get_ns().empty() ? "base_link" : ns_ + "/base_link";
        scan.angle_min = -fov_ / 2.0;
        scan.angle_max = fov_ / 2.0;
        scan.angle_increment = fov_ / beams_;
        scan.range_min = 0.5;
        scan.range_max = max_range_;
        scan.ranges.resize(beams_, max_range_ + 1.0); // 默认值为超出量程
    
        // 构建机体到世界坐标系的旋转矩阵 R_wb
        tf2::Quaternion q;
        q.setRPY(state.roll, state.pitch, state.yaw);
        tf2::Matrix3x3 R_wb(q);
        Eigen::Vector3d ray_origin(state.x, state.y, state.z);

        double speed = std::sqrt(state.u * state.u + state.v * state.v + state.w * state.w);
        double current_std = noise_std_base_ + noise_vel_factor_ * speed;
        std::normal_distribution<double> noise_dist(0.0, current_std);
    
        // 发射 beams_ 根声纳射线
        for (int i = 0; i < beams_; ++i) {
            double angle = scan.angle_min + i * scan.angle_increment;

            // 射线在机体系下处于水平面内
            tf2::Vector3 ray_b(cos(angle), sin(angle), 0.0); 
            
            // 转换为世界系下真正的 3D 向量
            tf2::Vector3 ray_w_tf = R_wb * ray_b;
            Eigen::Vector3d ray_dir(ray_w_tf.x(), ray_w_tf.y(), ray_w_tf.z());
            ray_dir.normalize();

            // 错检判定- 即使无障碍也可能返回随机值
            if (uniform_dist_(generator_) < false_prob_) {
                scan.ranges[i] = 0.5 + uniform_dist_(generator_) * (max_range_ - 0.5);
                continue;
            }

            // 漏检判定- 即使有障碍也可能丢失回波
            if (uniform_dist_(generator_) < miss_prob_) {
                scan.ranges[i] = max_range_ + 1.0;
                continue;
            }
    
            // 执行 3D 数学求交算法
            double min_dist = checkIntersection(ray_origin, ray_dir);
            if (min_dist < max_range_) {
                // 3. 注入动态高斯噪声
                double noisy_dist = min_dist + noise_dist(generator_);
                scan.ranges[i] = std::max(0.5, std::min(noisy_dist, max_range_));
            }
        }
        pub_scan_.publish(scan);
    }
    void initPublishDebug() override {}
    void publishDebug(const ros::Time& time) override {}

public:
    Simple2DSonar() = default;
    ~Simple2DSonar() override = default;

    void initPlugin(ros::NodeHandle& nh, const std::string& plugin_xml) override {
        uuv_interface::XmlParamReader reader(plugin_xml);

        reader.param("beams", beams_, 30);
        reader.param("max_range", max_range_, 200.0);
        double fov_deg;
        reader.param("fov_deg", fov_deg, 150.0);
        fov_ = fov_deg * M_PI / 180.0;
    
        pub_scan_ = nh.advertise<sensor_msgs::LaserScan>("sonar_scan", 10);

        reader.param("noise_std_base", noise_std_base_, 0.5);
        reader.param("noise_vel_factor", noise_vel_factor_, 0.1);
        reader.param("miss_prob", miss_prob_, 0.05);
        reader.param("false_prob", false_prob_, 0.02);
        
        UUV_INFO << "[Simple2DSonar] Xml Params Loaded: " << "\n beams=\n" 
                        << beams_ << "\n max_range=\n" << max_range_ << "\n fov_deg=\n" << fov_deg;
    
        loadEnvironment(nh);
    }

};

} // namespace uuv_sensor

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uuv_sensor::Simple2DSonar, uuv_interface::SensorPluginBase)