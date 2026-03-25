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
#include <queue>   // 新增：包含队列头文件
#include <utility> // 新增：用于 std::pair

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
    double noise_dist_factor_;
    double miss_prob_;
    double false_prob_;
    double sonar_delay_;
    int max_patch_error_size_;    // 最大的成片错/漏检宽度
    int max_patch_error_time_;    // 最长的成片错/漏检持续帧数
    
    // --- 新增：成片干扰块记忆结构 ---
    struct PatchInterference {
        bool active = false;    // 当前是否处于激活状态
        int start_idx = 0;      // 影响的起始波束索引
        int end_idx = 0;        // 影响的结束波束索引
        int frames_left = 0;    // 剩余持续帧数
        double patch_distance = 0.0;    // 记录这片错检的统一基准距离
    };
    PatchInterference current_patch_miss_;  // 当前的成片漏检块
    PatchInterference current_patch_false_; // 当前的成片错检块

    // 环境障碍物容器
    std::vector<Obstacle> obstacles_;
    // pair.first 存储该消息允许发布的真实系统时间，pair.second 存储消息本体
    std::queue<std::pair<ros::Time, sensor_msgs::LaserScan>> delay_queue_;

    // 随机数生成器
    std::default_random_engine generator_;
    std::uniform_real_distribution<double> uniform_dist_{0.0, 1.0};
    std::uniform_int_distribution<int> center_dist_;
    std::uniform_int_distribution<int> width_dist_;
    std::uniform_int_distribution<int> frame_dist_;

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
        scan.ranges.resize(beams_, max_range_ - 0.1); // 默认值为超出量程
        scan.intensities.resize(beams_, 0.0);  // 初始化方差数组
    
        // 构建机体到世界坐标系的旋转矩阵 R_wb
        tf2::Quaternion q;
        q.setRPY(state.roll, state.pitch, state.yaw);
        tf2::Matrix3x3 R_wb(q);
        Eigen::Vector3d ray_origin(state.x, state.y, state.z);

        double speed = std::sqrt(state.u * state.u + state.v * state.v + state.w * state.w);
        double base_and_speed_std = noise_std_base_ * (1+noise_vel_factor_ * speed);
        std::normal_distribution<double> standard_normal_dist(0.0, 1.0);

        // 更新或生成成片（块状）的漏检干扰
        if (current_patch_miss_.active) {
            current_patch_miss_.frames_left--;
            if (current_patch_miss_.frames_left <= 0) current_patch_miss_.active = false;
        } else {
            // 以单点漏检概率的 1/2 触发成片漏检
            if (uniform_dist_(generator_) < (miss_prob_ / 2.0)) {
                current_patch_miss_.active = true;
                int center = center_dist_(generator_);
                int width = width_dist_(generator_);
                current_patch_miss_.start_idx = std::max(0, center - width / 2);
                current_patch_miss_.end_idx = std::min(beams_ - 1, center + width / 2);
                current_patch_miss_.frames_left = frame_dist_(generator_);
            }
        }
        // 2. 更新或生成成片错检 (False Alarm Patch)
        if (current_patch_false_.active) {
            current_patch_false_.frames_left--;
            if (current_patch_false_.frames_left <= 0) current_patch_false_.active = false;
        } else {
            // 以单点错检概率的 1/2 触发成片错检
            if (uniform_dist_(generator_) < (false_prob_ / 2.0)) {
                current_patch_false_.active = true;
                int center = center_dist_(generator_);
                int width = width_dist_(generator_);
                current_patch_false_.start_idx = std::max(0, center - width / 2);
                current_patch_false_.end_idx = std::min(beams_ - 1, center + width / 2);
                current_patch_false_.frames_left = frame_dist_(generator_);
                // 为这片“幽灵墙”生成一个统一的距离，更倾向于生成在 UUV 附近
                current_patch_false_.patch_distance = 20.0 + uniform_dist_(generator_) * (max_range_ / 2.0);
            }
        }
    
        // 发射 beams_ 根声纳射线
        for (int i = 0; i < beams_; ++i) {
            double angle = scan.angle_min + i * scan.angle_increment;
            // 射线在机体系下处于水平面内
            tf2::Vector3 ray_b(cos(angle), sin(angle), 0.0); 
            // 转换为世界系下真正的 3D 向量
            tf2::Vector3 ray_w_tf = R_wb * ray_b;
            Eigen::Vector3d ray_dir(ray_w_tf.x(), ray_w_tf.y(), ray_w_tf.z());
            ray_dir.normalize();

            // 1. 执行 3D 数学求交算法
            double min_dist = checkIntersection(ray_origin, ray_dir);

            // 2. 计算并注入高斯背景底噪 (无论是否扫到东西都要加)
            // 假设空旷水域的虚拟回波也带有与最大量程相当的噪声
            double effective_dist = (min_dist < max_range_) ? min_dist : max_range_-0.1;
            double current_std = base_and_speed_std * (1.0 + noise_dist_factor_ * effective_dist);
            double noisy_dist = effective_dist + standard_normal_dist(generator_) * current_std;
            scan.ranges[i] = std::max(0.5, std::min(noisy_dist, max_range_ - 0.1));
            scan.intensities[i] = current_std * current_std;

            bool is_in_false_patch = (current_patch_false_.active && i >= current_patch_false_.start_idx && i <= current_patch_false_.end_idx);
            bool is_in_miss_patch = (current_patch_miss_.active && i >= current_patch_miss_.start_idx && i <= current_patch_miss_.end_idx);

            // is_in_false_patch = false;
            // is_in_miss_patch = false;

            // 成片错检
            if (is_in_false_patch) {
                // 使用基准距离，并加上微小的高斯波动 让假墙显得有粗糙度
                double fluctuation = standard_normal_dist(generator_) * 2; 
                scan.ranges[i] = std::max(0.5, std::min(current_patch_false_.patch_distance + fluctuation, max_range_));
                scan.intensities[i] = 100.0;
                continue;
            }

            // 单点错检
            if (uniform_dist_(generator_) < false_prob_) {
                scan.ranges[i] = 0.5 + uniform_dist_(generator_) * (max_range_ - 0.5);
                scan.intensities[i] = 100.0;
                continue;
            }

            // 2. 漏检判定 (单点漏检 或 处于成片漏检块中)
            if (uniform_dist_(generator_) < miss_prob_ || is_in_miss_patch) {
                scan.ranges[i] = std::max(0.5, std::min((max_range_-0.1) + standard_normal_dist(generator_) * 0.5, max_range_-0.1));
                scan.intensities[i] = 0.0;
                continue;
            }
    

            if (min_dist < max_range_) {
                // 真实物理噪声标准差随距离线性放大
                double current_variance = current_std * current_std;
                // 3. 注入动态高斯噪声
                scan.ranges[i] = std::max(0.5, std::min(noisy_dist, max_range_));
                // 4. 将该射线的当前方差赋值给 intensities (距离越远误差越大)
                scan.intensities[i] = current_variance;
            }
        }
        // ================== 时延模拟逻辑 ==================
        // 1. 获取当前系统时间
        ros::Time current_time = ros::Time::now();
        // 2. 计算这条数据应该在未来哪个时间点被发布出来
        ros::Time target_pub_time = current_time + ros::Duration(sonar_delay_);
        // 3. 将其压入延迟队列，暂不发布
        delay_queue_.push({target_pub_time, scan});
        // 4. 检查队列头部，把所有“时间已到”的老数据发布出去
        while (!delay_queue_.empty() && delay_queue_.front().first <= current_time) {
            pub_scan_.publish(delay_queue_.front().second);
            delay_queue_.pop();
        }
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
        reader.param("noise_dist_factor", noise_dist_factor_, 0.02);
        reader.param("miss_prob", miss_prob_, 0.05);
        reader.param("false_prob", false_prob_, 0.02);
        reader.param("sonar_delay", sonar_delay_, 0.4);
        reader.param("max_patch_error_size", max_patch_error_size_, 8);
        reader.param("max_patch_error_time", max_patch_error_time_, 10);

        center_dist_ = std::uniform_int_distribution<int>(0, beams_ - 1);
        width_dist_ = std::uniform_int_distribution<int>(1, max_patch_error_size_);
        frame_dist_ = std::uniform_int_distribution<int>(1, max_patch_error_time_);


        UUV_INFO << "[Simple2DSonar] Xml Params Loaded: " << "\n beams=\n" 
                        << beams_ << "\n max_range=\n" << max_range_ << "\n fov_deg=\n" << fov_deg;
    
        loadEnvironment(nh);
    }

};

} // namespace uuv_sensor

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uuv_sensor::Simple2DSonar, uuv_interface::SensorPluginBase)