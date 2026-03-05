#include <uuv_interface/GuidanceBase.h>
#include <pluginlib/class_list_macros.h>
#include <uuv_interface/utils/XmlParamReader.h>
#include <uuv_interface/utils/utils.h>  
#include <Eigen/Dense>
#include <cmath>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Geometry> // 用于计算3D旋转矩阵

namespace uuv_control {

struct VirtualTarget3D {
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d dir; 
};

class SpacialFlockingGuidance : public uuv_interface::GuidanceBase {
private:
    double max_pitch_rad_;
    double cruise_speed_;           // 宏观巡航速度 (集群稳定时的速度)
    double min_speed_;              // 最小机动维持速度
    double max_speed_;              // 物理极限速度 (允许用来追赶队友的最大速度)
    double t_ftotal_;               // 合力的平滑限幅窗，值越大越平滑
    double forward_tendency_;       // 决定对抗侧向力的航向刚度，用于航向/俯仰角的计算
    double speed_map_scale_;        // 航速映射标尺，决定受到多大纵向分力时，UUV将达到最高航速


    // --- 极简且完美的物理模型参数 ---
    // 导航力
    double zeta_p_;                 // 导航力位置跟踪系数
    double zeta_v_;                 // 导航力速度阻尼系数
    double nav_force_limit_;        // 导航力限幅
    // 内力
    double zeta_n_;                 // 排斥力增益
    double delta_n_;                // 平衡距离 (零力点)
    double k_pull_;                 // 吸引力削弱系数
    double r_comm_;                 // 通信/感知半径
    double force_lp_gain_;          // 低通滤波平滑系数，越小越平滑
    double flock_force_limit_;      // 内力限幅


    // --- 避障力 (Obstacle Avoidance) 模型参数 ---
    double d_sense_;                // 障碍物感知距离阈值
    double d_inflation_;            // 膨胀安全距离
    double zeta_obs_;               // 避障力强度系数
    double delta_obs_;              // 避障力衰减指数系数
    double zeta_norm_min_;          // 法向力最小权重
    double zeta_norm_max_;          // 法向力最大权重
    double zeta_tan_min_;           // 切向力最小权重
    double zeta_tan_max_;           // 切向力最大权重
    int num_sectors_;               // 扇区化池化数量
    

    std::map<std::string, uuv_interface::LeastSquaresPredictor3D> nb_predictors_;

    Eigen::Vector3d avoid_dir_smooth_{0, 0, 0}; // 平滑绕行轴

    VirtualTarget3D vTgt_;

    // 可视化发布器与状态记录变量
    ros::Publisher debug_marker_pub_;
    Eigen::Vector3d latest_f_nav_{0,0,0};
    Eigen::Vector3d latest_f_flock_{0,0,0};
    Eigen::Vector3d latest_f_obs_{0,0,0};
    Eigen::Vector3d latest_f_total_{0,0,0};
    Eigen::Vector3d latest_self_pos_{0,0,0};

    uuv_interface::State3D latest_state_;
    std::string local_frame_id_ = "base_link";

    // =========================================================
    // 子函数 1：计算 3D 虚拟扰动合力 (Perturbation Force)
    // =========================================================
    Eigen::Vector3d computeNavForce(const uuv_interface::TargetPoint3D& target, const uuv_interface::State3D& state, double dt) {
        vTgt_.pos += vTgt_.vel * dt;
        // 目标发生变化时，更新虚拟领航点的运动方向
        if (!isSamePoint(target,latest_target_)) {
            latest_target_ = target;
            Eigen::Vector3d target_pos(target.n, target.e, target.d);
            Eigen::Vector3d diff = target_pos - vTgt_.pos;
            if (diff.norm() > 1e-4) vTgt_.dir = diff.normalized();
            else vTgt_.dir = Eigen::Vector3d(1.0, 0.0, 0.0);          // 防止当真实目标点距离虚拟领航点的初始位置太近时导致dir计算出nan值
            vTgt_.vel = vTgt_.dir * cruise_speed_;
            UUV_INFO << "[Guidance] New Target Task (ID: " << latest_target_.id << ") Received!";
        }
        // 计算位置跟踪力
        Eigen::Vector3d self_pos(state.x, state.y, state.z);
        // 采用多项式平滑限幅，保证在远离目标时不会产生无限大的导航力
        Eigen::Vector3d Fnav_p = zeta_p_ * (vTgt_.pos - self_pos);         
        // 计算速度阻尼力
        Eigen::Vector3d vel_body(state.u, 0.0, 0.0); 
        Eigen::Vector3d self_vel = uuv_interface::bodyToWorld(vel_body, state.roll, state.pitch, state.yaw);
        Eigen::Vector3d Fnav_v = zeta_v_ * (vTgt_.vel - self_vel);
        return uuv_interface::softClampVec(Fnav_v+Fnav_p, nav_force_limit_, t_ftotal_, false);
    }


    // =========================================================
    // 子函数 2：算 3D 集群邻居内力 (Flocking Force)
    // =========================================================
    Eigen::Vector3d computeFlockingForce(const uuv_interface::State3D& state, double dt) {
        Eigen::Vector3d self_pos(state.x, state.y, state.z);
        double total_weight = 0.0;
        Eigen::Vector3d total_force = Eigen::Vector3d::Zero();

        // 若无邻居，则直接返回平滑后的力，这个力将逐渐趋近于0
        const auto& neighbors = this->latest_neighbors_.neighbors; 
        if (neighbors.empty()) {
            if (!std::isnan(latest_f_flock_.x())) {
                total_force = (1.0 - force_lp_gain_) * latest_f_flock_;
            }
            return total_force;
        }

        // A. 第一轮遍历：找到最近邻距离，用于高斯权重注意力机制
        double min_dist = 9999.0;
        for (const auto& nb : neighbors) {
            if (nb.distance < min_dist) {
                min_dist = nb.distance;
            }
        }

        // B. 第二轮遍历：计算各邻居作用力及软分配时的权重
        for (const auto& nb : neighbors) {
            double dist = nb.distance;
            if (dist < 0.1) dist = 0.1; // 防除零
            if (dist > r_comm_) continue;

            Eigen::Vector3d nb_pos(nb.state.x, nb.state.y, nb.state.z);
            Eigen::Vector3d vec_diff = self_pos - nb_pos; // 指向自身的排斥向量


            // 完美复刻 2D 版本的连续势场公式
            double f_val = 0.0;
            if (dist < delta_n_) {
                f_val = zeta_n_ * (std::exp(delta_n_ / dist) - std::exp(1.0));
            } else {
                f_val = k_pull_ * zeta_n_ * (std::exp(delta_n_ / dist) - std::exp(1.0));
            }

            // 异构编队逻辑：非同一目标的队友，只排斥，不产生吸引力
            // if (nb.state.target_id != current_target_id && f_val < 0) {
            //     f_val = 0.0;
            // }

            Eigen::Vector3d f_vec = f_val * vec_diff;

            // 基于 softmax 思想的高斯距离注意力机制
            double sigma = delta_n_ / 4.0;
            double dist_diff = dist - min_dist;
            double sigma_sq = sigma * sigma;
            double weight = std::exp(-(dist_diff * dist_diff) / sigma_sq);

            total_weight += weight;
            total_force += weight * f_vec;
        }

        // 归一化加权合力
        if (total_weight > 1e-6) {
            total_force /= total_weight;
        } else {
            total_force.setZero();
        }

        // C. 低通滤波平滑 (复用 latest_f_flock_ 状态)
        if (std::isnan(latest_f_flock_.x())) {
            latest_f_flock_ = total_force;
        }
        total_force = force_lp_gain_ * total_force + (1.0 - force_lp_gain_) * latest_f_flock_;
        
        // D. 极其重要的防溢出硬限幅 (基于我们在导航力讨论时的神仙打架原则)
        total_force = uuv_interface::softClampVec(total_force, flock_force_limit_, t_ftotal_, false);

        return total_force;
    }


    // =========================================================
    // 子函数 3：基于 2D 声纳的严格共面 3D 避障力 (Strictly Planar Obstacle Avoidance)
    // =========================================================
    Eigen::Vector3d computeObstacleForce(const uuv_interface::State3D& state) {
        Eigen::Vector3d f_obs_total(0, 0, 0);
        
        sensor_msgs::LaserScan scan = this->latest_sonar_;
        int total_rays = scan.ranges.size();
        if (total_rays == 0 || num_sectors_ <= 0) return f_obs_total;

        int rays_per_sector = total_rays / num_sectors_;
        double min_dist_global = 9999.0;
        std::vector<std::pair<double, double>> sector_minima;

        // 1. 扇区化最小池化 (与原逻辑保持一致)
        for (int i = 0; i < num_sectors_; ++i) {
            double min_dist = 9999.0;
            int min_index = -1;
            for (int j = 0; j < rays_per_sector; ++j) {
                int idx = i * rays_per_sector + j;
                if (idx >= total_rays) break;
                
                double r = scan.ranges[idx];
                if (std::isnan(r) || std::isinf(r)) continue;
                
                if (r > scan.range_min && r < scan.range_max && r < d_sense_) {
                    if (r < min_dist) {
                        min_dist = r;
                        min_index = idx;
                    }
                }
            }
            if (min_index != -1) {
                double angle = scan.angle_min + min_index * scan.angle_increment;
                sector_minima.push_back({min_dist, angle});
                if (min_dist < min_dist_global) min_dist_global = min_dist;
            }
        }

        if (sector_minima.empty()) {
            latest_f_obs_ = f_obs_total;
            return f_obs_total;
        }

        // 2. 坐标系转换：提取声纳平面特征
        Eigen::Matrix3d R_body_to_world = (Eigen::AngleAxisd(state.yaw, Eigen::Vector3d::UnitZ()) *
                                           Eigen::AngleAxisd(state.pitch, Eigen::Vector3d::UnitY()) *
                                           Eigen::AngleAxisd(state.roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
        
        Eigen::Vector3d body_forward(1.0, 0.0, 0.0);
        Eigen::Vector3d body_z_down(0.0, 0.0, 1.0);
        
        // 机体朝向 (在世界坐标系下)
        Eigen::Vector3d world_heading = R_body_to_world * body_forward;
        // 【核心修改1】：提取严格垂直于当前声纳平面的法向量 (Sonar Plane Normal)
        Eigen::Vector3d sonar_plane_normal = R_body_to_world * body_z_down; 

        // 3. 计算群体避让趋势
        Eigen::Vector3d avoid_trend_sum(0, 0, 0);
        std::vector<Eigen::Vector3d> world_obs_vectors;
        std::vector<double> obs_distances;

        for (auto& min_pt : sector_minima) {
            double dist = min_pt.first;
            double angle = min_pt.second;
            
            Eigen::Vector3d body_obs_dir(cos(angle), sin(angle), 0.0);
            Eigen::Vector3d world_obs_dir = R_body_to_world * body_obs_dir;
            
            world_obs_vectors.push_back(world_obs_dir);
            obs_distances.push_back(dist);

            double dist_diff = dist - min_dist_global;
            double weight = std::exp(-(dist_diff * dist_diff) / 100.0); // sigma=10
            
            avoid_trend_sum += world_heading.cross(world_obs_dir) * weight;
        }

        // 平滑绕行趋势
        double alpha_obs = 0.9;
        avoid_dir_smooth_ = alpha_obs * avoid_dir_smooth_ + (1.0 - alpha_obs) * avoid_trend_sum;

        // 【核心修改2】：强制将平滑后的趋势投影回当前声纳平面的法向量上！
        // 这样可以彻底过滤掉由于姿态变化引起的、脱离声纳平面的非物理趋势。
        double projection = avoid_dir_smooth_.dot(sonar_plane_normal);
        if (std::abs(projection) < 0.1) {
            projection = 1.0; // 默认向右侧绕行 (NED坐标系中，Z轴正方向为下，即右手定则的右转)
        }
        
        // 旋转轴被死死锁定为声纳平面的法向量 (要么指下，要么指上)
        Eigen::Vector3d rotate_axis = (projection > 0) ? sonar_plane_normal : -sonar_plane_normal;


        // 4. 计算各个扇区的势场力
        for (size_t i = 0; i < world_obs_vectors.size(); ++i) {
            double dist = obs_distances[i];
            Eigen::Vector3d vec_to_obs = world_obs_vectors[i];

            double abs_cos = std::abs(world_heading.dot(vec_to_obs));
            double w_norm = zeta_norm_min_ + (zeta_norm_max_ - zeta_norm_min_) * abs_cos;
            double w_tan  = zeta_tan_min_  + (zeta_tan_max_  - zeta_tan_min_)  * abs_cos;

            double dist_diff = dist - min_dist_global;
            double w_dist = std::exp(-(dist_diff * dist_diff) / 100.0);
            w_norm *= w_dist;
            w_tan *= w_dist;

            // 【核心修改3】：恢复 1.0 的硬限制，彻底消灭指数爆炸！
            double dist_inflation = dist - d_inflation_;
            if (dist_inflation < 1.0) dist_inflation = 1.0; 
            
            double force_mag = zeta_obs_ * dist_inflation * (std::exp(delta_obs_ / dist_inflation) - std::exp(1.0));
            if (force_mag < 0.0) force_mag = 0.0;
            
            // 为了防止求和前单根射线的力就已经溢出，预先在循环内对其硬限幅 (借用 flock_force_limit_)
            if (force_mag > flock_force_limit_) force_mag = flock_force_limit_;

            // 法向排斥力 (反向推开)
            Eigen::Vector3d f_normal = -1.0 * vec_to_obs * force_mag * w_norm;
            
            // 【核心修改4】：修复叉乘顺序！ vec_to_obs 叉乘 rotate_axis，产生正确的逃离切向力
            Eigen::Vector3d t_dir = vec_to_obs.cross(rotate_axis).normalized();
            Eigen::Vector3d f_tangent = t_dir * force_mag * w_tan;

            f_obs_total += (f_normal + f_tangent);
        }

        // 5. 非对称滤波平滑
        double alpha_decay = 0.5;
        if (f_obs_total.norm() < latest_f_obs_.norm()) {
            f_obs_total = (1.0 - alpha_decay) * f_obs_total + alpha_decay * latest_f_obs_;
        }
        latest_f_obs_ = f_obs_total;

        // 6. 最终安全限幅
        if (f_obs_total.norm() > flock_force_limit_) {
            f_obs_total = f_obs_total.normalized() * flock_force_limit_;
        }

        return f_obs_total;
    }




    // =========================================================
    // 子函数 3：指令映射 (Vector Field Mapping)
    // =========================================================
    uuv_interface::Cmd3D mapForceToCmd(const Eigen::Vector3d& f_total, const uuv_interface::State3D& state) {
        uuv_interface::Cmd3D out;
        Eigen::Vector3d tau = vTgt_.dir.normalized();
        double f_lon = f_total.dot(tau);
        Eigen::Vector3d f_lat = f_total - f_lon * tau;

        double dynamic_forward = std::max(f_lon, forward_tendency_);
        
        Eigen::Vector3d V_cmd = tau * dynamic_forward + f_lat;

        // UUV_INFO << "f_total: " << f_total.norm();
        // UUV_INFO << "f_total: " << f_total(0) << "," << f_total(1) << "," << f_total(2);

        // Eigen::Vector3d V_cmd = f_total;
        double V_norm = V_cmd.norm();
        if (V_norm < 1e-4) {  // 极低速/零力保护(防除零保护)
            out.target_pitch = 0.0;
            out.target_yaw = state.yaw;
        } else {
            out.target_yaw = std::atan2(V_cmd.y(), V_cmd.x());
            // NED坐标系，z轴向下，俯仰角取反
            double raw_pitch = std::atan2(-V_cmd.z(), std::hypot(V_cmd.x(), V_cmd.y()));
            // 俯仰角区间平滑限幅 [-max_pitch, max_pitch]
            double pitch_transition = max_pitch_rad_/6.0;
            out.target_pitch = uuv_interface::softClampScl(raw_pitch, -max_pitch_rad_, max_pitch_rad_, pitch_transition);
        }

        Eigen::Vector3d body_forward(1.0, 0.0, 0.0); 
        Eigen::Vector3d heading_vec = uuv_interface::bodyToWorld(body_forward, state.roll, state.pitch, state.yaw);
        
        // 计算合力在当前机头朝向上的投影大小
        double target_speed;
        double f_proj = f_total.dot(heading_vec);
        
        // 【核心修改】：以 cruise_speed_ 为零力平衡点进行分段线性映射
        if (f_proj >= 0.0) {
            // 受正向拉力：在 巡航速度 和 最大速度 之间加速追赶
            target_speed = cruise_speed_ + (f_proj / speed_map_scale_) * (max_speed_ - cruise_speed_);
        } else {
            // 受反向推力：在 巡航速度 和 最小速度 之间减速等待 (注意 f_proj 是负数，所以相当于减去)
            target_speed = cruise_speed_ + (f_proj / speed_map_scale_) * (cruise_speed_ - min_speed_);
        }
        
        out.target_u = uuv_interface::softClampScl(target_speed, min_speed_, max_speed_, 0.4);

        return out; 
    }
    

public:
    void initPlugin(ros::NodeHandle& gnh, const std::string& plugin_xml) override {
        this->registerSensorSubscribers();
        std::string ns = get_ns();
        local_frame_id_ = ns.empty() ? "base_link" : (ns + "/base_link");

        uuv_interface::XmlParamReader reader(plugin_xml);
        reader.param("min_speed", min_speed_, 0.5);      
        reader.param("cruise_speed", cruise_speed_, 2.0); // 编队平衡时的标准航速
        reader.param("max_speed", max_speed_, 3.0);       // [新增] 允许追赶的极限航速
        
        double max_pitch_deg;
        reader.param("max_pitch_deg", max_pitch_deg, 30.0);
        max_pitch_rad_ = max_pitch_deg / 180.0 * M_PI;
        reader.param("t_ftotal", t_ftotal_, 0.5);
        reader.param("forward_tendency", forward_tendency_, 30);
        reader.param("speed_map_scale", speed_map_scale_, 30);

        reader.param("zeta_p", zeta_p_, 1.5);
        reader.param("zeta_v", zeta_v_, 0.6);
        reader.param("nav_force_limit", nav_force_limit_, 30);

        reader.param("zeta_n", zeta_n_, 0.06);
        reader.param("delta_n", delta_n_, 30.0);
        reader.param("k_pull", k_pull_, 0.33);
        reader.param("r_comm", r_comm_, 100.0);
        reader.param("force_lp_gain", force_lp_gain_, 0.2);
        reader.param("flock_force_limit", flock_force_limit_, 2001);


        reader.param("d_sense", d_sense_, 200.0);
        reader.param("d_inflation", d_inflation_, 10.0);
        reader.param("zeta_obs", zeta_obs_, 0.5);
        reader.param("delta_obs", delta_obs_, 80.0);
        reader.param("num_sectors", num_sectors_, 8); // 将2D声纳均分为8个扇区处理
        std::vector<double> range_zeta_norm, range_zeta_tan;
        reader.param("range_zeta_norm", range_zeta_norm, std::vector<double>{0.4, 1.6});
        reader.param("range_zeta_tan", range_zeta_tan, std::vector<double>{0.4, 1.6});
        zeta_norm_min_ = (range_zeta_norm.size() >= 2) ? range_zeta_norm[0] : 0.4;
        zeta_norm_max_ = (range_zeta_norm.size() >= 2) ? range_zeta_norm[1] : 1.6;
        zeta_tan_min_ = (range_zeta_tan.size() >= 2) ? range_zeta_tan[0] : 0.4;
        zeta_tan_max_ = (range_zeta_tan.size() >= 2) ? range_zeta_tan[1] : 1.6;


        double cn, ce, cd;
        gnh.param("start_n", cn, 0.0);
        gnh.param("start_e", ce, 0.0);
        gnh.param("start_d", cd, 0.0);
        
        vTgt_.pos = Eigen::Vector3d(cn, ce, cd);
        vTgt_.vel.setZero();
        vTgt_.dir = Eigen::Vector3d(1.0, 0.0, 0.0); 

        UUV_INFO << "[SpacialFlockingGuidance] SpacialFlockingGuidance param Loaded: \n"
                 <<"\n min_speed=\n"<<min_speed_<<"\n cruise_speed=\n"<<cruise_speed_<<"\n max_speed=\n"<<max_speed_<<"\n max_pitch_deg=\n"<<max_pitch_deg
                 <<"\n t_ftotal=\n"<<t_ftotal_<<"\n forward_tendency=\n"<<forward_tendency_<<"\n speed_map_scale=\n"<<speed_map_scale_
                 <<"\n zeta_p=\n"<<zeta_p_<<"\n zeta_v=\n"<<zeta_v_<<"\n nav_force_limit=\n"<<nav_force_limit_
                 <<"\n zeta_n=\n"<<zeta_n_<<"\n delta_n=\n"<<delta_n_<<"\n k_pull=\n"<<k_pull_<<"\n r_comm=\n"<<r_comm_<<"\n force_lp_gain=\n"<<force_lp_gain_<<"\n flock_force_limit=\n"<<flock_force_limit_
                 <<"\n d_sense=\n"<<d_sense_<<"\n d_inflation=\n"<<d_inflation_<<"\n zeta_obs=\n"<<zeta_obs_<<"\n delta_obs=\n"<<delta_obs_<<"\n num_sectors=\n"<<num_sectors_
                 <<"\n range_zeta_norm=\n("<<zeta_norm_min_<<","<<zeta_norm_max_<<")\n range_zeta_tan=\n"<<zeta_tan_min_<<","<<zeta_tan_max_<<")"
                 <<"\n start_pos=("<<cn<<","<<ce<<","<<cd<<")";
    }

    virtual void initPublishDebug() override {
        debug_marker_pub_ = get_nh().advertise<visualization_msgs::MarkerArray>("guidance_debug_markers", 1);
    }
    
    void publishDebug(const ros::Time& time) override {
        // 如果 RViz 没有订阅，为了节省 CPU 就不计算了
        if (debug_marker_pub_.getNumSubscribers() == 0) return;

        visualization_msgs::MarkerArray msg;
        
        // 辅助函数：快速生成受力箭头
        auto make_arrow = [&](int id, const Eigen::Vector3d& world_force,
                              float r, float g, float b, const std::string& ns) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = local_frame_id_;
            marker.header.stamp = ros::Time(0);   // 0代表获取最新TF，消灭频闪
            marker.ns = ns;
            marker.id = id;
            marker.type = visualization_msgs::Marker::ARROW;
            
            // 如果力太小（接近 0），将其删除隐藏，保持界面干净
            if (world_force.norm() < 1e-3) {
                marker.action = visualization_msgs::Marker::DELETE;
                return marker;
            } 
            
            marker.action = visualization_msgs::Marker::ADD;
            Eigen::Vector3d local_force = uuv_interface::worldToBody(
                world_force, latest_state_.roll, latest_state_.pitch, latest_state_.yaw);


            geometry_msgs::Point p_start, p_end;
            p_start.x =0; p_start.y = 0; p_start.z = 0;
            p_end.x = local_force.x(); 
            p_end.y = local_force.y(); 
            p_end.z = local_force.z();
            marker.points.push_back(p_start);
            marker.points.push_back(p_end);

            // 尺寸: x=箭身粗细, y=箭头直径, z=箭头长度
            marker.scale.x = 0.1; marker.scale.y = 0.25; marker.scale.z = 0.25;
            marker.color.r = r; marker.color.g = g; marker.color.b = b; marker.color.a = 0.9;
            marker.lifetime = ros::Duration(0.5);
            return marker;
        };

        // 1. 发布导航力 (绿色 Green)
        msg.markers.push_back(make_arrow(0, latest_f_nav_, 0.0, 1.0, 0.0, "Force_Nav"));
        // 2. 发布集群内力 (蓝色 Blue)
        msg.markers.push_back(make_arrow(1, latest_f_flock_, 0.0, 0.0, 1.0, "Force_Flock"));
        // 3. 发布避障力 (红色 Red)
        msg.markers.push_back(make_arrow(2, latest_f_obs_, 1.0, 0.0, 0.0, "Force_Obs"));
        // 4. 发布总合力 (黑色 Black)
        msg.markers.push_back(make_arrow(3, latest_f_total_, 0.0, 0.0, 0.0, "Force_Total"));

        // 发布虚拟领航点 vTgt
        Eigen::Vector3d self_pos(latest_state_.x, latest_state_.y, latest_state_.z);
        Eigen::Vector3d relative_pos_world = vTgt_.pos - self_pos;
        // [核心简化] 将相对坐标转换到局部坐标系下
        Eigen::Vector3d relative_pos_local = uuv_interface::worldToBody(
            relative_pos_world, latest_state_.roll, latest_state_.pitch, latest_state_.yaw);

        visualization_msgs::Marker vTgt_marker;
        vTgt_marker.header.frame_id = local_frame_id_;
        vTgt_marker.header.stamp = ros::Time(0);
        vTgt_marker.ns = "Virtual_Target";
        vTgt_marker.id = 4;
        vTgt_marker.type = visualization_msgs::Marker::SPHERE;
        vTgt_marker.action = visualization_msgs::Marker::ADD;
        vTgt_marker.pose.position.x = relative_pos_local.x();
        vTgt_marker.pose.position.y = relative_pos_local.y();
        vTgt_marker.pose.position.z = relative_pos_local.z();
        vTgt_marker.pose.orientation.w = 1.0;

        vTgt_marker.scale.x = 2; 
        vTgt_marker.scale.y = 2;
        vTgt_marker.scale.z = 2;
        vTgt_marker.color.r = 0.0; vTgt_marker.color.g = 1.0; vTgt_marker.color.b = 1.0; vTgt_marker.color.a = 0.5;
        vTgt_marker.lifetime = ros::Duration(0.5);
        msg.markers.push_back(vTgt_marker);

        debug_marker_pub_.publish(msg);
    }

    uuv_interface::Cmd3D customUpdate(const uuv_interface::TargetPoint3D& target, const uuv_interface::State3D& state, double dt) override {
        latest_state_ = state;
        latest_self_pos_ = Eigen::Vector3d(state.x, state.y, state.z);
        // 0. 若目标（target.id < 0） 则停止
        if (target.id < 0) {
            uuv_interface::Cmd3D idle_cmd;
            idle_cmd.target_u = 0.0;         // 强制动力归零，无视 min_speed_
            idle_cmd.target_pitch = 0.0;     // 俯仰角回正
            idle_cmd.target_yaw = state.yaw; // 锁定下水时的朝向 (非常重要，防止原地鬼畜)
            return idle_cmd;
        }

        // 1. 计算导航力
        Eigen::Vector3d f_nav = computeNavForce(target, state, dt);
        Eigen::Vector3d f_flocking = computeFlockingForce(state, dt);
        // Eigen::Vector3d f_flocking      = Eigen::Vector3d::Zero(); // TODO: 预留给未来
        // Eigen::Vector3d f_apf      = Eigen::Vector3d::Zero(); // TODO: 预留给未来
        Eigen::Vector3d f_apf = computeObstacleForce(state);
        Eigen::Vector3d f_total = f_nav + f_flocking + f_apf;

        latest_f_nav_   = f_nav;
        latest_f_flock_ = f_flocking;
        latest_f_obs_   = f_apf;
        latest_f_total_   = f_total;
        
        // 2. 将合力注入速度矢量场并映射为指令
        return mapForceToCmd(f_total, state);
    }


};

} // namespace uuv_control

PLUGINLIB_EXPORT_CLASS(uuv_control::SpacialFlockingGuidance, uuv_interface::GuidanceBase)