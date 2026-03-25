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
#include <deque>

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
    double zeta_damp_;              // 邻居速度阻尼力
    double delta_n_;                // 平衡距离 (零力点)
    double k_pull_;                 // 吸引力削弱系数
    double r_comm_;                 // 感知半径
    double force_lp_gain_;          // 低通滤波平滑系数，越小越平滑
    double flock_force_limit_;      // 内力限幅

    // 用于平滑合力
    Eigen::Vector3d f_total_filtered_ = Eigen::Vector3d::Zero();


    double delay_tau_;       
    double max_delta_n_;     
    double k_delay_;         
    int est_window_size_;
    // === 封装与 2D 完全一致的 NeighborHistory 结构体 (10Hz 原生，无需任何锁) ===
    struct NeighborHistory3D {
        std::deque<Eigen::Vector3d> positions;
        std::deque<double> times;
        double latest_dynamic_delta_n = -1.0;
        void add(const Eigen::Vector3d& pos, double t, int max_size) {
            positions.push_back(pos);
            times.push_back(t);
            if (positions.size() > max_size) {
                positions.pop_front();
                times.pop_front();
            }
        }
    };
    std::map<std::string, NeighborHistory3D> nb_histories_;


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
    double sonar_delay_ = 0.0;      // 声呐综合时延
    bool is_filter_initialized_ = false;
    std::vector<double> memory_grid_;
    std::vector<std::deque<double>> sliding_windows_;
    ros::Publisher compensated_scan_pub_; // 发布运动学补偿后的最终声纳数据
    double sonar_update_rate_ = 10.0;   // 声纳更新率
    double last_yaw_ = 0.0;             // 记录上一帧的机体偏航角
    double yaw_drift_ = 0.0;            // 累积的角度漂移池

    // std::map<std::string, uuv_interface::LeastSquaresPredictor3D> nb_predictors_;

    Eigen::Vector3d avoid_dir_smooth_{0, 0, 0}; // 平滑绕行轴

    VirtualTarget3D vTgt_;

    double target_start_time_ = 0.0;
    Eigen::Vector3d target_start_pos_{0, 0, 0};

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
        // vTgt_.pos += vTgt_.vel * dt;
        Eigen::Vector3d target_pos(target.n, target.e, target.d);
        // 目标发生变化时，更新虚拟领航点的运动方向
        if (!isSamePoint(target,latest_target_)) {
            latest_target_ = target;

            target_start_pos_ = vTgt_.pos;
            UUV_INFO << "vTgt.pos: " << vTgt_.pos;

            // b. 提取消息自带的全局绝对时间戳
            target_start_time_ = target.header.stamp.toSec();
            if (target_start_time_ < 0.001) { // 兜底：如果没发时间戳，用当前时间
                target_start_time_ = ros::Time::now().toSec();
            }

            // c. 计算运动方向和速度 (注意：这里用的是 target_start_pos_ 作为起点)
            Eigen::Vector3d diff = target_pos - target_start_pos_;
            if (diff.norm() > 1e-4) vTgt_.dir = diff.normalized();
            else vTgt_.dir = Eigen::Vector3d(1.0, 0.0, 0.0);
            vTgt_.vel = vTgt_.dir * cruise_speed_;
            
            UUV_INFO << "[Guidance] New Target (ID: " << latest_target_.id 
                     << ") Sync Time: " << target_start_time_;
        }
        // 计算位置跟踪力
        double current_time = ros::Time::now().toSec();
        double elapsed_time = current_time - target_start_time_;
        if (elapsed_time < 0.0) elapsed_time = 0.0;

        // 计算从起点开始的理论总位移
        Eigen::Vector3d expected_displacement = vTgt_.vel * elapsed_time;
        // 计算起点到真实目标点的总距离
        Eigen::Vector3d total_diff = target_pos - target_start_pos_;
        
        // 如果理论位移还没超过总距离，则更新绝对位置；否则直接钉在目标点上
        if (expected_displacement.norm() < total_diff.norm()) {
            vTgt_.pos = target_start_pos_ + expected_displacement;
        } else {
            vTgt_.pos = target_pos;

            // ✨ 核心修正：绝对不设为0！
            // 保持巡航速度大小，但强制让牵引速度的方向“死死盯住目标点”！
            Eigen::Vector3d dir_to_target = (target_pos - latest_self_pos_);
            if (dir_to_target.norm() > 0.01) {
                dir_to_target.normalize();
            } else {
                dir_to_target = vTgt_.dir; // 防止除零
            }
            
            vTgt_.vel = dir_to_target * cruise_speed_;

        }
        Eigen::Vector3d self_pos(state.x, state.y, state.z);
        // 采用多项式平滑限幅，保证在远离目标时不会产生无限大的导航力
        Eigen::Vector3d Fnav_p = zeta_p_ * (vTgt_.pos - self_pos);         
        // 计算速度阻尼力
        Eigen::Vector3d vel_body(state.u, state.v, state.w); 
        Eigen::Vector3d self_vel = uuv_interface::bodyToWorld(vel_body, state.roll, state.pitch, state.yaw);
        Eigen::Vector3d Fnav_v = zeta_v_ * (vTgt_.vel - self_vel);
        return uuv_interface::softClampVec(Fnav_v+Fnav_p, nav_force_limit_, t_ftotal_, false);
    }

    // =========================================================
    // 子函数 2：算 3D 集群邻居内力 (Flocking Force)
    // =========================================================
    // Eigen::Vector3d computeFlockingForce(const uuv_interface::TargetPoint3D& target, 
    //                                      const uuv_interface::State3D& state,
    //                                      const std::vector<uuv_interface::Neighbor3D>& neighbors, 
    //                                      double dt) {
    //     Eigen::Vector3d self_pos(state.x, state.y, state.z);
    //     double total_weight = 0.0;
    //     Eigen::Vector3d total_force = Eigen::Vector3d::Zero();

    //     // 若无邻居，则直接返回平滑后的力，这个力将逐渐趋近于0
    //     if (neighbors.empty()) {
    //         if (!std::isnan(latest_f_flock_.x())) {
    //             total_force = (1.0 - force_lp_gain_) * latest_f_flock_;
    //         }
    //         return total_force;
    //     }

    //     // A. 第一轮遍历：找到最近邻距离，用于高斯权重注意力机制
    //     double min_dist = 9999.0;
    //     for (const auto& nb : neighbors) {
    //         double dist = nb.passive_distance; // ✅ 核心修复：直接读取真正的相对距离！
    //         if (dist < 0.1) dist = 0.1;
    //         if (dist > r_comm_) continue;
            
    //         if (dist < min_dist) {
    //             min_dist = dist;
    //         }
    //     }
    //     Eigen::Matrix3d R_body_to_world = (Eigen::AngleAxisd(state.yaw, Eigen::Vector3d::UnitZ()) *
    //                                         Eigen::AngleAxisd(state.pitch, Eigen::Vector3d::UnitY()) *
    //                                         Eigen::AngleAxisd(state.roll, Eigen::Vector3d::UnitX())).toRotationMatrix();

    //     // B. 第二轮遍历：计算各邻居作用力及软分配时的权重
    //     for (const auto& nb : neighbors) {
    //         double dist = nb.passive_distance;
    //         if (dist < 0.1) dist = 0.1;
    //         if (dist > r_comm_) continue;

    //         // =========================================================
    //         // 🔴 核心修复：彻底抛弃 R_body_to_world！直接读取绝对坐标！
    //         // 这和你的 2D 代码 Eigen::Vector2d nb_pos(nb.state.n, nb.state.e); 完全一样！
    //         // =========================================================
    //         Eigen::Vector3d nb_pos(nb.passive_state.x, nb.passive_state.y, nb.passive_state.z);
            
    //         // 排斥方向向量，直接由绝对坐标相减得到，极其稳定！
    //         Eigen::Vector3d vec_diff = self_pos - nb_pos; 

    //         // --- 最小二乘法预测速度 (这里现在吃到的坐标和2D一样平滑了！) ---
    //         double current_time = ros::Time::now().toSec();
    //         Eigen::Vector3d nb_vel_est(0, 0, 0); 
            
    //         if (nb_histories_.find(nb.uuv_name) == nb_histories_.end()) {
    //             nb_histories_[nb.uuv_name] = NeighborHistory3D();
    //         }
    //         auto& hist = nb_histories_[nb.uuv_name];
    //         hist.add(nb_pos, current_time, est_window_size_);

    //         if (hist.positions.size() >= 3) {
    //             nb_vel_est = estimateVelocityLeastSquares3D(hist.positions, hist.times);
    //         } else if (hist.positions.size() == 2) {
    //             double dt = hist.times.back() - hist.times.front();
    //             if (dt > 1e-3) {
    //                 nb_vel_est = (hist.positions.back() - hist.positions.front()) / dt;
    //             }
    //         }
            
    //         if (nb_vel_est.norm() > cruise_speed_ * 1.5) {
    //             nb_vel_est = nb_vel_est.normalized() * cruise_speed_ * 1.5;
    //         }

    //         // --- 动态调节 delta_n ---
    //         Eigen::Vector3d body_vel(state.u, 0.0, 0.0);
    //         Eigen::Vector3d self_vel = uuv_interface::bodyToWorld(body_vel, state.roll, state.pitch, state.yaw);

    //         double target_dynamic_delta_n = delta_n_; 
    //         if (delay_tau_ > 0.001) {
    //             Eigen::Vector3d rel_vel = nb_vel_est - self_vel;
    //             // dir_nb_to_self 严格对齐 2D
    //             Eigen::Vector3d dir_nb_to_self = vec_diff.normalized(); 
    //             double closing_speed = rel_vel.dot(dir_nb_to_self);


    //             double deadzone = 0.0;
                
                
    //             if (closing_speed > deadzone) {
    //                 // 现在 closing_speed 极其稳定，乘以 50 也会平滑涨缩！
    //                 double buffer = (closing_speed-deadzone) * delay_tau_ * k_delay_;
    //                 target_dynamic_delta_n += buffer;
    //                 if (target_dynamic_delta_n > max_delta_n_) {
    //                     target_dynamic_delta_n = max_delta_n_;
    //                 }
    //             }
    //         }

    //         if (hist.latest_dynamic_delta_n < 0.0) {
    //             hist.latest_dynamic_delta_n = delta_n_; // 第一次运行，初始化为基准值
    //         }
    //         double alpha_delta = 0.9; // 滤波系数：值越小，力场撑开和收缩的过程越平滑
    //         double dynamic_delta_n = (1.0 - alpha_delta) * target_dynamic_delta_n + alpha_delta * hist.latest_dynamic_delta_n;
    //         hist.latest_dynamic_delta_n = dynamic_delta_n;

    //         // 完美复刻 2D 版本的连续势场公式
    //         double f_val = 0.0;
    //         if (dist < dynamic_delta_n) {
    //             f_val = zeta_n_ * (std::exp(dynamic_delta_n / dist) - std::exp(1.0));
    //         } else {
    //             f_val = k_pull_ * zeta_n_ * (std::exp(dynamic_delta_n / dist) - std::exp(1.0));
    //         }

    //         // 异构编队逻辑：非同一目标的队友，只排斥，不产生吸引力
    //         if (nb.state.target_id != target.id && f_val < 0) {
    //             f_val = 0.0;
    //         }

    //         Eigen::Vector3d f_vec = f_val * vec_diff;

    //         // 基于 softmax 思想的高斯距离注意力机制
    //         double sigma = dynamic_delta_n / 4.0;
    //         double dist_diff = dist - min_dist;
    //         double sigma_sq = sigma * sigma;
    //         double weight = std::exp(-(dist_diff * dist_diff) / sigma_sq);

    //         total_weight += weight;
    //         total_force += weight * f_vec;
    //     }

    //     // 归一化加权合力
    //     if (total_weight > 1e-6) {
    //         total_force /= total_weight;
    //     } else {
    //         total_force.setZero();
    //     }

    //     // C. 低通滤波平滑 (复用 latest_f_flock_ 状态)
    //     if (std::isnan(latest_f_flock_.x())) {
    //         latest_f_flock_ = total_force;
    //     }
    //     total_force = force_lp_gain_ * total_force + (1.0 - force_lp_gain_) * latest_f_flock_;
        
    //     // D. 极其重要的防溢出硬限幅 (基于我们在导航力讨论时的神仙打架原则)
    //     total_force = uuv_interface::softClampVec(total_force, flock_force_limit_, t_ftotal_, false);

    //     return total_force;
    // }



//     Eigen::Vector3d computeFlockingForce(const uuv_interface::TargetPoint3D& target, 
//         const uuv_interface::State3D& state,
//         const std::vector<uuv_interface::Neighbor3D>& neighbors, 
//         double dt) {
//             Eigen::Vector3d self_pos(state.x, state.y, state.z);
//             double total_weight = 0.0;
//             Eigen::Vector3d total_force = Eigen::Vector3d::Zero();

//             // 若无邻居，则直接返回平滑后的力，这个力将逐渐趋近于0
//             if (neighbors.empty()) {
//             if (!std::isnan(latest_f_flock_.x())) {
//             total_force = (1.0 - force_lp_gain_) * latest_f_flock_;
//             }
//             return total_force;
//             }

//             // A. 第一轮遍历：找到最近邻距离，用于高斯权重注意力机制
//             double min_dist = 9999.0;
//             for (const auto& nb : neighbors) {
//             if (nb.distance < min_dist) {
//             min_dist = nb.distance;
//             }
//             }

//             // B. 第二轮遍历：计算各邻居作用力及软分配时的权重
//             for (const auto& nb : neighbors) {
//             double dist = nb.distance;
//             if (dist < 0.1) dist = 0.1; // 防除零
//             if (dist > r_comm_) continue;

//             Eigen::Vector3d nb_pos(nb.state.x, nb.state.y, nb.state.z);
//             Eigen::Vector3d vec_diff = self_pos - nb_pos; // 指向自身的排斥向量


//             // 完美复刻 2D 版本的连续势场公式
//             double f_val = 0.0;
//             if (dist < delta_n_) {
//             f_val = zeta_n_ * (std::exp(delta_n_ / dist) - std::exp(1.0));
//             } else {
//             f_val = k_pull_ * zeta_n_ * (std::exp(delta_n_ / dist) - std::exp(1.0));
//             }

//             // 异构编队逻辑：非同一目标的队友，只排斥，不产生吸引力
//             if (nb.state.target_id != target.id && f_val < 0) {
//             f_val = 0.0;
//             }

//             Eigen::Vector3d f_vec = f_val * vec_diff;

//             // 基于 softmax 思想的高斯距离注意力机制
//             double sigma = delta_n_ / 4.0;
//             double dist_diff = dist - min_dist;
//             double sigma_sq = sigma * sigma;
//             double weight = std::exp(-(dist_diff * dist_diff) / sigma_sq);

//             total_weight += weight;
//             total_force += weight * f_vec;
//             }

//             // 归一化加权合力
//             if (total_weight > 1e-6) {
//             total_force /= total_weight;
//             } else {
//             total_force.setZero();
//             }

//             // C. 低通滤波平滑 (复用 latest_f_flock_ 状态)
//             if (std::isnan(latest_f_flock_.x())) {
//             latest_f_flock_ = total_force;
//             }
//             total_force = force_lp_gain_ * total_force + (1.0 - force_lp_gain_) * latest_f_flock_;

//             // D. 极其重要的防溢出硬限幅 (基于我们在导航力讨论时的神仙打架原则)
//             total_force = uuv_interface::softClampVec(total_force, flock_force_limit_, t_ftotal_, false);

//             return total_force;
//      }



    Eigen::Vector3d computeFlockingForce(const uuv_interface::TargetPoint3D& target, 
        const uuv_interface::State3D& state,
        const std::vector<uuv_interface::Neighbor3D>& neighbors, 
        double dt) {


        double current_time = ros::Time::now().toSec();
        Eigen::Vector3d body_vel(state.u, state.v, state.w);
        Eigen::Vector3d self_vel = uuv_interface::bodyToWorld(body_vel, state.roll, state.pitch, state.yaw);

        Eigen::Vector3d self_pos(state.x, state.y, state.z);
        double total_weight = 0.0;
        Eigen::Vector3d total_force = Eigen::Vector3d::Zero();

        // 若无邻居，则直接返回平滑后的力，这个力将逐渐趋近于0
        if (neighbors.empty()) {
            if (!std::isnan(latest_f_flock_.x())) {
                total_force = (1.0 - force_lp_gain_) * latest_f_flock_;
            }
            return total_force;
        }

        // A. 第一轮遍历：找到最近邻距离，用于高斯权重注意力机制
        double min_dist = 9999.0;
        for (const auto& nb : neighbors) {
            if (nb.passive_distance < min_dist) {
                min_dist = nb.passive_distance;
            }
        }

        // B. 第二轮遍历：计算各邻居作用力及软分配时的权重
        for (const auto& nb : neighbors) {
            // === [修正 1]：最先提取距离和绝对坐标，供后续所有逻辑使用 ===
            double dist = nb.passive_distance;
            if (dist < 0.1) dist = 0.1; // 防除零
            if (dist > r_comm_) continue;

            Eigen::Vector3d nb_pos(nb.passive_state.x, nb.passive_state.y, nb.passive_state.z);

            // --- 1. 基于邻居状态利用最小二乘法进行速度估计 ---
            Eigen::Vector3d nb_vel_est(0,0,0); 
            // 3D 消息中邻居名字叫 uuv_name
            if (nb_histories_.find(nb.uuv_name) == nb_histories_.end()) {
                nb_histories_[nb.uuv_name] = NeighborHistory3D();
            }
            auto& hist = nb_histories_[nb.uuv_name];
            
            // 此时 nb_pos 已经合法定义，可以直接安全推入队列
            if (hist.positions.empty()) {   
                hist.add(nb_pos, current_time, est_window_size_);
            } else {
                hist.add(nb_pos, current_time, est_window_size_);
            }

            // 最小二乘法估计邻居速度 (>=3帧)
            if (hist.positions.size() >= 3) {
                nb_vel_est = estimateVelocityLeastSquares3D(hist.positions, hist.times);
            } else if (hist.positions.size() == 2) {
                double dt_hist = hist.times.back() - hist.times.front();
                if (dt_hist > 1e-3) {
                    nb_vel_est = (hist.positions.back() - hist.positions.front()) / dt_hist;
                }
            }
            // 速度限幅
            if (nb_vel_est.norm() > max_speed_) {
                nb_vel_est = nb_vel_est.normalized() * max_speed_;
            }

            // === [核心改进 1：对最小二乘法估计出的速度进行时间序列低通滤波] ===
            static std::map<std::string, Eigen::Vector3d> smoothed_vel_map;
            std::string pair_key = local_frame_id_ + "_" + nb.uuv_name; 
            if (smoothed_vel_map.find(pair_key) == smoothed_vel_map.end()) {
                smoothed_vel_map[pair_key] = nb_vel_est;
            }
            double alpha_v = 0.1; 
            nb_vel_est = alpha_v * nb_vel_est + (1.0 - alpha_v) * smoothed_vel_map[pair_key];
            smoothed_vel_map[pair_key] = nb_vel_est;

            // === [核心修复：补全水声通信的物理时延，进行预测] ===
            Eigen::Vector3d predict_nb_pos = nb_pos;
            if (!hist.positions.empty() && hist.positions.size() >= 3) {
                double last_msg_time = hist.times.back();
                double dt_age = current_time - last_msg_time;
                
                double total_predict_time = delay_tau_ + dt_age;
                if (total_predict_time > 0.0 && total_predict_time < 10.0) {
                    predict_nb_pos = hist.positions.back() + nb_vel_est * total_predict_time;
                }
            }

            // === [核心改进 2：非对称平滑呼吸机制 (Asymmetric Smoothing)] ===
            double dynamic_delta_n = delta_n_; 
            if (k_delay_ > 0.001) {
                Eigen::Vector3d rel_vel = nb_vel_est - self_vel;
                Eigen::Vector3d dir_nb_to_self = (self_pos - predict_nb_pos).normalized();
                double closing_speed = rel_vel.dot(dir_nb_to_self);

                static std::map<std::string, double> smoothed_buffer_map;
                std::string pair_key_b = local_frame_id_ + "_" + nb.uuv_name;
                if (smoothed_buffer_map.find(pair_key_b) == smoothed_buffer_map.end()) {
                    smoothed_buffer_map[pair_key_b] = 0.0;
                }

                if (closing_speed > 0) {
                    // [正在接近]：缓慢“撑开”安全圈
                    double raw_buffer = closing_speed * k_delay_;
                    double alpha_inflate = 1.0; 
                    smoothed_buffer_map[pair_key_b] = alpha_inflate * raw_buffer + (1.0 - alpha_inflate) * smoothed_buffer_map[pair_key_b];
                    
                    dynamic_delta_n += smoothed_buffer_map[pair_key_b];
                    if (dynamic_delta_n > max_delta_n_) {
                        dynamic_delta_n = max_delta_n_;
                    }
                } else {
                    // [停止接近 / 正在远离]：平滑“泄压”
                    double alpha_deflate = 1.0; 
                    smoothed_buffer_map[pair_key_b] = (1.0 - alpha_deflate) * smoothed_buffer_map[pair_key_b];
                    
                    if (smoothed_buffer_map[pair_key_b] > 0.1) {
                        dynamic_delta_n += smoothed_buffer_map[pair_key_b];
                    } else {
                        smoothed_buffer_map[pair_key_b] = 0.0; 
                    }
                }
            }

            // --- 2. 计算原始力 (Raw Force) 严格对齐 2D ---
            // === [修正 2]：在这里唯一声明 vec_diff，并且使用的是推演后的预测位置 ===
            Eigen::Vector3d vec_diff = self_pos - predict_nb_pos; 
            double predicted_dist = (delay_tau_ < 0.001) ? dist : vec_diff.norm();
            if (predicted_dist < 0.1) predicted_dist = 0.1;
            
            double f_val = 0.0;
            
            if (predicted_dist < dynamic_delta_n) {
                f_val = zeta_n_ * (std::exp(dynamic_delta_n / predicted_dist) - std::exp(1.0));
            } else {
                f_val = k_pull_ * zeta_n_ * (std::exp(delta_n_ / predicted_dist) - std::exp(1.0));
            }

            if (nb.state.target_id != target.id && f_val < 0) f_val = 0.0;
            
            Eigen::Vector3d f_vec = f_val * vec_diff;
            
            // 加上基于速度差的阻尼力，抵消震荡
            if (std::abs(zeta_damp_) > 1e-4) {
                f_vec += zeta_damp_ * (nb_vel_est - self_vel);
            }

            // 权重分配：注意 sigma 使用动态圈，而差值依然使用最纯粹的未推演距离 dist，和 2D 一模一样！
            double sigma = dynamic_delta_n / 4.0;
            double dist_diff = dist - min_dist;
            double sigma_sq = std::pow(sigma, 2.0);
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




    Eigen::Vector3d computeObstacleForce(const uuv_interface::State3D& state) {
        Eigen::Vector3d f_obs_total(0, 0, 0);
        
        sensor_msgs::LaserScan scan = this->latest_sonar_;
        int total_rays = scan.ranges.size();
        if (total_rays == 0) return f_obs_total;

        // 【模块 A：初始化与自运动旋转前馈补偿 (Ego-motion Shift)】
        if (!is_filter_initialized_ || memory_grid_.size() != total_rays) {
            memory_grid_.assign(total_rays, scan.range_max - 0.1);
            sliding_windows_.assign(total_rays, std::deque<double>());
            is_filter_initialized_ = true;
            last_yaw_ = state.yaw;
            yaw_drift_ = 0.0;
        }

        // 1. 计算两帧之间的偏航角增量
        double current_yaw = state.yaw;
        double delta_yaw = current_yaw - last_yaw_;
        // 将角度归一化到 [-pi, pi] 之间
        delta_yaw = std::atan2(std::sin(delta_yaw), std::cos(delta_yaw)); 
        last_yaw_ = current_yaw;

        // 是否加滤波的开关
        bool use_filter1 = true;
        bool use_filter2 = true;
        int median_filter_window_ = 30;

        // 2. 将转角累加到漂移池中
        yaw_drift_ += delta_yaw;
        // 3. 计算需要平移的数组索引步长 (角位移 / 雷达角分辨率)
        int index_shift = std::round(yaw_drift_ / scan.angle_increment);
        // 4. 执行历史记忆的物理对齐平移！(流水灯机制)
        if (index_shift != 0) {
            std::vector<std::deque<double>> new_windows(total_rays, std::deque<double>());
            std::vector<double> new_memory(total_rays, scan.range_max - 0.1);
            for (int i = 0; i < total_rays; ++i) {
                // 如果UUV左转(yaw>0)，障碍物相对机体右移(在数组中即index变小)
                // 那么当前 i 位置，应该继承原先在它左侧(较大index)的历史数据。
                int old_i = i + index_shift; 
                
                if (old_i >= 0 && old_i < total_rays) {
                    new_windows[i] = sliding_windows_[old_i];
                    new_memory[i] = memory_grid_[old_i];
                } else {
                    // 用当前的真实观测值瞬间填满盲区！
                    // 这样即使转弯时边缘突然扫到墙，滤波器也会瞬间反应，绝不会变成200！
                    double current_raw = scan.ranges[i];
                    new_windows[i].assign(median_filter_window_, current_raw); 
                    new_memory[i] = current_raw;
                }
            }
            sliding_windows_ = std::move(new_windows);
            memory_grid_ = std::move(new_memory);
            // 扣除已经补偿掉的角度，保留不足一个索引的微小漂移
            yaw_drift_ -= index_shift * scan.angle_increment; 
        }
        

        // 【模块 B：两级级联滤波 (清洗原始脏数据)】
        std::vector<float> filtered_ranges(total_rays, scan.range_max);

        for (int i = 0; i < total_rays; ++i) {
            double z_t = scan.ranges[i];
            if (std::isnan(z_t) || std::isinf(z_t)) z_t = scan.range_max - 0.1;
            double z_spatial = z_t;
            if (use_filter1) {
                // 1. 空间孤立度评估 判断某个维度的数据和其两侧的数据之间差别是否太大
                // 首先求出和左右两侧数据的最小差距delta_d_min，若为两头的数据，则直接将单侧的数据作为delta_d_min
                double delta_d_min = scan.range_max - 0.1;
                if (i == 0) delta_d_min = std::abs(z_t - scan.ranges[i+1]);
                else if (i == total_rays - 1) delta_d_min = std::abs(z_t - scan.ranges[i-1]);
                else delta_d_min = std::min(std::abs(z_t - scan.ranges[i-1]), std::abs(z_t - scan.ranges[i+1]));
                // sigma_iso是实际测得距离的0.1倍，与实际测得的距离z_t相关，
                // 这决定了当实际测得的距离越大，那么我们给空间上的突变以更大的宽容度（从高斯核函数中体现）
                // 如果实际测得的距离很小，sigma_iso也很小，那么我们对空间上的突变宽容度就更低
                // 这样的话，能够更加关注近距离突然突脸的突变，而对那些远距离的突变不那么要紧，因为远距离处，声纳波束由于发散效应，可能本身就是真的有个小障碍物在那，而不是突变！
                double sigma_iso = 0.1 * z_t; 
                // w_spatial 就是根据空间距离突变度delta_d_min和距离相关容忍度sigma_iso共同决定的对该突变的关注度
                double w_spatial = std::exp(-(delta_d_min * delta_d_min) / (2.0 * sigma_iso * sigma_iso + 1e-6));
                // 若对该突变的关注度高，那么在过滤后的值中包含的测得的真实值z_t部分的占比就高，否则，历史值的占比更高
                z_spatial = w_spatial * z_t + (1.0 - w_spatial) * memory_grid_[i];
            }
            double m_t = z_spatial;
            if (use_filter2) {
                // 2. 时域滑动中值滤波
                if (use_filter1) sliding_windows_[i].push_back(z_spatial);
                else sliding_windows_[i].push_back(z_t);
                if (sliding_windows_[i].size() > median_filter_window_) sliding_windows_[i].pop_front();
                std::vector<double> sorted_win(sliding_windows_[i].begin(), sliding_windows_[i].end());
                std::sort(sorted_win.begin(), sorted_win.end());
                m_t = sorted_win[sorted_win.size() / 2];
            }
            memory_grid_[i] = m_t;
            filtered_ranges[i] = m_t;
        }



        // 【模块 C：严谨的时延解耦前馈补偿】
        double T_sensor = sonar_delay_; // 0.4s 硬件延时
        double T_filter = 0.0;          // 滤波导致的历史距离延时
        if (use_filter2) {
            T_filter = ((median_filter_window_ - 1) / 2.0) * (1.0 / sonar_update_rate_);
        }

        // 解耦预测核心逻辑：
        // 1. 平移补偿：必须包含 T_filter！因为过去的点在物理径向上没有跟上UUV的平移
        double delta_x = state.u * (T_sensor + T_filter);
        double delta_y = state.v * (T_sensor + T_filter);

        // 2. 旋转补偿：坚决只补 T_sensor！
        // 因为 T_filter 导致的旋转滞后，已经在【模块A】的物理平移中被彻底干掉了！
        double delta_psi = state.r * T_sensor + yaw_drift_;
        std::vector<float> compensated_ranges(total_rays, scan.range_max - 0.1);

        int effective_sectors = (num_sectors_ > 0 && num_sectors_ < total_rays) ? num_sectors_ : total_rays;
        std::vector<double> sector_min_dists(effective_sectors, 9999.0);
        std::vector<double> sector_min_angles(effective_sectors, 0.0);
        double min_dist_global = 9999.0;

        // --- 单次循环完成全部遍历任务 ---
        for (int idx = 0; idx < total_rays; ++idx) {
            double d_old = filtered_ranges[idx];
            if (d_old >= scan.range_max - 1.0) continue; 
            // 1. 时间穿越：运动学推演
            double angle_old = scan.angle_min + idx * scan.angle_increment;
            double x_old = d_old * std::cos(angle_old);
            double y_old = d_old * std::sin(angle_old);

            double x_temp = x_old - delta_x;
            double y_temp = y_old - delta_y;
            double x_new = x_temp * std::cos(-delta_psi) - y_temp * std::sin(-delta_psi);
            double y_new = x_temp * std::sin(-delta_psi) + y_temp * std::cos(-delta_psi);

            double d_new = std::sqrt(x_new * x_new + y_new * y_new);
            double angle_new = std::atan2(y_new, x_new);

            // 2. 映射回新的声纳索引
            int new_idx = std::round((angle_new - scan.angle_min) / scan.angle_increment);
            
            if (new_idx >= 0 && new_idx < total_rays) {
                // 任务 A：同步收集发布用的补偿数据 (保留最近距离即可)
                if (d_new < compensated_ranges[new_idx]) {
                    compensated_ranges[new_idx] = d_new;
                }
                // 任务 B：扇区化池化 (利用除法直接锁定目标扇区，无需二次遍历)
                int sector_idx = new_idx * effective_sectors / total_rays;
                if (sector_idx >= 0 && sector_idx < effective_sectors) {
                    if (d_new < d_sense_ && d_new < sector_min_dists[sector_idx]) {
                        sector_min_dists[sector_idx] = d_new;
                        // 记录该扇区当前最近障碍点的真实角度
                        sector_min_angles[sector_idx] = scan.angle_min + new_idx * scan.angle_increment;
                    }
                }
            }
        }
        // =======================================================
        // 3. 发布：将补偿了时延的干净数据打包发布到 RViz
        // =======================================================
        sensor_msgs::LaserScan comp_scan = scan; 
        comp_scan.ranges = compensated_ranges;      
        // compensated_scan_pub_.publish(comp_scan);   
        // 4. 汇总并输出 8 个扇区的特征向量用于物理受力结算
        std::vector<std::pair<double, double>> sector_minima;

        // 直接读取在上方单重循环中已经利用哈希除法瞬间池化好的极值，彻底消除二次遍历！
        for (int i = 0; i < effective_sectors; ++i) {
            if (sector_min_dists[i] < 9999.0) {
                sector_minima.push_back({sector_min_dists[i], sector_min_angles[i]});
                if (sector_min_dists[i] < min_dist_global) {
                    min_dist_global = sector_min_dists[i];
                }
            }
        }
        // 若扇区没有被正确获取，直接返回0避障力!!!
        if (sector_minima.empty()) {
            double alpha_decay = 0.9; // 保持与底部的衰减系数完全一致
            // 此时当前的计算力 f_obs_total 为 (0,0,0)，只需让历史力自然衰减
            if (f_obs_total.norm() < latest_f_obs_.norm()) {
                f_obs_total = (1.0 - alpha_decay) * f_obs_total + alpha_decay * latest_f_obs_;
            }
            latest_f_obs_ = f_obs_total;
            return f_obs_total;
        }

        // 【模块 D：坐标转换与平滑绕行轴计算 (完全保留原逻辑)】
        Eigen::Matrix3d R_body_to_world = (Eigen::AngleAxisd(state.yaw, Eigen::Vector3d::UnitZ()) *
                                           Eigen::AngleAxisd(state.pitch, Eigen::Vector3d::UnitY()) *
                                           Eigen::AngleAxisd(state.roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
        
        Eigen::Vector3d body_forward(1.0, 0.0, 0.0);
        Eigen::Vector3d body_z_down(0.0, 0.0, 1.0);
        Eigen::Vector3d world_heading = R_body_to_world * body_forward;
        Eigen::Vector3d sonar_plane_normal = R_body_to_world * body_z_down; 

        Eigen::Vector3d avoid_trend_sum(0, 0, 0);
        std::vector<Eigen::Vector3d> world_obs_vectors;
        std::vector<double> obs_distances;
        std::vector<double> obs_angles; // 储存真实角度，用于角度注意力计算

        for (auto& min_pt : sector_minima) {
            double dist = min_pt.first;
            double angle = min_pt.second;
            
            Eigen::Vector3d body_obs_dir(cos(angle), sin(angle), 0.0);
            Eigen::Vector3d world_obs_dir = R_body_to_world * body_obs_dir;
            
            world_obs_vectors.push_back(world_obs_dir);
            obs_distances.push_back(dist);
            obs_angles.push_back(angle);

            // 距离注意力：用于群体避让趋势的加权
            double dist_diff = dist - min_dist_global;
            double weight = std::exp(-(dist_diff * dist_diff) / 100.0); // sigma=10
            avoid_trend_sum += world_heading.cross(world_obs_dir) * weight;
        }

        double alpha_obs = 0.9;
        avoid_dir_smooth_ = alpha_obs * avoid_dir_smooth_ + (1.0 - alpha_obs) * avoid_trend_sum;

        double projection = avoid_dir_smooth_.dot(sonar_plane_normal);
        if (std::abs(projection) < 0.1) projection = 1.0; 
        Eigen::Vector3d rotate_axis = (projection > 0) ? sonar_plane_normal : -sonar_plane_normal;

        // 【模块 E：结合注意力机制的斥力解算】
        for (size_t i = 0; i < world_obs_vectors.size(); ++i) {
            double dist = obs_distances[i];
            double angle = obs_angles[i];
            Eigen::Vector3d vec_to_obs = world_obs_vectors[i];

            double abs_cos = std::abs(world_heading.dot(vec_to_obs));
            double w_norm = zeta_norm_max_;
            double w_tan  = zeta_tan_max_;

            // --- 注意力机制分离实现 ---
            // 1. 距离注意力 (Gaussian)：离最近点越远，权重越小
            double dist_diff = dist - min_dist_global;
            double w_dist = std::exp(-(dist_diff * dist_diff) / 100.0);
            
            // 2. 角度注意力 (Cosine)：越靠两侧边缘，权重越趋近于0！
            // 由于cos在 [-90°, 90°] 内为正，两端平滑下降到0，完美契合忽视边缘的要求
            double norm_angle = angle / scan.angle_max;
            double mapped_angle = norm_angle * (M_PI / 2.0);
            double w_angle = std::pow(std::max(0.0, std::cos(mapped_angle)), 2);
            w_angle = std::max(0.0, std::min(1.0, w_angle));
            // UUV_INFO << "angle_max: " << scan.angle_max << ", angle: " << angle << ", mapped_angle: " << mapped_angle << " w_angle: " << w_angle;

            // 将双重注意力附加到切向与法向权重上
            w_norm *= (w_dist * w_angle);
            w_tan  *= (w_dist * w_angle);

            // --- 原始核心斥力公式 ---
            double dist_inflation = dist - d_inflation_;
            if (dist_inflation < 1.0) dist_inflation = 1.0; 
            
            double force_mag = zeta_obs_ * dist_inflation * (std::exp(delta_obs_ / dist_inflation) - std::exp(1.0));
            if (force_mag < 0.0) force_mag = 0.0;
            if (force_mag > flock_force_limit_) force_mag = flock_force_limit_;

            Eigen::Vector3d f_normal = -1.0 * vec_to_obs * force_mag * w_norm;
            Eigen::Vector3d t_dir = vec_to_obs.cross(rotate_axis).normalized();
            Eigen::Vector3d f_tangent = t_dir * force_mag * w_tan;

            f_obs_total += (f_normal + f_tangent);
        }

        // 5. 对称滤波平滑 (原样保留)
        double alpha_decay = 0.99;
        if (f_obs_total.norm() != latest_f_obs_.norm()) {
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
    // 子函数 3：指令映射 (Vector Field Mapping)   版本1,集群能稳定（forward_tendency_设置为30），但是单UUV无法抵达目标点
    // =========================================================
    // uuv_interface::Cmd3D mapForceToCmd(const Eigen::Vector3d& f_total, const uuv_interface::State3D& state) {
    //     uuv_interface::Cmd3D out;
    //     Eigen::Vector3d tau = vTgt_.dir.normalized();
    //     double f_lon = f_total.dot(tau);
    //     Eigen::Vector3d f_lat = f_total - f_lon * tau;

    //     double dynamic_forward = std::max(f_lon, forward_tendency_);
        
    //     Eigen::Vector3d V_cmd = tau * dynamic_forward + f_lat;

    //     // UUV_INFO << "f_total: " << f_total.norm();
    //     // UUV_INFO << "f_total: " << f_total(0) << "," << f_total(1) << "," << f_total(2);

    //     // Eigen::Vector3d V_cmd = f_total;
    //     double V_norm = V_cmd.norm();
    //     if (V_norm < 1e-4) {  // 极低速/零力保护(防除零保护)
    //         out.target_pitch = 0.0;
    //         out.target_yaw = state.yaw;
    //     } else {
    //         out.target_yaw = std::atan2(V_cmd.y(), V_cmd.x());
    //         // NED坐标系，z轴向下，俯仰角取反
    //         double raw_pitch = std::atan2(-V_cmd.z(), std::hypot(V_cmd.x(), V_cmd.y()));
    //         // 俯仰角区间平滑限幅 [-max_pitch, max_pitch]
    //         double pitch_transition = max_pitch_rad_/6.0;
    //         out.target_pitch = uuv_interface::softClampScl(raw_pitch, -max_pitch_rad_, max_pitch_rad_, pitch_transition);
    //     }

    //     Eigen::Vector3d body_forward(1.0, 0.0, 0.0); 
    //     Eigen::Vector3d heading_vec = uuv_interface::bodyToWorld(body_forward, state.roll, state.pitch, state.yaw);
        
    //     // 计算合力在当前机头朝向上的投影大小
    //     double target_speed;
    //     double f_proj = f_total.dot(heading_vec);
        
    //     // 【核心修改】：以 cruise_speed_ 为零力平衡点进行分段线性映射
    //     if (f_proj >= 0.0) {
    //         // 受正向拉力：在 巡航速度 和 最大速度 之间加速追赶
    //         target_speed = cruise_speed_ + (f_proj / speed_map_scale_) * (max_speed_ - cruise_speed_);
    //     } else {
    //         // 受反向推力：在 巡航速度 和 最小速度 之间减速等待 (注意 f_proj 是负数，所以相当于减去)
    //         target_speed = cruise_speed_ + (f_proj / speed_map_scale_) * (cruise_speed_ - min_speed_);
    //     }
        
    //     out.target_u = uuv_interface::softClampScl(target_speed, min_speed_, max_speed_, 0.4);

    //     return out; 
    // }



    // =========================================================
    // 子函数 3：指令映射 (Vector Field Mapping)   版本2,单UUV能够抵达目标点，但是集群无法稳定
    // =========================================================
    // uuv_interface::Cmd3D mapForceToCmd(const Eigen::Vector3d& f_total, const uuv_interface::State3D& state) {
    //     uuv_interface::Cmd3D out;
        
    //     // =========================================================
    //     // 【核心修复】：移除 Vector Field 的 forward_tendency 逻辑
    //     // 让期望的航向矢量直接等于算出来的物理合力！
    //     // 这样当 UUV 越过目标点，合力向后时，机头也会老老实实地调转 180 度！
    //     // =========================================================
    //     Eigen::Vector3d V_cmd = f_total;

    //     double V_norm = V_cmd.norm();
    //     if (V_norm < 1e-4) {  // 极低速/零力保护(防除零保护)
    //         out.target_pitch = 0.0;
    //         out.target_yaw = state.yaw;
    //     } else {
    //         // 直接根据合力方向计算出期望的机体偏航角 (Yaw)
    //         out.target_yaw = std::atan2(V_cmd.y(), V_cmd.x());
            
    //         // NED坐标系，z轴向下，俯仰角取反计算 (Pitch)
    //         double raw_pitch = std::atan2(-V_cmd.z(), std::hypot(V_cmd.x(), V_cmd.y()));
    //         // 俯仰角区间平滑限幅 [-max_pitch, max_pitch]
    //         double pitch_transition = max_pitch_rad_/6.0;
    //         out.target_pitch = uuv_interface::softClampScl(raw_pitch, -max_pitch_rad_, max_pitch_rad_, pitch_transition);
    //     }

    //     Eigen::Vector3d body_forward(1.0, 0.0, 0.0); 
    //     Eigen::Vector3d heading_vec = uuv_interface::bodyToWorld(body_forward, state.roll, state.pitch, state.yaw);
        
    //     // 计算合力在当前机头实际朝向上的投影大小，用于控制油门 (Speed)
    //     double target_speed;
    //     double f_proj = f_total.dot(heading_vec);
        
    //     // 以 cruise_speed_ 为零力平衡点进行分段线性映射
    //     if (f_proj >= 0.0) {
    //         target_speed = cruise_speed_ + (f_proj / speed_map_scale_) * (max_speed_ - cruise_speed_);
    //     } else {
    //         target_speed = cruise_speed_ + (f_proj / speed_map_scale_) * (cruise_speed_ - min_speed_);
    //     }
        
    //     out.target_u = uuv_interface::softClampScl(target_speed, min_speed_, max_speed_, 0.4);

    //     return out; 
    // }


    // =========================================================
    // 子函数 3：指令映射 (回归最纯粹的 APF 物理牵引模型)
    // 采用你最成功的 2D 哲学：合力直接映射为速度与姿态，依靠自然落后距离牵引！
    // =========================================================
    uuv_interface::Cmd3D mapForceToCmd(const Eigen::Vector3d& f_total, const uuv_interface::State3D& state) {
        uuv_interface::Cmd3D out;
        
        // 合力指向哪，期望速度矢量就指向哪。避障斥力完全参与方向决策！
        Eigen::Vector3d V_cmd = f_total;

        double V_norm = V_cmd.norm();
        if (V_norm < 1e-4) {  
            out.target_pitch = 0.0;
            out.target_yaw = state.yaw;
            out.target_u = 0.0;
        } else {
            // 机头 100% 听从合力的方向
            // out.target_yaw = std::atan2(V_cmd.y(), V_cmd.x());

            // 1. 算出纯净合力对应的绝对目标航向
            double raw_target_yaw = std::atan2(V_cmd.y(), V_cmd.x());
            
            // 2. 计算航向误差，并严格归一化到 [-PI, PI]
            double yaw_err = raw_target_yaw - state.yaw;
            while (yaw_err > M_PI)  yaw_err -= 2.0 * M_PI;
            while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

            // 3. 防鬼畜侧滑逻辑
            if (yaw_err > M_PI / 2.0) {
                // 受力在左后方 -> 坚决不向左猛打180度！机头向右微调，保持前进姿态侧滑避让
                yaw_err = M_PI - yaw_err; 
            } else if (yaw_err < -M_PI / 2.0) {
                // 受力在右后方 -> 机头向左微调，侧滑避让
                yaw_err = -M_PI - yaw_err; 
            }

            // 4. 将安全处理后的误差加回当前机头，生成给底层 PID 的最终期望航向
            out.target_yaw = state.yaw + yaw_err;
            // 为防止加上误差后越界，再做一次标准的归一化
            out.target_yaw = std::atan2(std::sin(out.target_yaw), std::cos(out.target_yaw));



            double raw_pitch = std::atan2(-V_cmd.z(), std::hypot(V_cmd.x(), V_cmd.y()));
            
            // 俯仰角柔性限幅
            double pitch_transition = max_pitch_rad_ / 6.0;
            out.target_pitch = uuv_interface::softClampScl(raw_pitch, -max_pitch_rad_, max_pitch_rad_, pitch_transition);
            
            // 【核心神技 2】：3D 非完整约束的灵魂保护
            // 期望速度 = 合力矢量在当前【真实机头方向】上的投影
            // 这意味着：如果需要大转弯避障，UUV 会先减速，等机头转过去对准了安全方向，再一脚油门冲过去！
            // 完美防止 3D 空间下的“漂移”和超调。
            Eigen::Vector3d body_forward(1.0, 0.0, 0.0); 
            Eigen::Vector3d heading_vec = uuv_interface::bodyToWorld(body_forward, state.roll, state.pitch, state.yaw);
            
            double target_speed = V_cmd.dot(heading_vec);
            
            // 直接限幅输出实际油门指令！
            // 注意：下限使用 min_speed_ 保证 UUV 具备基础舵效
            out.target_u = uuv_interface::softClampScl(target_speed, min_speed_, max_speed_, 0.4);
        }

        return out; 
    }
    

public:

    // =========================================================
    // 独立算法层：3D 最小二乘法 (完全对齐 2D 版本逻辑)
    // =========================================================
    Eigen::Vector3d estimateVelocityLeastSquares3D(
        const std::deque<Eigen::Vector3d>& pos_hist, 
        const std::deque<double>& time_hist) 
    {
        size_t n = pos_hist.size();
        if (n < 2) return Eigen::Vector3d(0, 0, 0);

        double sum_t = 0.0, sum_t2 = 0.0;
        Eigen::Vector3d sum_p(0, 0, 0), sum_tp(0, 0, 0);
        double t0 = time_hist.front();

        for (size_t i = 0; i < n; ++i) {
            double t = time_hist[i] - t0;
            Eigen::Vector3d p = pos_hist[i];

            sum_t += t;
            sum_t2 += t * t;
            sum_p += p;
            sum_tp += t * p;
        }

        double denominator = n * sum_t2 - sum_t * sum_t;
        
        if (std::abs(denominator) < 1e-6) {
            double total_dt = time_hist.back() - time_hist.front();
            if (total_dt < 1e-4) return Eigen::Vector3d::Zero();
            return (pos_hist.back() - pos_hist.front()) / total_dt;
        }

        return (n * sum_tp - sum_t * sum_p) / denominator;
    }

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
        reader.param("zeta_damp", zeta_damp_, 0.0);
        reader.param("delta_n", delta_n_, 30.0);
        reader.param("k_pull", k_pull_, 0.33);
        reader.param("r_comm", r_comm_, 100.0);
        reader.param("force_lp_gain", force_lp_gain_, 0.2);
        reader.param("flock_force_limit", flock_force_limit_, 2001);

        reader.param("delay_tau", delay_tau_, 1.0); 
        reader.param("max_delta_n", max_delta_n_, 400.0);
        reader.param("k_delay", k_delay_, 50.0);
        reader.param("est_window_size", est_window_size_, 30);


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

        compensated_scan_pub_ = gnh.advertise<sensor_msgs::LaserScan>("compensated_sonar_scan", 1);


        UUV_INFO << "[aSpacialFlockingGuidance] SpacialFlockingGuidance param Loaded: \n"
                 <<"\n min_speed=\n"<<min_speed_<<"\n cruise_speed=\n"<<cruise_speed_<<"\n max_speed=\n"<<max_speed_<<"\n max_pitch_deg=\n"<<max_pitch_deg
                 <<"\n t_ftotal=\n"<<t_ftotal_<<"\n forward_tendency=\n"<<forward_tendency_<<"\n speed_map_scale=\n"<<speed_map_scale_
                 <<"\n zeta_p=\n"<<zeta_p_<<"\n zeta_v=\n"<<zeta_v_<<"\n nav_force_limit=\n"<<nav_force_limit_
                 <<"\n zeta_n=\n"<<zeta_n_<<"\n delta_n=\n"<<delta_n_<<"\n k_pull=\n"<<k_pull_<<"\n r_comm=\n"<<r_comm_<<"\n force_lp_gain=\n"<<force_lp_gain_<<"\n flock_force_limit=\n"<<flock_force_limit_
                 <<"\n delay_tau=\n"<<delay_tau_<<"\n max_delta_n=\n"<<max_delta_n_<<"\n k_delay=\n"<<k_delay_<<"\n est_window_size=\n"<<est_window_size_
                 <<"\n d_sense=\n"<<d_sense_<<"\n d_inflation=\n"<<d_inflation_<<"\n zeta_obs=\n"<<zeta_obs_<<"\n delta_obs=\n"<<delta_obs_<<"\n num_sectors=\n"<<num_sectors_
                 <<"\n range_zeta_norm=\n("<<zeta_norm_min_<<","<<zeta_norm_max_<<")\n range_zeta_tan=\n"<<zeta_tan_min_<<","<<zeta_tan_max_<<")"
                 <<"\n start_pos=("<<cn<<","<<ce<<","<<cd<<")";
    }

    virtual void initPublishDebug() override {
        debug_marker_pub_ = get_nh().advertise<visualization_msgs::MarkerArray>("guidance_debug_markers", 1);
    }


    Eigen::Vector3d getTargetDir() const override {
        return vTgt_.dir; 
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

    uuv_interface::Cmd3D customUpdate(const uuv_interface::TargetPoint3D& target, 
                                      const uuv_interface::State3D& state, 
                                      const std::vector<uuv_interface::Neighbor3D>& neighbors, 
                                      double dt) override {
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
        Eigen::Vector3d f_flocking = computeFlockingForce(target, state, neighbors, dt);
        // Eigen::Vector3d f_flocking      = Eigen::Vector3d::Zero(); // TODO: 预留给未来
        // Eigen::Vector3d f_apf      = Eigen::Vector3d::Zero(); // TODO: 预留给未来
        Eigen::Vector3d f_apf = computeObstacleForce(state);

        Eigen::Vector3d f_total_raw = f_nav + f_flocking + f_apf;
        // 3. ✨ 核心修复：引入合力低通滤波（虚拟惯性）
        double alpha_f = 0.90; // 滤波平滑系数 (取值 0.0 ~ 1.0)
        if (f_total_filtered_.norm() < 1e-6) {
            f_total_filtered_ = f_total_raw;
        } else {
            f_total_filtered_ = (1.0 - alpha_f) * f_total_raw +  alpha_f * f_total_filtered_;
        }
        // Eigen::Vector3d f_total = f_nav + f_flocking + f_apf;

        latest_f_nav_   = f_nav;
        latest_f_flock_ = f_flocking;
        latest_f_obs_   = f_apf;
        latest_f_total_   = f_total_filtered_;
        
        // 2. 将合力注入速度矢量场并映射为指令
        return mapForceToCmd(f_total_filtered_, state);
    }


};

} // namespace uuv_control

PLUGINLIB_EXPORT_CLASS(uuv_control::SpacialFlockingGuidance, uuv_interface::GuidanceBase)