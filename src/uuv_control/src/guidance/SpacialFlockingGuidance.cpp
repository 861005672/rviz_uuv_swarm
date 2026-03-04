#include <uuv_interface/GuidanceBase.h>
#include <pluginlib/class_list_macros.h>
#include <uuv_interface/utils/XmlParamReader.h>
#include <uuv_interface/utils/utils.h>  
#include <Eigen/Dense>
#include <cmath>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

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
    double max_ftotal_;             // 合力的限幅，他决定了虚拟力的尺度
    double t_ftotal_;               // 合力的平滑限幅窗，值越大越平滑

    // --- 极简且完美的物理模型参数 ---
    // 导航力
    double zeta_p_;                 // 导航力位置跟踪系数
    double zeta_v_;                 // 导航力速度阻尼系数
    // 内力
    double zeta_n_;                 // 排斥力增益
    double delta_n_;                // 平衡距离 (零力点)
    double k_pull_;                 // 吸引力削弱系数
    double r_comm_;                 // 通信/感知半径

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
        return uuv_interface::softClampVec(Fnav_v+Fnav_p, max_ftotal_, t_ftotal_);
    }


    // =========================================================
    // 子函数 2：计算 3D 集群邻居内力 (Flocking Force)
    // =========================================================
    Eigen::Vector3d computeFlockingForce(const uuv_interface::State3D& state) {
        Eigen::Vector3d self_pos(state.x, state.y, state.z);
        double min_dist = 9999.0;
        double total_weight = 0.0;
        Eigen::Vector3d total_force = Eigen::Vector3d::Zero();

        // 假设父类中有 neighbors_ 成员变量 (如果不叫这个请修改)
        const auto& neighbors = this->latest_neighbors_.neighbors; 
        if (neighbors.empty()) return total_force;

        // A. 第一轮遍历：找到最近邻距离，用于高斯权重注意力机制
        for (const auto& nb : neighbors) {
            if (nb.distance < min_dist) {
                min_dist = nb.distance;
            }
        }

        // B. 第二轮遍历：计算各邻居作用力及软分配权重
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
        // if (std::isnan(latest_f_flock_.x())) {
        //     latest_f_flock_ = total_force;
        // }
        // total_force = force_lp_gain_ * total_force + (1.0 - force_lp_gain_) * latest_f_flock_;
        
        // D. 极其重要的防溢出硬限幅 (基于我们在导航力讨论时的神仙打架原则)
        // 这个 2000.0 足以在极限逼近时无情碾压 2.0 的导航力，又不会导致 double 崩溃
        if (total_force.norm() > 2000.0) {
            total_force = total_force.normalized() * 2000.0;
        }

        return total_force;
    }


    // =========================================================
    // 子函数 3：指令映射 (Vector Field Mapping)
    // =========================================================
    uuv_interface::Cmd3D mapForceToCmd(const Eigen::Vector3d& f_total, const uuv_interface::State3D& state) {
        uuv_interface::Cmd3D out;

        Eigen::Vector3d V_cmd = f_total;
        double V_norm = V_cmd.norm();
        if (V_norm < 1e-4) {  // 极低速保护(防除零保护)
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
        if (f_proj <= 0.0) {
            target_speed = min_speed_; 
        } else {
            target_speed = min_speed_ + (f_proj / max_ftotal_) * (max_speed_ - min_speed_);
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
        reader.param("max_ftotal", max_ftotal_, 2.0);         // 注意：现在的力直接等于速度偏移量。设为2.0意味着最多能提供 ±2.0m/s 的纠偏能力
        reader.param("t_ftotal", t_ftotal_, 0.5);

        reader.param("zeta_p", zeta_p_, 1.5);
        reader.param("zeta_v", zeta_v_, 0.6);

        reader.param("zeta_n", zeta_n_, 0.06);
        reader.param("delta_n", delta_n_, 30.0);
        reader.param("k_pull", k_pull_, 0.33);
        reader.param("r_comm", r_comm_, 100.0);

        double cn, ce, cd;
        gnh.param("start_n", cn, 0.0);
        gnh.param("start_e", ce, 0.0);
        gnh.param("start_d", cd, 0.0);
        
        vTgt_.pos = Eigen::Vector3d(cn, ce, cd);
        vTgt_.vel.setZero();
        vTgt_.dir = Eigen::Vector3d(1.0, 0.0, 0.0); 

        UUV_INFO << "[SpacialFlockingGuidance] SpacialFlockingGuidance param Loaded: \n"
                 <<"\n min_speed=\n"<<min_speed_<<"\n cruise_speed=\n"<<cruise_speed_
                 <<"\n max_speed=\n"<<max_speed_<<"\n max_pitch_deg=\n"<<max_pitch_deg
                 <<"\n zeta_p=\n"<<zeta_p_<<"\n zeta_v=\n"<<zeta_v_<<"\n max_ftotal=\n"
                 <<max_ftotal_<<"\n t_ftotal=\n"<<t_ftotal_<<"\n start_pos=("<<cn<<","<<ce<<","<<cd<<")";
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
        // 4. 发布总合力 (黄色 Yellow)
        msg.markers.push_back(make_arrow(3, latest_f_total_, 1.0, 1.0, 0.0, "Force_Total"));

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
        Eigen::Vector3d f_flocking = computeFlockingForce(state);
        Eigen::Vector3d f_apf      = Eigen::Vector3d::Zero(); // TODO: 预留给未来
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