#include <uuv_interface/DecisionBase.h>
#include <uuv_interface/Neighborhood3D.h>
#include <std_msgs/String.h>
#include <pluginlib/class_list_macros.h>
#include <sstream>
#include <cmath>
#include <limits>
#include <algorithm>
#include <random>
#include <iomanip>
#include <iostream>

namespace uuv_control {

struct BidInfo {
    std::string winner_id;
    double bid_value;
};

class CBAADecision : public uuv_interface::DecisionBase {
protected:
    std::string uuv_name_;
    int virtual_id_multiplier_;
    int gossip_size_;
    size_t last_known_task_count_;
    bool replan_needed_;
    double map_range_;
    int max_bundle_size_;

    std::map<int, BidInfo> consensus_map_;
    std::map<int, uuv_interface::TargetPoint3D> known_tasks_;
    std::vector<int> my_bundle_;

    // 用于分布式通信的侧信道
    ros::Subscriber sub_nb_;
    ros::Subscriber sub_bids_;
    ros::Publisher pub_bids_;

    std::vector<std::string> current_neighbors_;
    std::map<std::string, std::string> latest_neighbor_bids_;

    // 监听邻居话题，只为了获取当前哪些人在我的通信范围内
    void nbCallback(const uuv_interface::Neighborhood3D::ConstPtr& msg) {
        current_neighbors_.clear();
        for (const auto& nb : msg->neighbors) {
            current_neighbors_.push_back(nb.uuv_name);
        }
    }

    // 监听全局的竞标池，但后续处理时会根据 current_neighbors_ 进行物理距离过滤
    void bidsCallback(const std_msgs::String::ConstPtr& msg) {
        std::string raw = msg->data;
        size_t pos = raw.find('|');
        if (pos != std::string::npos) {
            std::string sender = raw.substr(0, pos);
            std::string json = raw.substr(pos + 1);
            latest_neighbor_bids_[sender] = json;
        }
    }

    void initPlugin(ros::NodeHandle& gnh, const std::string& plugin_xml) override {
        uuv_name_ = gnh.getNamespace();
        if(!uuv_name_.empty() && uuv_name_[0] == '/') uuv_name_ = uuv_name_.substr(1);

        ros::NodeHandle nh("~");
        nh.param("/config_decision_cbaa/max_bundle_size", max_bundle_size_, 1);
        nh.param("/config_decision_cbaa/cbba_virtual_id_scale", virtual_id_multiplier_, 100);
        nh.param("/config_decision_cbaa/cbba_gossip_size", gossip_size_, 5);
        
        map_range_ = 2000.0;
        last_known_task_count_ = 0;
        replan_needed_ = true;

        // 订阅自身专属的邻居话题
        sub_nb_ = gnh.subscribe("neighborhood", 1, &CBAADecision::nbCallback, this);
        // 订阅与发布全局的竞标侧信道话题
        sub_bids_ = gnh.subscribe("/swarm/cbaa_bids", 50, &CBAADecision::bidsCallback, this);
        pub_bids_ = gnh.advertise<std_msgs::String>("/swarm/cbaa_bids", 10);

        UUV_INFO << "[" << uuv_name_ << "] CBAADecision Init. Mode: " 
                 << (max_bundle_size_ == 1 ? "CBAA (Auction)" : "CBBA (Bundle)") << ". GossipSize: " << gossip_size_;
    }

    double calcScore(const uuv_interface::TargetPoint3D& task, const uuv_interface::State3D& state) {
        // 3D 距离计算
        double dn = task.n - state.x;
        double de = task.e - state.y;
        double dd = task.d - state.z;
        double dist = std::sqrt(dn*dn + de*de + dd*dd);
        
        double score = 0.0;
        if (dist < 1.0) score = std::pow(map_range_, 2);
        else score = std::pow(map_range_, 2) / std::pow(dist, 2.0);

        return score;
    }

    bool processNeighborData() {
        std::string tag = "[CBBA,";
        bool consensus_changed = false; 

        // 【关键】：严格按照当前的物理邻居列表过滤通信
        for (const std::string& nb_name : current_neighbors_) {
            if (latest_neighbor_bids_.find(nb_name) == latest_neighbor_bids_.end()) continue;
            
            const std::string& data = latest_neighbor_bids_[nb_name];
            size_t search_pos = 0;

            while ((search_pos = data.find(tag, search_pos)) != std::string::npos) {
                size_t end_pos = data.find(']', search_pos);
                if (end_pos == std::string::npos) break; 

                size_t content_start = search_pos + tag.length();
                std::string content = data.substr(content_start, end_pos - content_start);

                std::stringstream ss(content);
                std::string segment;
                std::vector<std::string> parts;
                while(std::getline(ss, segment, ',')) {
                    parts.push_back(segment);
                }

                if (parts.size() >= 3) {
                    try {
                        int nb_tid = std::stoi(parts[0]);
                        std::string nb_wid = parts[1];
                        double nb_bid = std::stod(parts[2]);

                        if (consensus_map_.find(nb_tid) != consensus_map_.end()) {
                            BidInfo& local_info = consensus_map_[nb_tid];
                            bool update = false;

                            bool is_conflict = (local_info.winner_id == uuv_name_ && nb_wid != uuv_name_);

                            if (nb_bid > local_info.bid_value + 1e-15) {
                                update = true;
                            } 
                            else if (std::abs(nb_bid - local_info.bid_value) <= 1e-15) {
                                if (nb_wid > local_info.winner_id) update = true;
                            }

                            if (is_conflict) {
                                if (update) {
                                    // ROS_INFO_STREAM("[" << uuv_name_ << "] LOST. I accept neighbor is better.");
                                } else {
                                    // ROS_INFO_STREAM("[" << uuv_name_ << "] IGNORED/STALEMATE.");
                                }
                            }

                            if (update) {
                                local_info.winner_id = nb_wid;
                                local_info.bid_value = nb_bid;
                                consensus_changed = true;
                            }
                        }
                    } catch (...) {}
                }
                search_pos = end_pos + 1;
            }
        }
        return consensus_changed;
    }

    void buildBundle(const uuv_interface::State3D& state) {
        // 复用你原本优秀的包含 Random Dispersion 的分组贪婪逻辑
        std::vector<int> valid_bundle;
        for (int vid : my_bundle_) {
            bool is_winner = (consensus_map_[vid].winner_id == uuv_name_);
            bool task_exists = (known_tasks_.find(vid) != known_tasks_.end());
            if (is_winner && task_exists) {
                valid_bundle.push_back(vid);
            } else {
                break; 
            }
        }
        my_bundle_ = valid_bundle;

        if (my_bundle_.size() >= max_bundle_size_) return;

        std::map<int, std::vector<int>> target_groups;
        for (const auto& kv : known_tasks_) {
            int vid = kv.first;
            int tid = vid / virtual_id_multiplier_; 
            
            bool already_owned = false;
            for(int owned : my_bundle_) if(owned == vid) already_owned = true;
            if(!already_owned) target_groups[tid].push_back(vid);
        }

        int best_target_vid = -1;
        double best_my_score = -1.0;

        for (const auto& entry : target_groups) {
            int tid = entry.first;
            const std::vector<int>& slots = entry.second;
            if (slots.empty()) continue;

            double min_winning_bid = 1e15;
            for (int vid : slots) {
                if (consensus_map_[vid].bid_value < min_winning_bid) {
                    min_winning_bid = consensus_map_[vid].bid_value;
                }
            }

            std::vector<int> candidates;
            for (int vid : slots) {
                if (std::abs(consensus_map_[vid].bid_value - min_winning_bid) <= 1e-15) {
                    candidates.push_back(vid);
                }
            }

            int weakest_vid = -1;
            std::string min_winner_id = "NONE";
            if (!candidates.empty()) {
                int my_id_hash = 0;
                try {
                    size_t last_digit_idx = uuv_name_.find_last_not_of("0123456789");
                    if (last_digit_idx != std::string::npos && last_digit_idx + 1 < uuv_name_.length()) {
                        my_id_hash = std::stoi(uuv_name_.substr(last_digit_idx + 1));
                    }
                } catch (...) { my_id_hash = 0; }
                
                int selected_idx = (my_id_hash + tid) % candidates.size();
                weakest_vid = candidates[selected_idx];
                min_winner_id = consensus_map_[weakest_vid].winner_id;
            }

            if (weakest_vid != -1) {
                double my_score = calcScore(known_tasks_.at(weakest_vid), state);
                double delta = my_score - min_winning_bid;

                bool i_win = false;
                if (delta > 1e-7) { 
                    i_win = true;
                } else if (std::abs(delta) <= 1e-7) {
                    if (uuv_name_ > min_winner_id) i_win = true;
                }

                if (i_win) {
                    if (my_score > best_my_score) {
                        best_my_score = my_score;
                        best_target_vid = weakest_vid;
                    }
                }
            }
        }

        if (best_target_vid != -1) {
            my_bundle_.push_back(best_target_vid);
            consensus_map_[best_target_vid].winner_id = uuv_name_;
            consensus_map_[best_target_vid].bid_value = best_my_score;
            // ROS_INFO_STREAM("[" << uuv_name_ << "] Picked Task " << best_target_vid << " Score: " << best_my_score);
        }
    }

    // ------------------------------------------------------------------
    // 核心覆写：Decision 层的 Update
    // ------------------------------------------------------------------
    uuv_interface::TargetPoint3D customUpdate(const uuv_interface::State3D& state, const uuv_interface::TargetPoint3DArray& mission_list, double dt) override {
        
        // 1. 拆解目标 (适应 3D TargetPoint3D)
        std::vector<int> current_observed_ids;
        std::map<int, uuv_interface::TargetPoint3D> current_task_map;
        int current_total_tasks = 0;

        for (const auto& t : mission_list.targets) {
            // 如果你忘了填 required_uuvs，这里默认至少需要 1 架
            int required = (t.required_uuvs > 0) ? (int)t.required_uuvs : 1; 
            current_total_tasks += required;
            
            for (int i = 0; i < required; ++i) {
                int vid = t.id * virtual_id_multiplier_ + i;
                current_observed_ids.push_back(vid);
                
                uuv_interface::TargetPoint3D virtual_task = t;
                virtual_task.id = vid; 
                current_task_map[vid] = virtual_task;

                if (consensus_map_.find(vid) == consensus_map_.end()) {
                    consensus_map_[vid] = BidInfo{"none", 0.0}; 
                    replan_needed_ = true; 
                }
            }
        }

        known_tasks_ = current_task_map;

        // 2. 移除丢失的目标
        if (!my_bundle_.empty()) {
            bool bundle_broken = false;
            for (int task_in_bundle : my_bundle_) {
                bool still_exists = false;
                for (int obs_id : current_observed_ids) {
                    if (obs_id == task_in_bundle) { still_exists = true; break; }
                }
                if (!still_exists) { bundle_broken = true; break; }
            }
            if (bundle_broken) {
                my_bundle_.clear();
                replan_needed_ = true;
            }
        }

        // 3. 清理共识表
        for (auto it = consensus_map_.begin(); it != consensus_map_.end(); ) {
            if (known_tasks_.find(it->first) == known_tasks_.end()) {
                it = consensus_map_.erase(it);
                replan_needed_ = true;
            } else ++it;
        }

        // 4. 处理邻居共识
        if (processNeighborData()) {
            replan_needed_ = true;
        }
        if (my_bundle_.size() < max_bundle_size_) {
            replan_needed_ = true;
        }

        // 5. 本地竞标打分
        if(replan_needed_) {
            buildBundle(state);
            replan_needed_ = false;
        }

        // 6. 构造输出 TargetPoint3D
        uuv_interface::TargetPoint3D result_target;
        if (!my_bundle_.empty()) {
            int my_tid = my_bundle_[0];
            if (known_tasks_.count(my_tid)) {
                 result_target = known_tasks_[my_tid];
                 // 还原为真实的物理 ID 交给 Guidance
                 result_target.id = result_target.id / virtual_id_multiplier_;   
            }
        } else {
            // 没有抢到任务，或者任务列表为空：原地待命 (-1)
            result_target.id = -1;
            result_target.n = state.x;
            result_target.e = state.y;
            result_target.d = state.z;
        }

        // 7. 发送自己的 Gossip 侧信道广播
        std::vector<int> tasks_to_broadcast;
        if (!my_bundle_.empty()) tasks_to_broadcast.push_back(my_bundle_[0]);

        std::vector<int> candidates;
        for (const auto& kv : consensus_map_) {
            if (!my_bundle_.empty() && kv.first == my_bundle_[0]) continue;     
            if (kv.second.bid_value > 0) candidates.push_back(kv.first);
        }

        if (!candidates.empty()) {
            std::random_device rd;
            std::mt19937 g(rd());
            std::shuffle(candidates.begin(), candidates.end(), g);
            for (int vid : candidates) {
                if ((int)tasks_to_broadcast.size() >= gossip_size_) break;
                tasks_to_broadcast.push_back(vid);
            }
        }

        std::stringstream ss_out;
        ss_out << std::fixed << std::setprecision(4);
        for (int vid : tasks_to_broadcast) {
            const BidInfo& info = consensus_map_[vid];
            ss_out << "[CBBA," << vid << "," << info.winner_id << "," << info.bid_value << "]";
        }

        // 发布自己的出价到全局池，其他邻居会在 processNeighborData 中处理
        std_msgs::String gossip_msg;
        gossip_msg.data = uuv_name_ + "|" + ss_out.str();
        pub_bids_.publish(gossip_msg);

        last_known_task_count_ = current_total_tasks;
        return result_target;
    }

public:
    void initPublishDebug() override {}
    void publishDebug(const ros::Time& time) override {}
};

} // namespace uuv_control

// 向 pluginlib 注册该 Decision 插件
PLUGINLIB_EXPORT_CLASS(uuv_control::CBAADecision, uuv_interface::DecisionBase)