#include <uuv_interface/DecisionBase.h>
#include <pluginlib/class_list_macros.h>
#include <sstream>
#include <cmath>
#include <limits>
#include <algorithm>
#include <random>
#include <iomanip>
#include <uuv_interface/utils/XmlParamReader.h>

namespace uuv_control {

struct BidInfo {
    std::string winner_id;
    double bid_value;
};

class CBAADecision : public uuv_interface::DecisionBase {
protected:
    std::string uuv_name_;
    int virtual_id_multiplier_ = 100;
    int gossip_size_ = 100;
    size_t last_known_task_count_ = 0;
    bool replan_needed_ = true;
    double map_range_ = 2000.0;
    int max_bundle_size_ = 1;

    std::map<int, BidInfo> consensus_map_;
    std::map<int, uuv_interface::TargetPoint3D> known_tasks_;
    std::vector<int> my_bundle_;

    void initPlugin(ros::NodeHandle& gnh, const std::string& plugin_xml) override {
        uuv_name_ = gnh.getNamespace();
        if(!uuv_name_.empty() && uuv_name_[0] == '/') uuv_name_ = uuv_name_.substr(1);

        uuv_interface::XmlParamReader reader(plugin_xml);
        
        reader.param("max_bundle_size", max_bundle_size_, 1);
        reader.param("virtual_id_multiplier", virtual_id_multiplier_, 100);
        reader.param("gossip_size", gossip_size_, 5);
        reader.param("map_range", map_range_, 2000.0);
        
        UUV_INFO << "[" << uuv_name_ << "] CBAADecision Init. Mode: " 
                 << (max_bundle_size_ == 1 ? "CBAA (Auction)" : "CBBA (Bundle)") 
                 << ". GossipSize: " << gossip_size_ << ", MapRange: " << map_range_;
    }

    double calcScore(const uuv_interface::TargetPoint3D& task, const uuv_interface::State3D& state) {
        double dn = task.n - state.x;
        double de = task.e - state.y;
        double dd = task.d - state.z;
        double dist = std::sqrt(dn*dn + de*de + dd*dd);
        
        double score = 0.0;
        if (dist < 1.0) score = std::pow(map_range_, 2);
        else score = std::pow(map_range_, 2) / std::pow(dist, 2.0);
        return score;
    }

    bool processNeighborData(const std::vector<uuv_interface::Neighbor3D>& neighbors) {
        std::string tag = "[CBBA,";
        bool consensus_changed = false; 

        for (const auto& nb : neighbors) {
            if (nb.state.data_json.empty()) continue; // 直接从状态中提取数据
            const std::string& data = nb.state.data_json;
            size_t search_pos = 0;

            while ((search_pos = data.find(tag, search_pos)) != std::string::npos) {
                size_t end_pos = data.find(']', search_pos);
                if (end_pos == std::string::npos) break; 
                size_t content_start = search_pos + tag.length();
                std::string content = data.substr(content_start, end_pos - content_start);

                std::stringstream ss(content);
                std::string segment;
                std::vector<std::string> parts;
                while(std::getline(ss, segment, ',')) parts.push_back(segment);

                if (parts.size() >= 3) {
                    try {
                        int nb_tid = std::stoi(parts[0]);
                        std::string nb_wid = parts[1];
                        double nb_bid = std::stod(parts[2]);

                        if (consensus_map_.find(nb_tid) != consensus_map_.end()) {
                            BidInfo& local_info = consensus_map_[nb_tid];
                            bool update = false;

                            if (nb_bid > local_info.bid_value + 1e-15) update = true;
                            else if (std::abs(nb_bid - local_info.bid_value) <= 1e-15) {
                                if (nb_wid > local_info.winner_id) update = true;
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
        // =========================================================
        // Step 1: 验证并清理已有 bundle
        // =========================================================
        std::vector<int> valid_bundle;
        for (int vid : my_bundle_) {
            bool is_winner = (consensus_map_[vid].winner_id == uuv_name_);
            bool task_exists = (known_tasks_.find(vid) != known_tasks_.end());
            if (is_winner && task_exists) {
                valid_bundle.push_back(vid);
            } else break; 
        }
        my_bundle_ = valid_bundle;
        if (my_bundle_.size() >= max_bundle_size_) return;

        // =========================================================
        // Step 2: 目标分组 & 准备调试头信息
        // =========================================================
        std::map<int, std::vector<int>> target_groups;
        for (const auto& kv : known_tasks_) {
            int vid = kv.first;
            int tid = vid / virtual_id_multiplier_; 
            bool already_owned = false;    // 过滤掉我已经拥有的任务
            for(int owned : my_bundle_) if(owned == vid) already_owned = true;
            if(!already_owned) target_groups[tid].push_back(vid);
        }

        // =========================================================
        // Step 3: 全局寻优 (引入随机离散化)
        // =========================================================
        int best_target_vid = -1;
        double best_my_score = -1.0;

        for (const auto& entry : target_groups) {
            int tid = entry.first;
            const std::vector<int>& slots = entry.second;
            if (slots.empty()) continue;

            // --- 3.1 寻找最弱链路 (Randomized Dispersion) ---
            // 核心目标：如果一堆空位分值一样低，不要只盯着ID最小的那个抢
            double min_winning_bid = 1e15;
            // 第一遍：找最小值
            for (int vid : slots) {
                double curr_bid = consensus_map_[vid].bid_value;
                if (curr_bid < min_winning_bid) min_winning_bid = curr_bid;
            }
            // 第二遍：收集所有等于最小值的“候选坑位”
            std::vector<int> candidates;
            for (int vid : slots) {
                double curr_bid = consensus_map_[vid].bid_value;
                if (std::abs(curr_bid - min_winning_bid) <= 1e-15) {
                    candidates.push_back(vid);
                }
            }
            // 第三步：随机挑选一个突破口
            int weakest_vid = -1;
            std::string min_winner_id = "NONE";
            if (!candidates.empty()) {
                // 简单的伪随机哈希，基于 UUV ID 和 候选列表长度
                // 确保不同 ID 的 UUV 倾向于选择不同的坑位
                int my_id_hash = 0;
                try {
                    size_t last_digit_idx = uuv_name_.find_last_not_of("0123456789");
                    if (last_digit_idx != std::string::npos && last_digit_idx + 1 < uuv_name_.length()) {
                        my_id_hash = std::stoi(uuv_name_.substr(last_digit_idx + 1));
                    }
                } catch (...) { my_id_hash = 0; }

                // 为了增加随机性，可以将 tid 也加入哈希计算，或者 candidates.size()
                int selected_idx = (my_id_hash + tid) % candidates.size();
                weakest_vid = candidates[selected_idx];
                min_winner_id = consensus_map_[weakest_vid].winner_id;
            }

            // --- 3.2 评估得分 ---
            if (weakest_vid != -1) {
                double my_score = calcScore(known_tasks_.at(weakest_vid), state);
                double delta = my_score - min_winning_bid;
                bool i_win = false;
                if (delta > 1e-7) i_win = true;
                else if (std::abs(delta) <= 1e-7 && uuv_name_ > min_winner_id) i_win = true;

                if (i_win && my_score > best_my_score) {
                    best_my_score = my_score;
                    best_target_vid = weakest_vid;
                }
            }
        }

        // =========================================================
        // Step 4: 执行决策
        // =========================================================
        if (best_target_vid != -1) {
            my_bundle_.push_back(best_target_vid);
            consensus_map_[best_target_vid].winner_id = uuv_name_;
            consensus_map_[best_target_vid].bid_value = best_my_score;
        }
    }

    // 更新签名以包含 data_json
    uuv_interface::TargetPoint3D customUpdate(const uuv_interface::State3D& state, 
                                              const uuv_interface::TargetPoint3DArray& mission_list, 
                                              const std::vector<uuv_interface::Neighbor3D>& neighbors, 
                                              std::string& data_json, 
                                              double dt) override {
        
        // ---------------------------------------------------------
        // Step 1: 【拆解任务】与【环境感知】
        // ---------------------------------------------------------
        std::vector<int> current_observed_ids;
        std::map<int, uuv_interface::TargetPoint3D> current_task_map;
        int current_total_tasks = 0;

        for (const auto& t : mission_list.targets) {
            int required = (t.required_uuvs > 0) ? (int)t.required_uuvs : 0;
            current_total_tasks += required;
            for (int i = 0; i < required; ++i) {
                int vid = t.id * virtual_id_multiplier_ + i;
                current_observed_ids.push_back(vid);
                current_task_map[vid] = t;
                current_task_map[vid].id = vid; 
                if (consensus_map_.find(vid) == consensus_map_.end()) {
                    consensus_map_[vid] = BidInfo{"none", 0.0}; 
                    replan_needed_ = true; 
                }
            }
        }
        known_tasks_ = current_task_map;
        // ---------------------------------------------------------
        // Step 2: 【自我审查】检查我当前的任务是否还存在
        // ---------------------------------------------------------
        if (!my_bundle_.empty()) {
            bool bundle_broken = false;
            for (int task_in_bundle : my_bundle_) {
                bool still_exists = false;
                for (int obs_id : current_observed_ids) {
                    if (obs_id == task_in_bundle) { still_exists = true; break; }
                }
                if (!still_exists) { bundle_broken = true; break; }
            }
            if (bundle_broken) { my_bundle_.clear(); replan_needed_ = true; }
        }
        // ---------------------------------------------------------
        // Step 3: 【清理战场】移除共识表中过期的任务
        // ---------------------------------------------------------
        for (auto it = consensus_map_.begin(); it != consensus_map_.end(); ) {
            int vid = it->first;
            if (known_tasks_.find(vid) == known_tasks_.end()) {
                it = consensus_map_.erase(it); replan_needed_ = true;
            } else ++it;
        }
        // ---------------------------------------------------------
        // Step 4: 【处理邻居数据】(共识更新)
        // ---------------------------------------------------------
        // 只有在没有发生 Reset 的情况下才完全信任邻居数据
        // 如果刚发生了 Reset，我们希望先基于本地感知建立初步竞标，再处理邻居数据
        bool consensus_changed = processNeighborData(neighbors);
        if (consensus_changed) replan_needed_ = true;
        if (my_bundle_.size() < max_bundle_size_) replan_needed_ = true;

        // ---------------------------------------------------------
        // Step 5: 【贪婪抢座】
        // ---------------------------------------------------------
        // 如果刚才发生了 reset，这里会根据当前 UUV 位置重新计算所有任务的 Score
        // 因为 consensus_map_ 里的 winner 都是 "none"，大家会公平竞争
        if(replan_needed_) {
            buildBundle(state);
            replan_needed_ = false;
        }

        // ---------------------------------------------------------
        // Step 6: 【输出结果】
        // ---------------------------------------------------------
        uuv_interface::TargetPoint3D result_target;
        if (!my_bundle_.empty() && known_tasks_.count(my_bundle_[0])) {
             result_target = known_tasks_[my_bundle_[0]];
             result_target.id = result_target.id / virtual_id_multiplier_;   
        } else {
            result_target.id = -1;
            result_target.n = state.x; result_target.e = state.y; result_target.d = state.z;
        }

        // ---------------------------------------------------------
        // Step 7: 【构造广播消息】(Gossip)
        // ---------------------------------------------------------
        std::vector<int> tasks_to_broadcast;
        if (!my_bundle_.empty()) tasks_to_broadcast.push_back(my_bundle_[0]);
        std::vector<int> candidates;
        for (const auto& kv : consensus_map_) {
            if (!my_bundle_.empty() && kv.first == my_bundle_[0]) continue;     
            if (kv.second.bid_value > 0) candidates.push_back(kv.first);
        }
        if (!candidates.empty()) {
            std::random_device rd; std::mt19937 g(rd());
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
        data_json = ss_out.str(); // 写入引用参数
        
        last_known_task_count_ = current_total_tasks;
        return result_target;
    }

public:
    void initPublishDebug() override {}
    void publishDebug(const ros::Time& time) override {}
};
} // namespace uuv_control
PLUGINLIB_EXPORT_CLASS(uuv_control::CBAADecision, uuv_interface::DecisionBase)