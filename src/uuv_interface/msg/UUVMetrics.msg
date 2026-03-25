Header header

string uuv_name
float64 min_obs_dist            # 上帝视角下距离最近障碍物表面的真实物理距离
float64 min_nb_dist             # 上帝视角下距离最近邻居的真实物理距离
float64 cumulative_path_length  # 累计航程
float64 cumulative_projected_dist  # 有效投影推进位移
float64 detour_ratio               # 实时绕行度 (总航程 / 投影位移)
float64 rms_yaw_rate            # 偏航角速度均方根 (RMS-HVR)
