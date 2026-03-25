Header header

# === 位置 (NED 坐标系) ===
float64 x
float64 y
float64 z

# === 姿态 (欧拉角) ===
float64 roll    # Roll (横滚)
float64 pitch  # Pitch (俯仰)
float64 yaw    # Yaw (偏航)

# === 线速度 (机体坐标系) ===
float64 u      # Surge (纵荡)
float64 v      # Sway (横荡)
float64 w      # Heave (垂荡)

# === 角速度 (机体坐标系) ===
float64 p      # Roll rate
float64 q      # Pitch rate
float64 r      # Yaw rate

int32 target_id
string data_json


# 虚拟领航点的期望前进方向
float64 vt_dir_x
float64 vt_dir_y
float64 vt_dir_z