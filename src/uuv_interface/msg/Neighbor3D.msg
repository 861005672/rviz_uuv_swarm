string uuv_name
float64 distance

# 真实的当前状态
uuv_interface/State3D state
# 2. 被动声呐感知信息（供制导层 Flocking/APF 使用）
uuv_interface/State3D passive_state
float64 passive_distance             # 测距：带时延的相对距离
float64 passive_direction_azimuth    # 测向：带时延的相对方位角（偏航）
float64 passive_direction_elevation  # 测向：带时延的相对俯仰角
float64 passive_delay_time           # 延迟时间
float64 passive_distance_variance    # 测距方差
float64 passive_direction_variance   # 测向方差
# 主动声纳通信交互的邻居状态
uuv_interface/State3D active_delayed_state
float32 active_delay_time