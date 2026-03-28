#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from uuv_interface.msg import State3D, Neighbor3D, Neighborhood3D, CollisionInfo, CollisionInfoArray, SwarmState, UUVMetrics
from visualization_msgs.msg import MarkerArray, Marker
import xml.etree.ElementTree as ET
import numpy as np
import tf.transformations as tf_trans
from uuv_interface.srv import SetTargetPoint3D
import random

def check_sat_collision(pos1, R1, size1, pos2, R2, size2):
    """
    使用分离轴定理 (SAT) 严格判断两个 3D 旋转包围盒是否发生交叉
    :param pos1, pos2: 两个物体的中心坐标 (np.array)
    :param R1, R2: 两个物体的 3x3 旋转矩阵 (np.array)
    :param size1, size2: 两个物体的 [L, W, H] 尺寸 (np.array)
    """
    T = pos2 - pos1
    a = size1 / 2.0
    b = size2 / 2.0
    
    # 提取两个盒子的各 3 个局部坐标轴
    A = [R1[:, 0], R1[:, 1], R1[:, 2]]
    B = [R2[:, 0], R2[:, 1], R2[:, 2]]
    
    # 构建需要测试的 15 根分离轴 (3个A轴 + 3个B轴 + 9个相互的叉乘轴)
    axes = A + B
    for i in range(3):
        for j in range(3):
            cross_vec = np.cross(A[i], B[j])
            mag = np.linalg.norm(cross_vec)
            if mag > 1e-5:
                axes.append(cross_vec / mag)
                
    # 逐一在 15 根轴上进行投影测试
    for axis in axes:
        # 中心距在当前轴的投影
        t_proj = abs(np.dot(T, axis))
        # 两个盒子在当前轴的投影半径之和
        rA = sum([a[i] * abs(np.dot(A[i], axis)) for i in range(3)])
        rB = sum([b[i] * abs(np.dot(B[i], axis)) for i in range(3)])
        
        # 只要在任意一根轴上能够分离，就绝对没有碰撞！
        if t_proj > rA + rB:
            return False 
            
    # 如果 15 根轴上都有重叠，说明必然发生物理碰撞
    return True

class UUVManager:
    def __init__(self):
        # 邻居判定半径（默认 30.0 米），可以通过 launch 参数动态调整
        self.nb_sense_radius = rospy.get_param('~nb_sense_radius', 30.0)
        self.topo_k_nearest = rospy.get_param('~topo_k_nearest', 6)

        self.passive_sonar_delay = rospy.get_param('~passive_sonar_delay', 0.0) 
        self.active_sonar_delay = rospy.get_param('~active_sonar_delay', 0.0)   
        self.noise_dist_std = rospy.get_param('~noise_dist_std', 0.0)           
        self.noise_ang_std = rospy.get_param('~noise_ang_std', 0.0)            
        self.miss_prob = rospy.get_param('~miss_prob', 0.0)                    
        self.false_prob = rospy.get_param('~false_prob', 0.0)                  
        self.state_history = {} # 状态历史队列 {uuv_id: [(timestamp, State3D), ...]}

        self.default_uuv_size = np.array([2.0, 0.3, 0.3])
        self.default_uuv_max_radius = np.linalg.norm(self.default_uuv_size) / 2.0
        self.uuv_params = {} # 格式: {uuv_id: {'size': np.array, 'max_radius': float, 'parsed': bool}}
        
        self.uuv_targets = {}

        # 计算与发布频率：10Hz 足够满足底层的平滑度要求
        self.rate = rospy.Rate(10)
        
        self.uuv_states = {}
        self.state_subs = {}
        self.nb_pubs = {}
        self.metrics_pubs = {}
        self.obstacles = []
        self.uuv_metrics_data = {}
        rospy.Subscriber('/env_markers', MarkerArray, self.obstacle_callback)
        self.pub_collisions = rospy.Publisher('/uuv_manager/collisions', CollisionInfoArray, queue_size=5)
        self.pub_topology = rospy.Publisher('/uuv_manager/swarm_topology', MarkerArray, queue_size=1)
        self.pub_swarm_state = rospy.Publisher('/uuv_manager/swarm_state', SwarmState, queue_size=1)

        # 定时器：每 1 秒扫描一次系统中是否有新加载的 UUV
        self.discover_timer = rospy.Timer(rospy.Duration(1.0), self.discover_uuvs)
        self.active_collisions = set()
        rospy.loginfo(f"[Neighborhood] Calculator Started! Sensing Radius: {self.nb_sense_radius}m")

    def get_collision(self, uuv_id):
        # ================= 原生 XML 暴力解析真实 URDF 包围盒 =================
        try:
            param_key = f'/{uuv_id}/robot_description'
            
            # 1. 安全获取参数字符串
            if not rospy.has_param(param_key):
                if rospy.has_param('/robot_description'):
                    param_key = '/robot_description'
                else:
                    return # 还没加载出来，下一帧再试
                    
            urdf_str = rospy.get_param(param_key)
            
            # 2. 使用原生 ElementTree 解析 XML 树，完全无视未知标签！
            root = ET.fromstring(urdf_str)
            
            # 3. 精准寻找 base_link
            expected_link_name = f"{uuv_id}/base_link"
            base_link = None
            
            for link in root.findall('link'):
                name = link.get('name')
                if name == expected_link_name or name == 'base_link':
                    base_link = link
                    break
                    
            if base_link is None:
                raise ValueError(f"Could not find link named {expected_link_name} or base_link")

            # 4. 提取 collision 和 geometry
            collision = base_link.find('collision')
            if collision is None:
                raise ValueError("No <collision> tag found in base_link")
                
            geom = collision.find('geometry')
            if geom is None:
                raise ValueError("No <geometry> tag found in collision")

            # 5. 提取真实尺寸
            size = self.default_uuv_size
            box = geom.find('box')
            cylinder = geom.find('cylinder')
            
            if box is not None:
                size_str = box.get('size')
                size = np.array([float(x) for x in size_str.split()])
            elif cylinder is not None:
                length = float(cylinder.get('length'))
                radius = float(cylinder.get('radius'))
                # 映射到我们的 [L, W, H] 物理检测数组中
                size = np.array([length, radius * 2.0, radius * 2.0])
            else:
                raise ValueError("Geometry is neither <box> nor <cylinder>")
            
            max_radius = np.linalg.norm(size) / 2.0
            
            # 6. 成功写入字典并打上标记
            self.uuv_params[uuv_id]['size'] = size
            self.uuv_params[uuv_id]['max_radius'] = max_radius
            self.uuv_params[uuv_id]['parsed'] = True
            
            rospy.loginfo(f"\033[92m[UUVManager] Native XML Parsed for {uuv_id}! Real OBB Size -> {size}\033[0m")
            
        except Exception as e:
            # 打印出真实的异常原因，绝不掩盖错误！
            rospy.logwarn_throttle(2.0, f"\033[93m[UUVManager] Parsing pending for {uuv_id}. Reason: {e}\033[0m")

    def obstacle_callback(self, msg):
        """解析环境节点发来的 Marker，缓存为 NED 坐标系下的纯数学几何体"""
        parsed_obstacles = []
        T_enu_to_ned = np.array([
            [0, 1, 0],
            [1, 0, 0],
            [0, 0, -1]
        ])
        for marker in msg.markers:
            px, py, pz = marker.pose.position.x, marker.pose.position.y, marker.pose.position.z
            
            q = [marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w]
            if sum([abs(x) for x in q]) < 1e-5:
                q = [0, 0, 0, 1]
                
            # 2. 将四元数转为 3x3 旋转矩阵
            R_local = tf_trans.quaternion_matrix(q)[:3, :3]
            
            if marker.header.frame_id == 'map':
                pos_ned = (py, px, -pz)
                # 将 ENU 坐标系下的三个局部旋转轴映射到 NED 全局坐标系下
                R_ned = np.dot(T_enu_to_ned, R_local)
            else:
                pos_ned = (px, py, pz)
                R_ned = R_local
                
            parsed_obstacles.append({
                'id': f"Obstacle_{marker.id}",
                'type': marker.type,
                'pos': pos_ned,
                'R': R_ned,   # <==== 【核心新增】存入障碍物的真实旋转矩阵
                'scale': (marker.scale.x, marker.scale.y, marker.scale.z)
            })
        self.obstacles = parsed_obstacles

    def discover_uuvs(self, event):
        """动态发现新的 UUV，并自动为它建立专属的订阅器与发布器"""
        topics = rospy.get_published_topics()
        for topic_name, topic_type in topics:
            if '/state' in topic_name and 'uuv_interface/State3D' in topic_type:
                # 提取扁平化的 UUV 名称，例如从 "/uuv_15/state" 提取出 "uuv_15"
                parts = topic_name.split('/')
                if len(parts) >= 3 and parts[1].startswith('uuv_'):
                    uuv_id = parts[1]
                    
                    if uuv_id not in self.state_subs:
                        # 【核心架构】为它建立专属的邻居话题发布器
                        self.nb_pubs[uuv_id] = rospy.Publisher(
                            f'/{uuv_id}/neighborhood', Neighborhood3D, queue_size=1)
                        self.metrics_pubs[uuv_id] = rospy.Publisher(
                            f'/{uuv_id}/metrics', UUVMetrics, queue_size=1)
                        self.uuv_params[uuv_id] = {
                            'size': self.default_uuv_size,
                            'max_radius': self.default_uuv_max_radius,
                            'parsed': False
                        }
                        self.uuv_metrics_data[uuv_id] = {
                            'last_pos': None,
                            'path_length': 0.0,
                            'projected_length': 0.0,
                            'sum_r_sq': 0.0,
                            'count': 0
                        }
                        # 订阅它的状态
                        self.state_subs[uuv_id] = rospy.Subscriber(
                            topic_name, State3D, self.state_callback, callback_args=uuv_id)
                        rospy.loginfo(f"[Neighborhood] Automatically hooked {uuv_id}. Dedicated topic created.")
                    if not self.uuv_params[uuv_id]['parsed']:
                        self.get_collision(uuv_id)

    def state_callback(self, msg, uuv_id):
        # 更新最新状态快照
        self.uuv_states[uuv_id] = msg

    def get_relative_info(self, self_state, dx, dy, dz):
        """
        计算目标在自身机体坐标系 (Body Frame) 下的相对位置与视线方位
        :param self_state: 自身的全局状态
        :param dx, dy, dz: 目标全局坐标减去自身全局坐标 (NED系下)
        """
        # 1. 绕 Z 轴逆旋转 (Yaw)
        x1 = dx * math.cos(self_state.yaw) + dy * math.sin(self_state.yaw)
        y1 = -dx * math.sin(self_state.yaw) + dy * math.cos(self_state.yaw)
        z1 = dz
        
        # 2. 绕 Y 轴逆旋转 (Pitch)
        x2 = x1 * math.cos(self_state.pitch) - z1 * math.sin(self_state.pitch)
        y2 = y1
        z2 = x1 * math.sin(self_state.pitch) + z1 * math.cos(self_state.pitch)
        
        # 3. 绕 X 轴逆旋转 (Roll)
        rel_x = x2
        rel_y = y2 * math.cos(self_state.roll) + z2 * math.sin(self_state.roll)
        rel_z = -y2 * math.sin(self_state.roll) + z2 * math.cos(self_state.roll)
        
        # 4. 计算视线方位角 (Line-Of-Sight)
        # Azimuth: 左右方位角 (右为正)
        rel_yaw = math.atan2(rel_y, rel_x)
        # Elevation: 上下方位角 (由于NED中Z朝下，因此用 -rel_z 代表向上为正)
        rel_pitch = math.atan2(-rel_z, math.hypot(rel_x, rel_y))
        
        return rel_x, rel_y, rel_z, rel_yaw, rel_pitch
    # ====================================================================


    def check_los(self, p1, p2):
        """
        【核心新增】：视线遮挡 (Line-Of-Sight, LOS) 检测
        检测两点 p1 和 p2 之间的直线线段是否被环境中的任何障碍物遮挡
        :return: True (视线无遮挡), False (被遮挡)
        """
        d = p2 - p1
        d_len = np.linalg.norm(d)
        if d_len < 1e-5:
            return True # 距离太近，认为无遮挡

        for obs in self.obstacles:
            obs_pos = np.array(obs['pos'])
            sx, sy, sz = obs['scale']
            # 障碍物的最大外接球半径
            max_radius = math.sqrt((sx/2.0)**2 + (sy/2.0)**2 + (sz/2.0)**2)

            # 1. 宽相检测 (Broad-Phase)：点到线段的距离测试
            v_to_obs = obs_pos - p1
            t_proj = np.dot(v_to_obs, d) / (d_len * d_len)
            t_proj = max(0.0, min(1.0, t_proj)) # 限制在线段 [0, 1] 内
            closest_pt = p1 + t_proj * d
            dist_to_line = np.linalg.norm(obs_pos - closest_pt)
            
            # 如果线段离障碍物中心点的最短距离都大于外接球半径，绝对不可能遮挡，直接跳过！
            if dist_to_line > max_radius:
                continue

            # 2. 窄相检测 (Narrow-Phase)：将线段转换到障碍物的局部坐标系下
            R_mat = obs['R']
            local_p1 = np.dot(R_mat.T, p1 - obs_pos)
            local_p2 = np.dot(R_mat.T, p2 - obs_pos)
            local_d = local_p2 - local_p1

            if obs['type'] == Marker.SPHERE:
                # 射线 vs 球体交点测试
                r = sx / 2.0
                a = np.dot(local_d, local_d)
                b = 2.0 * np.dot(local_p1, local_d)
                c = np.dot(local_p1, local_p1) - r*r
                delta = b*b - 4*a*c
                if delta >= 0:
                    t1 = (-b - math.sqrt(delta)) / (2*a)
                    t2 = (-b + math.sqrt(delta)) / (2*a)
                    # 如果有任何一个交点在线段 t in [0, 1] 内部，说明被遮挡
                    if (0 <= t1 <= 1) or (0 <= t2 <= 1) or (t1 < 0 and t2 > 1):
                        return False 

            elif obs['type'] == Marker.CUBE:
                # 射线 vs 倾斜包围盒 (OBB/AABB) - Slab Method
                t_min, t_max = 0.0, 1.0
                h = np.array([sx/2.0, sy/2.0, sz/2.0])
                intersect = True
                for i in range(3):
                    if abs(local_d[i]) < 1e-6:
                        # 射线平行于该轴，判断起点是否在盒内
                        if local_p1[i] < -h[i] or local_p1[i] > h[i]:
                            intersect = False; break
                    else:
                        t1 = (-h[i] - local_p1[i]) / local_d[i]
                        t2 = (h[i] - local_p1[i]) / local_d[i]
                        if t1 > t2: t1, t2 = t2, t1
                        t_min = max(t_min, t1)
                        t_max = min(t_max, t2)
                        if t_min > t_max:
                            intersect = False; break
                if intersect and t_max >= 0 and t_min <= 1:
                    return False 

            elif obs['type'] == Marker.CYLINDER:
                # 射线 vs 倾斜圆柱体
                r = sx / 2.0
                half_h = sz / 2.0
                # 圆柱无限长方程投影：(x+dx*t)^2 + (y+dy*t)^2 = r^2
                a = local_d[0]**2 + local_d[1]**2
                b = 2.0 * (local_p1[0]*local_d[0] + local_p1[1]*local_d[1])
                c = local_p1[0]**2 + local_p1[1]**2 - r**2
                
                if a < 1e-6: # 射线平行于Z轴
                    if c <= 0: # 穿过圆柱内部
                        t_near = (-half_h - local_p1[2]) / local_d[2] if local_d[2] != 0 else -1
                        t_far = (half_h - local_p1[2]) / local_d[2] if local_d[2] != 0 else -1
                        if local_d[2] == 0:
                            if -half_h <= local_p1[2] <= half_h: return False
                        else:
                            if t_near > t_far: t_near, t_far = t_far, t_near
                            if max(0.0, t_near) <= min(1.0, t_far): return False
                else:
                    delta = b*b - 4*a*c
                    if delta >= 0:
                        t1 = (-b - math.sqrt(delta)) / (2*a)
                        t2 = (-b + math.sqrt(delta)) / (2*a)
                        if t1 > t2: t1, t2 = t2, t1
                        # 计算位于线段内的有效交点段
                        t_in, t_out = max(0.0, t1), min(1.0, t2)
                        if t_in <= t_out:
                            z_in = local_p1[2] + t_in * local_d[2]
                            z_out = local_p1[2] + t_out * local_d[2]
                            z_min, z_max = min(z_in, z_out), max(z_in, z_out)
                            # 如果该有效线段的高度落在圆柱上下底面范围内，则遮挡
                            if z_min <= half_h and z_max >= -half_h:
                                return False

        return True # 所有障碍物都未遮挡视线


    def update(self):
        # 获取当前所有 UUV 的状态快照
        states = dict(self.uuv_states)
        uuvs = list(states.keys())
        n = len(uuvs)

        current_time = rospy.Time.now().to_sec()
        self._update_state_history(current_time, states)
        t_passive = current_time - self.passive_sonar_delay
        t_active = current_time - self.active_sonar_delay

        for uuv_id, state in states.items():
            if uuv_id in self.uuv_metrics_data:
                data = self.uuv_metrics_data[uuv_id]
                current_pos = np.array([state.x, state.y, state.z])
                
                # 1. 计算累计绕行距离 (Path Length)
                if data['last_pos'] is not None:
                    # 1. 计算当前帧的真实位移向量
                    delta_pos = current_pos - data['last_pos']
                    data['path_length'] += np.linalg.norm(delta_pos)

                    # 2. 获取 vTgt 方向向量并归一化
                    vt_dir = np.array([state.vt_dir_x, state.vt_dir_y, state.vt_dir_z])
                    norm_vt = np.linalg.norm(vt_dir)
                    if norm_vt > 1e-4:
                        vt_dir /= norm_vt
                        # 【核心计算】：位移在任务方向上的投影
                        # 只有顺着 vTgt 方向走的部分才算有效推进
                        proj_dist = np.dot(delta_pos, vt_dir)
                        data['projected_length'] += proj_dist

                data['last_pos'] = current_pos
                
                # 2. 计算偏航角速度平方和 (用于最后开根号求 RMS)
                data['sum_r_sq'] += state.r ** 2
                data['count'] += 1
        
        # 初始化每个 UUV 的专属空邻居列表和最近障碍物距离、最近邻居距离统计量
        neighborhoods = {uuv: Neighborhood3D() for uuv in uuvs}
        min_nb_dists = {uuv: self.nb_sense_radius for uuv in uuvs}
        min_obs_dists = {uuv: 200.0 for uuv in uuvs}

        collision_msg_array = CollisionInfoArray()
        collision_msg_array.header.stamp = rospy.Time.now()
        current_collisions = set()

        # 0. 提前计算所有 UUV 的位置向量和 3x3 旋转矩阵，避免在循环中重复计算
        uuv_data = {}
        for uuv_id, state in states.items():
            cr, sr = math.cos(state.roll), math.sin(state.roll)
            cp, sp = math.cos(state.pitch), math.sin(state.pitch)
            cy, sy = math.cos(state.yaw), math.sin(state.yaw)
            
            R_mat = np.array([
                [cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy],
                [cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy],
                [-sp,   sr*cp,            cr*cp]
            ])
            uuv_data[uuv_id] = {
                'pos': np.array([state.x, state.y, state.z]),
                'R': R_mat
            }
        
        if n > 1:
            # O(N^2 / 2) 算法：双重遍历，计算距离并双向添加（避免重复计算 A到B 和 B到A）
            for i in range(n):
                uuv_i = uuvs[i]
                if uuv_i not in self.uuv_params: continue
                state_i = states[uuv_i]
                
                for j in range(i + 1, n):
                    uuv_j = uuvs[j]
                    if uuv_j not in self.uuv_params: continue
                    state_j = states[uuv_j]

                    # 计算 3D 欧几里得距离
                    dx = state_j.x - state_i.x
                    dy = state_j.y - state_i.y
                    dz = state_j.z - state_i.z
                    dist = math.sqrt(dx*dx + dy*dy + dz*dz)

                    if dist < min_nb_dists[uuv_i]: 
                        min_nb_dists[uuv_i] = dist
                        if min_nb_dists[uuv_i] > self.nb_sense_radius:
                            min_nb_dists[uuv_i] = self.nb_sense_radius
                    if dist < min_nb_dists[uuv_j]: 
                        min_nb_dists[uuv_j] = dist
                        if min_nb_dists[uuv_i] > self.nb_sense_radius:
                            min_nb_dists[uuv_i] = self.nb_sense_radius

                    # 若在感知半径内，则互相加为邻居
                    if dist <= self.nb_sense_radius and self.check_los(uuv_data[uuv_i]['pos'], uuv_data[uuv_j]['pos']):
                        # 生成 J 发给 I 的消息并添加（已包含漏检判定）
                        msg_j = self._build_neighbor_msg(uuv_i, uuv_j, dist, state_i, state_j, t_passive, t_active)
                        if msg_j: neighborhoods[uuv_i].neighbors.append(msg_j)
                        # 生成 I 发给 J 的消息并添加
                        msg_i = self._build_neighbor_msg(uuv_j, uuv_i, dist, state_j, state_i, t_passive, t_active)
                        if msg_i: neighborhoods[uuv_j].neighbors.append(msg_i)

                    # UUV之间碰撞检测  只有当距离小于两个最大外接球半径之和时，才进行详细 OBB 测试
                    if dist < self.uuv_params[uuv_i]['max_radius'] + self.uuv_params[uuv_j]['max_radius']:
                        pos1, R1 = uuv_data[uuv_i]['pos'], uuv_data[uuv_i]['R']
                        pos2, R2 = uuv_data[uuv_j]['pos'], uuv_data[uuv_j]['R']
                        
                        # Narrow-Phase: 调用引擎级 SAT 严格判定
                        if check_sat_collision(pos1, R1, self.uuv_params[uuv_i]['size'], pos2, R2, self.uuv_params[uuv_j]['size']):
                            pair_key = tuple(sorted([uuv_i, uuv_j]))
                            current_collisions.add(pair_key)
                            
                            if pair_key not in self.active_collisions:
                                rospy.logwarn(f"\033[93m[CRASH] Swarm OBB Collision! {uuv_i} hit {uuv_j}!\033[0m")
                            
                            info = CollisionInfo(); info.uuv_id = uuv_i; info.target_id = uuv_j; info.distance = dist
                            collision_msg_array.collisions.append(info)


        # 3. 动态/静态障碍物碰撞判定与精准安全距离计算
        for obs in self.obstacles:
            obs_id = obs['id']
            obs_pos = np.array(obs['pos'])
            obs_R = obs['R'] 
            sx, sy, sz = obs['scale']
            
            # 预计算障碍物的最大外接球半径
            obs_max_radius = math.sqrt((sx/2.0)**2 + (sy/2.0)**2 + (sz/2.0)**2)
            
            for uuv_id, data in uuv_data.items():
                u_pos = data['pos']
                u_R = data['R']
                
                dist_centers = np.linalg.norm(u_pos - obs_pos)
                rough_surface_dist = dist_centers - self.uuv_params[uuv_id]['max_radius'] - obs_max_radius
                
                # 初始化一个默认无穷大的精准距离
                exact_surface_dist = 9999.0
                
                # =========================================================
                # 逻辑通道一：精细距离统计
                # =========================================================
                # 如果粗略大球距离 < 50 米，说明真正的表面一定在危险区附近，开启精细计算！
                if rough_surface_dist < 50.0:
                    uuv_L_half = self.uuv_params[uuv_id]['size'][0] / 2.0
                    uuv_r = self.uuv_params[uuv_id]['size'][1] / 2.0
                    min_exact_dist = 9999.0
                    
                    for lx in np.linspace(-uuv_L_half, uuv_L_half, 5):
                        p_global = u_pos + np.dot(u_R, np.array([lx, 0.0, 0.0]))
                        p_local = np.dot(obs_R.T, p_global - obs_pos)
                        
                        if obs['type'] == Marker.SPHERE:
                            d = np.linalg.norm(p_local) - (sx / 2.0)
                        elif obs['type'] == Marker.CUBE:
                            obs_half = np.array([sx/2.0, sy/2.0, sz/2.0])
                            closest_p = np.clip(p_local, -obs_half, obs_half)
                            d = np.linalg.norm(p_local - closest_p)
                        elif obs['type'] == Marker.CYLINDER:
                            r_obs = sx / 2.0
                            h_half = sz / 2.0
                            d_xy = max(0.0, math.hypot(p_local[0], p_local[1]) - r_obs)
                            d_z = max(0.0, abs(p_local[2]) - h_half)
                            d = math.hypot(d_xy, d_z)
                        else:
                            d = dist_centers - obs_max_radius
                            
                        if d < min_exact_dist:
                            min_exact_dist = d
                            
                    # 【核心修改2】：只有精确计算的结果，才允许去更新打擂台的距离！
                    # 绝不把 rough_surface_dist 赋值给它，彻底消灭公式切换导致的跳变！
                    exact_surface_dist = min_exact_dist - uuv_r
                    if exact_surface_dist < min_obs_dists[uuv_id]:
                        min_obs_dists[uuv_id] = exact_surface_dist

                # =========================================================
                # 逻辑通道二：极速碰撞报警
                # =========================================================
                # 只要大包围球距离 > 0，说明连外接球都没蹭到，直接放行，极大节省CPU！
                if rough_surface_dist > 0.0:
                    continue
                    
                is_hit = False
                
                if obs['type'] == Marker.SPHERE:  
                    is_hit = (exact_surface_dist < 0)
                elif obs['type'] == Marker.CYLINDER:
                    is_hit = (exact_surface_dist < 0)
                elif obs['type'] == Marker.CUBE: 
                    # 只有真正贴身了，才调用 SAT 进行终极物理相交定性
                    obs_size = np.array([sx, sy, sz])
                    is_hit = check_sat_collision(u_pos, u_R, self.uuv_params[uuv_id]['size'], obs_pos, obs_R, obs_size)
                                
                if is_hit:
                    pair_key = (uuv_id, obs_id)
                    current_collisions.add(pair_key)
                    if pair_key not in self.active_collisions:
                        rospy.logwarn(f"\033[91m[WARN] Environment Hit! {uuv_id} crashed into {obs_id}!\033[0m")
                    info = CollisionInfo()
                    info.uuv_id = uuv_id
                    info.target_id = obs_id
                    info.distance = float(dist_centers)
                    collision_msg_array.collisions.append(info)

        # 碰撞接触检测
        resolved_collisions = self.active_collisions - current_collisions
        for pair in resolved_collisions:
            rospy.loginfo(f"\033[92m[SAFE] Collision Resolved! {pair[0]} separated from {pair[1]}.\033[0m")
            
        # 更新历史状态，供下一帧对比使用
        self.active_collisions = current_collisions

        # 遍历发布：每个 UUV 只收到属于自己的那一份数据
        for uuv in uuvs:
            if uuv in self.nb_pubs:
                self.nb_pubs[uuv].publish(neighborhoods[uuv])

        if len(collision_msg_array.collisions) > 0:
            self.pub_collisions.publish(collision_msg_array)

        # 发布集群状态
        self.publish_swarm_state(uuvs, neighborhoods)
        # 发布集群拓扑可视化网格
        self.publish_topology_marker(states, neighborhoods)
        # 发布所有UUV的统计指标信息
        time_now = rospy.Time.now()
        self.publish_swarm_metrics(time_now, uuvs, min_nb_dists, min_obs_dists)


    def publish_swarm_metrics(self, time, uuvs, min_nb_dists, min_obs_dists):
        """
        发布集群各个UUV的指标统计状态（最近邻居距离和最近障碍物距离）
        """
        for uuv in uuvs:
            if uuv in self.metrics_pubs:
                msg = UUVMetrics()
                msg.header.stamp = time
                msg.uuv_name = uuv
                msg.min_nb_dist = min_nb_dists[uuv]
                msg.min_obs_dist = min_obs_dists[uuv]
                data = self.uuv_metrics_data[uuv]
                msg.cumulative_path_length = data['path_length']
                msg.cumulative_projected_dist = data['projected_length']
                # 计算实时绕行度：总航程 / 有效推进航程
                if data['projected_length'] > 10.0:
                    msg.detour_ratio = data['path_length'] / data['projected_length']
                else:
                    msg.detour_ratio = 1.0
                # 均方根 (RMS) = 内部平方和的平均值的平方根
                if data['count'] > 0:
                    msg.rms_yaw_rate = math.sqrt(data['sum_r_sq'] / data['count'])
                else:
                    msg.rms_yaw_rate = 0.0
                self.metrics_pubs[uuv].publish(msg)

    def publish_swarm_state(self, uuvs, neighborhoods):
        """
        计算并发布集群拓扑状态（加权代数连通度和连通分量数）
        """
        n = len(uuvs)
        if n == 0:
            return
            
        L_mat = np.zeros((n, n))
        uuv_indices = {uuv_name: idx for idx, uuv_name in enumerate(uuvs)}
        
        # 构建加权无向图的拉普拉斯矩阵 (Weighted Laplacian Matrix)
        for uuv_i, nb_data in neighborhoods.items():
            idx_i = uuv_indices[uuv_i]
            weighted_degree = 0.0
            
            for nb in nb_data.neighbors:
                if nb.uuv_name not in uuv_indices:
                    continue
                idx_j = uuv_indices[nb.uuv_name]
                
                # 【核心修改】：计算基于距离的连续权重
                # 距离越近，权重越接近 1.0；距离到达感知边缘 (nb_sense_radius)，权重平滑降为 0.0
                weight = 1.0 - (nb.distance / self.nb_sense_radius)
                
                # 限幅保护，确保权重严格在 [0, 1] 之间
                weight = max(0.0, min(1.0, weight))
                
                # 非对角线元素为负的权重，对角线元素为权重之和
                L_mat[idx_i, idx_j] = -weight
                weighted_degree += weight
                
            L_mat[idx_i, idx_i] = weighted_degree
            
        try:
            # 使用 np.linalg.eigvalsh 求对称矩阵的特征值（结果会自动从小到大排序）
            evals = np.linalg.eigvalsh(L_mat)
            
            # 特征值接近 0 的个数即为连通分量数 (加权图中判定阈值需稍微放宽防浮点误差)
            num_components = int(np.sum(evals < 1e-4))
            # 代数连通度 (Algebraic Connectivity) 为第二小特征值
            alg_conn = float(evals[1]) if n > 1 else 0.0
            
        except Exception as e:
            num_components = n
            alg_conn = 0.0
            
        # 打包并发布自定义的 SwarmState 消息
        state_msg = SwarmState()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.connectivity = alg_conn
        state_msg.num_connected = num_components
        self.pub_swarm_state.publish(state_msg)


    def publish_topology_marker(self, states, neighborhoods):
        """
        构建并发布集群拓扑 (Swarm Topology) 网络 - KNN 优化版
        """
        if self.pub_topology.get_num_connections() == 0:
            return
            
        topo_marker_array = MarkerArray()
        
        lines = Marker()
        lines.header.frame_id = "map"  
        lines.header.stamp = rospy.Time.now()
        lines.ns = "swarm_topology"
        lines.id = 0
        lines.type = Marker.LINE_LIST
        lines.action = Marker.ADD
        lines.pose.orientation.w = 1.0
        lines.scale.x = 0.5  # 线宽
        
        from geometry_msgs.msg import Point
        from std_msgs.msg import ColorRGBA
        
        drawn_edges = set() # 用于无向图去重
        
        for uuv_i, nb_data in neighborhoods.items():
            pos_i = np.array([states[uuv_i].x, states[uuv_i].y, states[uuv_i].z])
            p1 = Point(x=pos_i[1], y=pos_i[0], z=-pos_i[2]) # NED转ENU
            
            # KNN逻辑：根据距离从小到大排序，并只取前 topo_k_nearest 个进行连线
            sorted_neighbors = sorted(nb_data.neighbors, key=lambda n: n.distance)
            knn_neighbors = sorted_neighbors[:self.topo_k_nearest]
            
            for n_j in knn_neighbors:
                uuv_j = n_j.uuv_name
                if uuv_j not in states:
                    continue
                
                # 连线去重：如果 uuv_1 和 uuv_2 连过线了，跳过
                edge_key = tuple(sorted([uuv_i, uuv_j]))
                if edge_key in drawn_edges:
                    continue
                drawn_edges.add(edge_key)
                
                # 取出邻居位置
                pos_j = np.array([states[uuv_j].x, states[uuv_j].y, states[uuv_j].z])
                p2 = Point(x=pos_j[1], y=pos_j[0], z=-pos_j[2])
                
                lines.points.append(p1)
                lines.points.append(p2)
                
                # 直接复用邻居数据里已经算好的 distance
                alpha = 1.0 - 0.5 * (n_j.distance / self.nb_sense_radius)
                alpha = max(0.1, alpha ** 1.5) 
                
                color = ColorRGBA(r=0.49, g=1.0, b=0.66, a=alpha)
                lines.colors.append(color)
                lines.colors.append(color)
        lines.frame_locked = True

        topo_marker_array.markers.append(lines)
        self.pub_topology.publish(topo_marker_array)


    def _update_state_history(self, current_time, states):
        """
        模块1：基于真实物理时间的滑动窗口历史队列（自适应任意运行频率）
        """
        # 1. 动态计算我们需要保留的最大历史时间跨度
        # 取主动时延和被动时延里的最大值，并额外加上 2.0 秒的“安全缓冲期”
        max_required_delay = max(self.passive_sonar_delay, self.active_sonar_delay)
        keep_time_window = max_required_delay + 2.0 

        for uuv_id, state in states.items():
            if uuv_id not in self.state_history:
                self.state_history[uuv_id] = []
                
            # 把当前最新状态压入队列末尾
            self.state_history[uuv_id].append((current_time, state))
            
            # 2. 【核心修改】：按绝对时间差，从头部剔除过于古老的数据
            # 如果队列头部（最老）的数据的时间戳，距离当前时间已经超过了我们需要的历史窗口，就把它扔掉
            while len(self.state_history[uuv_id]) > 0:
                oldest_time = self.state_history[uuv_id][0][0]
                if (current_time - oldest_time) > keep_time_window:
                    self.state_history[uuv_id].pop(0)
                else:
                    # 一旦头部的满足时间要求了，后面的肯定更满足，直接跳出 while 循环
                    break

    def _get_past_state(self, uuv_name, target_t):
        """模块2：根据时间戳回溯获取过去的状态"""
        if uuv_name not in self.state_history or not self.state_history[uuv_name]:
            return self.uuv_states[uuv_name]
        hist = self.state_history[uuv_name]
        closest_state = hist[0][1]
        min_diff = abs(hist[0][0] - target_t)
        for t, s in hist:
            diff = abs(t - target_t)
            if diff < min_diff:
                min_diff = diff
                closest_state = s
        return closest_state
    
    
    def _build_neighbor_msg(self, self_id, neighbor_id, real_dist, real_self_state, real_nb_state, t_passive, t_active):
        """模块3：组装邻居消息（包含漏检判定、时延回溯、加噪与突变错检）"""
        # 1. 漏检判定：一旦漏检，直接返回空，这帧彻底丢失该邻居
        if random.random() < self.miss_prob: 
            return None
            
        msg = Neighbor3D()
        msg.uuv_name = neighbor_id
        msg.distance = real_dist
        msg.state = real_nb_state
        
        # 兼容旧字段 (上帝视角的真实相对位置)
        rx, ry, rz, ryaw, rpitch = self.get_relative_info(real_self_state, 
                                                          real_nb_state.x - real_self_state.x, 
                                                          real_nb_state.y - real_self_state.y, 
                                                          real_nb_state.z - real_self_state.z)
        if hasattr(msg, 'rel_x'):
            msg.rel_x = rx; msg.rel_y = ry; msg.rel_z = rz
            msg.rel_yaw = ryaw; msg.rel_pitch = rpitch

# 2. 被动声纳专属：回溯真实的历史状态
        p_self_state = self._get_past_state(self_id, t_passive)
        p_nb_state = self._get_past_state(neighbor_id, t_passive)
        p_dx = p_nb_state.x - p_self_state.x
        p_dy = p_nb_state.y - p_self_state.y
        p_dz = p_nb_state.z - p_self_state.z
        p_dist = math.sqrt(p_dx**2 + p_dy**2 + p_dz**2)
        
        # 全局绝对角度 (用于还原加噪后的世界坐标)
        global_az = math.atan2(p_dy, p_dx)
        global_el = math.atan2(p_dz, math.hypot(p_dx, p_dy))
        
        # 机体相对角度 (用于声纳真实的测向感知)
        _, _, _, p_rel_yaw, p_rel_pitch = self.get_relative_info(
            p_self_state, p_dx, p_dy, p_dz)

        # 噪声叠加与突变处理
        if random.random() < self.false_prob:
            noisy_dist = random.uniform(2.0, self.nb_sense_radius) 
            noisy_global_az = random.uniform(-math.pi, math.pi) 
            noisy_global_el = random.uniform(-math.pi/4, math.pi/4)
            noisy_rel_yaw = random.uniform(-math.pi, math.pi)
            noisy_rel_pitch = random.uniform(-math.pi/4, math.pi/4)
        else:
            noise_d = random.gauss(0, self.noise_dist_std)
            noise_ang = random.gauss(0, self.noise_ang_std)
            
            noisy_dist = max(0.5, p_dist + noise_d)
            noisy_global_az = global_az + noise_ang
            noisy_global_el = global_el + noise_ang
            noisy_rel_yaw = p_rel_yaw + noise_ang
            noisy_rel_pitch = p_rel_pitch + noise_ang
            
        # 【修复 1】：发布加噪后的距离与相对机体方位角 (Debug脚本的曲线现在会动了)
        msg.passive_distance = noisy_dist
        msg.passive_direction_azimuth = noisy_rel_yaw
        msg.passive_direction_elevation = noisy_rel_pitch

        # 【修复 2】：新建 State3D 对象，并手动继承邻居的历史速度与姿态
        # (避免指针直接引用导致改坏 state_history 队列里的数据)
        msg.passive_state = State3D()
        msg.passive_state.yaw = p_nb_state.yaw
        msg.passive_state.pitch = p_nb_state.pitch
        msg.passive_state.roll = p_nb_state.roll
        msg.passive_state.u = p_nb_state.u
        msg.passive_state.v = p_nb_state.v
        msg.passive_state.w = p_nb_state.w
        msg.passive_state.p = p_nb_state.p
        msg.passive_state.q = p_nb_state.q
        msg.passive_state.r = p_nb_state.r

        # 【修复 3】：用带有噪声的全局角度和距离，覆盖 x, y, z
        msg.passive_state.x = p_self_state.x + noisy_dist * math.cos(noisy_global_el) * math.cos(noisy_global_az)
        msg.passive_state.y = p_self_state.y + noisy_dist * math.cos(noisy_global_el) * math.sin(noisy_global_az)
        msg.passive_state.z = p_self_state.z + noisy_dist * math.sin(noisy_global_el)
        
        # 3. 主动声纳专属：回溯 (不加噪)
        msg.active_delayed_state = self._get_past_state(neighbor_id, t_active)
        msg.active_delay_time = self.active_sonar_delay
        
        return msg