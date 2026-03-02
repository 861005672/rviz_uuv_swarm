#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from uuv_interface.msg import State3D, Neighbor3D, Neighborhood3D, CollisionInfo, CollisionInfoArray
from visualization_msgs.msg import MarkerArray, Marker
import xml.etree.ElementTree as ET
import numpy as np
import tf.transformations as tf_trans

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

class UUVManagerNode:
    def __init__(self):
        rospy.init_node('uuv_manager_node', anonymous=True)
        # 邻居判定半径（默认 30.0 米），可以通过 launch 参数动态调整
        self.nb_sense_radius = rospy.get_param('~nb_sense_radius', 30.0)

        self.default_uuv_size = np.array([2.0, 0.3, 0.3])
        self.default_uuv_max_radius = np.linalg.norm(self.default_uuv_size) / 2.0
        self.uuv_params = {} # 格式: {uuv_id: {'size': np.array, 'max_radius': float, 'parsed': bool}}
        
        # 计算与发布频率：10Hz 足够满足底层的平滑度要求
        self.rate = rospy.Rate(10)
        
        self.uuv_states = {}
        self.state_subs = {}
        self.nb_pubs = {}
        self.obstacles = []
        rospy.Subscriber('/env_markers', MarkerArray, self.obstacle_callback)
        self.pub_collisions = rospy.Publisher('/uuv_manager/collisions', CollisionInfoArray, queue_size=5)

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
                        # 订阅它的状态
                        self.state_subs[uuv_id] = rospy.Subscriber(
                            topic_name, State3D, self.state_callback, callback_args=uuv_id)
                        # 【核心架构】为它建立专属的邻居话题发布器
                        self.nb_pubs[uuv_id] = rospy.Publisher(
                            f'/{uuv_id}/neighborhood', Neighborhood3D, queue_size=1)
                        self.uuv_params[uuv_id] = {
                            'size': self.default_uuv_size,
                            'max_radius': self.default_uuv_max_radius,
                            'parsed': False
                        }
                        rospy.loginfo(f"[Neighborhood] Automatically hooked {uuv_id}. Dedicated topic created.")
                    if not self.uuv_params[uuv_id]['parsed']:
                        self.get_collision(uuv_id)

    def state_callback(self, msg, uuv_id):
        # 更新最新状态快照
        self.uuv_states[uuv_id] = msg

    def run(self):
        # 持续循环计算并发布
        while not rospy.is_shutdown():
            self.calculate_and_publish()
            self.rate.sleep()

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

    def calculate_and_publish(self):
        # 获取当前所有 UUV 的状态快照
        states = dict(self.uuv_states)
        uuvs = list(states.keys())
        n = len(uuvs)
        
        # 初始化每个 UUV 的专属空邻居列表
        neighborhoods = {uuv: Neighborhood3D() for uuv in uuvs}

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
                state_i = states[uuv_i]
                
                for j in range(i + 1, n):
                    uuv_j = uuvs[j]
                    state_j = states[uuv_j]

                    # 计算 3D 欧几里得距离
                    dx = state_j.x - state_i.x
                    dy = state_j.y - state_i.y
                    dz = state_j.z - state_i.z
                    dist = math.sqrt(dx*dx + dy*dy + dz*dz)

                    # 若在感知半径内，则互相加为邻居
                    if dist <= self.nb_sense_radius:
                        # 把 J 加入 I 的邻居列表
                        rx_j, ry_j, rz_j, ryaw_j, rpitch_j = self.get_relative_info(state_i, dx, dy, dz)
                        n_j = Neighbor3D()
                        n_j.uuv_name = uuv_j
                        n_j.distance = dist
                        n_j.state = state_j
                        n_j.rel_x = rx_j; n_j.rel_y = ry_j; n_j.rel_z = rz_j
                        n_j.rel_yaw = ryaw_j; n_j.rel_pitch = rpitch_j
                        neighborhoods[uuv_i].neighbors.append(n_j)

                        # 把 I 加入 J 的邻居列表
                        rx_i, ry_i, rz_i, ryaw_i, rpitch_i = self.get_relative_info(state_j, -dx, -dy, -dz)
                        n_i = Neighbor3D()
                        n_i.uuv_name = uuv_i; n_i.distance = dist; n_i.state = state_i
                        n_i.rel_x = rx_i; n_i.rel_y = ry_i; n_i.rel_z = rz_i
                        n_i.rel_yaw = ryaw_i; n_i.rel_pitch = rpitch_i
                        neighborhoods[uuv_j].neighbors.append(n_i)

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

        # 3. 动态/静态障碍物碰撞判定
        for obs in self.obstacles:
            obs_id = obs['id']
            obs_pos = np.array(obs['pos'])
            obs_R = obs['R'] # 这是你在上一轮加的，障碍物的真实倾斜旋转矩阵
            sx, sy, sz = obs['scale']
            
            # 预计算障碍物的最大外接球半径 (Broad-phase 快速剔除)
            obs_max_radius = math.sqrt((sx/2.0)**2 + (sy/2.0)**2 + (sz/2.0)**2)
            
            for uuv_id, data in uuv_data.items():
                u_pos = data['pos']
                u_R = data['R']
                
                # Broad-Phase: 粗略过滤。如果两个外接球都碰不到，绝对安全，直接跳过！
                dist_centers = np.linalg.norm(u_pos - obs_pos)
                if dist_centers > self.uuv_params[uuv_id]['max_radius'] + obs_max_radius:
                    continue 
                    
                is_hit = False
                dist_hit = dist_centers # 用于记录日志的参考距离
                
                if obs['type'] == Marker.SPHERE:  
                    # 【核心 1：严谨的 OBB vs 球体 算法】
                    r = sx / 2.0  
                    v = obs_pos - u_pos
                    v_local = np.dot(u_R.T, v) # 将球心转换到 UUV 局部坐标系
                    u_half = self.uuv_params[uuv_id]['size'] / 2.0
                    
                    # 寻找 OBB 表面上距离球心最近的精确点
                    closest_local = np.clip(v_local, -u_half, u_half)
                    dist = np.linalg.norm(v_local - closest_local)
                    if dist < r:
                        is_hit = True
                        dist_hit = dist
                        
                elif obs['type'] == Marker.CUBE: 
                    # 【核心 2：严谨的 OBB vs 倾斜 OBB (SAT) 算法】
                    obs_size = np.array([sx, sy, sz])
                    if check_sat_collision(u_pos, u_R, self.uuv_params[uuv_id]['size'], obs_pos, obs_R, obs_size):
                        is_hit = True
                        
                elif obs['type'] == Marker.CYLINDER:
                    # 【核心 3：严谨的 胶囊体(UUV) vs 倾斜圆柱体 算法】
                    a, b = sx / 2.0, sy / 2.0
                    h = sz
                    uuv_r = self.uuv_params[uuv_id]['size'][1] / 2.0
                    L_half = self.uuv_params[uuv_id]['size'][0] / 2.0
                    for lx in np.linspace(-L_half, L_half, 5):
                        # 转换到全局坐标
                        p_global = u_pos + np.dot(u_R, np.array([lx, 0.0, 0.0]))
                        # 转换到圆柱体的倾斜局部坐标系
                        p_cyl = np.dot(obs_R.T, p_global - obs_pos)
                        px, py, pz = p_cyl[0], p_cyl[1], p_cyl[2]
                        
                        # 检测该点是否在圆柱体内 (扩大了 UUV 本身的半径容差)
                        if abs(pz) < (h/2.0 + uuv_r): 
                            if (px**2 / (a + uuv_r)**2) + (py**2 / (b + uuv_r)**2) <= 1.0: 
                                is_hit = True
                                break
                if is_hit:
                    pair_key = (uuv_id, obs_id)
                    current_collisions.add(pair_key)
                    if pair_key not in self.active_collisions:
                        rospy.logwarn(f"\033[91m[WARN] Environment Hit! {uuv_id} crashed into {obs_id}!\033[0m")
                    info = CollisionInfo(); info.uuv_id = uuv_id; info.target_id = obs_id; info.distance = float(dist_hit)
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

if __name__ == '__main__':
    try:
        node = UUVManagerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass