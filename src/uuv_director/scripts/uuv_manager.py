#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from uuv_interface.msg import State3D, Neighbor3D, Neighborhood3D, CollisionInfo, CollisionInfoArray
from visualization_msgs.msg import MarkerArray, Marker

class UUVManagerNode:
    def __init__(self):
        rospy.init_node('uuv_manager_node', anonymous=True)
        # 邻居判定半径（默认 30.0 米），可以通过 launch 参数动态调整
        self.nb_sense_radius = rospy.get_param('~nb_sense_radius', 30.0)
        self.collision_radius = rospy.get_param('~collision_radius', 1.0)
        
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
        rospy.loginfo(f"[Neighborhood] Calculator Started! Sensing Radius: {self.nb_sense_radius}m, Collision Radius: {self.collision_radius}m")

    def obstacle_callback(self, msg):
        """解析环境节点发来的 Marker，缓存为 NED 坐标系下的纯数学几何体"""
        parsed_obstacles = []
        for marker in msg.markers:
            px, py, pz = marker.pose.position.x, marker.pose.position.y, marker.pose.position.z
            
            if marker.header.frame_id == 'map':
                pos_ned = (py, px, -pz)
            else:
                pos_ned = (px, py, pz)
                
            parsed_obstacles.append({
                'id': f"Obstacle_{marker.id}",
                'type': marker.type,
                'pos': pos_ned,
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
                        rospy.loginfo(f"[Neighborhood] Automatically hooked {uuv_id}. Dedicated topic created.")

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
        CR = self.collision_radius
        current_collisions = set()
        
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
                        n_i = Neighbor3D()
                        n_i.uuv_name = uuv_i
                        n_i.distance = dist
                        n_i.state = state_i
                        neighborhoods[uuv_j].neighbors.append(n_i)

                    # UUV之间碰撞检测
                    if dist < CR:
                        pair_key = tuple(sorted([uuv_i, uuv_j]))
                        current_collisions.add(pair_key)
                        if pair_key not in self.active_collisions:
                            rospy.logwarn(f"\033[93m[WARN] UUV Collision! {uuv_i} hit {uuv_j}! Dist: {dist:.2f}m\033[0m")
                        info = CollisionInfo(); info.uuv_id = uuv_i; info.target_id = uuv_j; info.distance = dist
                        collision_msg_array.collisions.append(info)

        # 3. 动态/静态障碍物碰撞判定
        for obs in self.obstacles:
            obs_id = obs['id']
            cx, cy, cz = obs['pos']
            sx, sy, sz = obs['scale']
            
            for uuv_id, state in states.items():
                dx = state.x - cx; dy = state.y - cy; dz = state.z - cz
                is_hit = False; dist = 0.0
                
                if obs['type'] == Marker.SPHERE:  
                    obs_r = sx / 2.0  
                    dist = math.sqrt(dx**2 + dy**2 + dz**2)
                    if dist < obs_r + CR: is_hit = True
                        
                elif obs['type'] == Marker.CUBE: 
                    px = max(-sx/2.0, min(dx, sx/2.0)); py = max(-sy/2.0, min(dy, sy/2.0)); pz = max(-sz/2.0, min(dz, sz/2.0))
                    dist = math.sqrt((dx-px)**2 + (dy-py)**2 + (dz-pz)**2)
                    if dist < CR: is_hit = True
                        
                elif obs['type'] == Marker.CYLINDER:
                    a, b = sx / 2.0, sy / 2.0; h = sz
                    if abs(dz) < h/2.0 + CR: 
                        if (dx**2 / (a + CR)**2) + (dy**2 / (b + CR)**2) <= 1.0: 
                            is_hit = True; dist = math.sqrt(dx**2 + dy**2)

                if is_hit:
                    pair_key = (uuv_id, obs_id)
                    current_collisions.add(pair_key)
                    if pair_key not in self.active_collisions:
                        rospy.logwarn(f"\033[91m[WARN] Environment Hit! {uuv_id} crashed into {obs_id}!\033[0m")
                    info = CollisionInfo(); info.uuv_id = uuv_id; info.target_id = obs_id; info.distance = dist
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