#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from uuv_interface.msg import State3D, Neighbor3D, Neighborhood3D

class NeighborhoodCalculator:
    def __init__(self):
        rospy.init_node('neighborhood_calculator', anonymous=True)
        # 邻居判定半径（默认 30.0 米），可以通过 launch 参数动态调整
        self.nb_sense_radius = rospy.get_param('~nb_sense_radius', 30.0)
        # 计算与发布频率：10Hz 足够满足底层的平滑度要求
        self.rate = rospy.Rate(10)
        
        self.uuv_states = {}
        self.subscribers = {}
        self.publishers = {}
        
        # 定时器：每 1 秒扫描一次系统中是否有新加载的 UUV
        self.discover_timer = rospy.Timer(rospy.Duration(1.0), self.discover_uuvs)
        rospy.loginfo(f"[Neighborhood] Calculator Started! Sensing Radius: {self.nb_sense_radius}m")

    def discover_uuvs(self, event):
        """动态发现新的 UUV，并自动为它建立专属的订阅器与发布器"""
        topics = rospy.get_published_topics()
        for topic_name, topic_type in topics:
            if '/state' in topic_name and 'uuv_interface/State3D' in topic_type:
                # 提取扁平化的 UUV 名称，例如从 "/uuv_15/state" 提取出 "uuv_15"
                parts = topic_name.split('/')
                if len(parts) >= 3 and parts[1].startswith('uuv_'):
                    uuv_id = parts[1]
                    
                    if uuv_id not in self.subscribers:
                        # 订阅它的状态
                        self.subscribers[uuv_id] = rospy.Subscriber(
                            topic_name, State3D, self.state_callback, callback_args=uuv_id)
                        # 【核心架构】为它建立专属的邻居话题发布器
                        self.publishers[uuv_id] = rospy.Publisher(
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

    def calculate_and_publish(self):
        # 获取当前所有 UUV 的状态快照
        states = dict(self.uuv_states)
        uuvs = list(states.keys())
        n = len(uuvs)
        
        # 初始化每个 UUV 的专属空邻居列表
        neighborhoods = {uuv: Neighborhood3D() for uuv in uuvs}
        
        if n > 1:
            # O(N^2 / 2) 算法：双重遍历，计算距离并双向添加（避免重复计算 A到B 和 B到A）
            for i in range(n):
                uuv_i = uuvs[i]
                state_i = states[uuv_i]
                
                for j in range(i + 1, n):
                    uuv_j = uuvs[j]
                    state_j = states[uuv_j]

                    # 计算 3D 欧几里得距离
                    dx = state_i.x - state_j.x
                    dy = state_i.y - state_j.y
                    dz = state_i.z - state_j.z
                    dist = math.sqrt(dx*dx + dy*dy + dz*dz)

                    # 若在感知半径内，则互相加为邻居
                    if dist <= self.nb_sense_radius:
                        # 把 J 加入 I 的邻居列表
                        n_j = Neighbor3D()
                        n_j.uuv_name = uuv_j
                        n_j.distance = dist
                        n_j.state = state_j
                        neighborhoods[uuv_i].neighbors.append(n_j)

                        # 把 I 加入 J 的邻居列表
                        n_i = Neighbor3D()
                        n_i.uuv_name = uuv_i
                        n_i.distance = dist
                        n_i.state = state_i
                        neighborhoods[uuv_j].neighbors.append(n_i)

        # 遍历发布：每个 UUV 只收到属于自己的那一份数据
        for uuv in uuvs:
            if uuv in self.publishers:
                self.publishers[uuv].publish(neighborhoods[uuv])

if __name__ == '__main__':
    try:
        node = NeighborhoodCalculator()
        node.run()
    except rospy.ROSInterruptException:
        pass