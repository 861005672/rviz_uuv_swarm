#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from uuv_interface.msg import TargetPoint3D
from uuv_interface.srv import SetTargetPoint3D
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

class ScenarioBase:
    """剧本基类"""
    def __init__(self, director):
        self.director = director # 持有主节点的引用，以便调用服务或获取数据

    def update(self, current_time, manager):
        """每轮循环执行的逻辑，由子类实现"""
        raise NotImplementedError


class SwarmNavScenario(ScenarioBase):
    def __init__(self, director):
        super().__init__(director)
        self.phase = 0
        
        # 1. 从参数服务器获取目标点列表，如果没有配置则使用默认值
        # 格式为: [[n1, e1, d1], [n2, e2, d2], ...]
        default_targets = [[500.0, 0.0, 0.0], [500.0, -500.0, 100.0], [0.0, 0.0, 0.0]]
        target_list_param = rospy.get_param("~target_list", default_targets)
        self.tolerance = rospy.get_param("~arrival_tolerance", 15.0) # 到达判定半径 (米)
        
        self.original_targets = []
        for i, pt in enumerate(target_list_param):
            t = TargetPoint3D()
            t.id = i + 1
            t.n = pt[0]
            t.e = pt[1]
            t.d = pt[2]
            self.original_targets.append(t)
            
        # 动态的目标点队列，达到一个就会弹出一个
        self.active_targets = list(self.original_targets)
        
        # 提取第一个目标点作为停止点 (id = -1)
        self.stop_target = TargetPoint3D()
        if self.original_targets:
            self.stop_target.id = -1
            self.stop_target.n = self.original_targets[-1].n
            self.stop_target.e = self.original_targets[-1].e
            self.stop_target.d = self.original_targets[-1].d

        # 用于可视化目标点的发布器
        self.marker_pub = rospy.Publisher('/scenario/target_markers', MarkerArray, queue_size=1, latch=True)

        rospy.loginfo(f"[Scenario] SwarmNavScenario Initialized with {len(self.original_targets)} targets.")

    def publish_target_markers(self):
        """发布固定数量的目标点 Marker 用于 Rviz 显示"""
        marker_array = MarkerArray()
        reached_count = len(self.original_targets) - len(self.active_targets)
        for i, target in enumerate(self.original_targets):
            marker = Marker()
            marker.header.frame_id = "map"  # 全局地图坐标系
            marker.header.stamp = rospy.Time.now()
            marker.ns = "scenario_targets"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # NED 转 ENU (供Rviz显示): x=E, y=N, z=-D
            marker.pose.position.x = target.e
            marker.pose.position.y = target.n
            marker.pose.position.z = -target.d
            marker.pose.orientation.w = 1.0
            
            # 设置球体大小和颜色
            marker.scale.x = 5.0
            marker.scale.y = 5.0
            marker.scale.z = 5.0
            if i < reached_count:
                # 已到达的目标点变绿色
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.6) 
            else:
                # 未到达的目标点保持红色
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)
            marker.frame_locked = True

            marker_array.markers.append(marker)
            
        self.marker_pub.publish(marker_array)

    def update(self, current_time, manager):
        # 确保标记点被发布
        self.publish_target_markers()

        # Phase 0: 等待至 20 秒
        if self.phase == 0 and current_time > 20.0:
            rospy.loginfo(f"[Scenario] Phase 1: 达到20s，开始下发初始目标点列表 (数量: {len(self.active_targets)})...")
            self.director.publish_missions(self.active_targets) 
            self.phase = 1

        # Phase 1: 导航阶段
        elif self.phase == 1:
            uuv_states = manager.uuv_states
            if not uuv_states or len(self.active_targets) == 0:
                return

            # --- 功能1：统计所有UUV的位置，计算集群中心 ---
            sum_n, sum_e, sum_d = 0.0, 0.0, 0.0
            count = len(uuv_states)
            for uuv_id, state in uuv_states.items():
                sum_n += state.x
                sum_e += state.y
                sum_d += state.z
                
            avg_n = sum_n / count
            avg_e = sum_e / count
            avg_d = sum_d / count

            # 获取当前的第一优先级目标点
            current_target = self.active_targets[0]
            
            # 计算集群中心与当前目标点的距离
            dist = math.sqrt((avg_n - current_target.n)**2 + 
                             (avg_e - current_target.e)**2 + 
                             (avg_d - current_target.d)**2)

            # --- 功能2 & 功能4：判断是否到达目标点并管理队列 ---
            if dist < self.tolerance:
                rospy.loginfo(f"[Scenario] 集群到达目标点 ID: {current_target.id} (当前中心误差: {dist:.2f}m)!")
                
                # 到达后将列表的第一个点弹出
                self.active_targets.pop(0)
                
                if len(self.active_targets) > 0:
                    # 如果列表中还有目标点，下发更新后的目标点列表
                    rospy.loginfo(f"[Scenario] 目标已更新！下发剩余 {len(self.active_targets)} 个目标。")
                    self.director.publish_missions(self.active_targets)
                else:
                    # --- 功能4：到达最后一个点，返回第一个点并停止 ---
                    rospy.loginfo("[Scenario] 集群到达最后一个目标！停止 (ID置为-1)。")
                    self.director.publish_missions([self.stop_target])
                    self.phase = 2  # 进入结束阶段



class ScenarioFactory:
    @staticmethod
    def create(name, director):
        if name == "swarm_nav":
            return SwarmNavScenario(director)
        else:
            rospy.logwarn(f"Unknown scenario name: {name}, using default.")
            return ScenarioBase(director)