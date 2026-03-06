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

    def update(self, current_time, manager):
        # 确保标记点被发布
        reached_count = len(self.original_targets) - len(self.active_targets)
        self.director.publish_target_markers(self.original_targets, reached_count=reached_count)

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

class CBAATestScenario(ScenarioBase):
    def __init__(self, director):
        super().__init__(director)
        self.phase = 0
        self.marker_pub = rospy.Publisher('/scenario/target_markers', MarkerArray, queue_size=1, latch=True)
        self.current_targets = []

        self.uuv_count = rospy.get_param("~uuv_count", 30)
        self.phase1_target = rospy.get_param("~phase1_target", [300.0, 0.0, 20.0])
        
        default_p2_targets = [[800.0, 400.0, 10.0], [800.0, -400.0, 30.0], [1000.0, 0.0, 50.0]]
        default_p2_reqs = [10, 10, 10]
        self.phase2_targets = rospy.get_param("~phase2_targets", default_p2_targets)
        self.phase2_reqs = rospy.get_param("~phase2_reqs", default_p2_reqs)

        rospy.loginfo("[Scenario] CBAATestScenario Initialized. Waiting for 10s to start Phase 1 (Aggregation).")

    def update(self, current_time, manager):
        if self.uuv_count == 0:
            return

        # Phase 0: 等待至 15 秒，下发聚合任务
        if self.phase == 0 and current_time > 15.0:
            rospy.loginfo(f"[Scenario] Phase 1: 发布单目标点，要求所有 {self.uuv_count} 架 UUV 聚集...")
            t1 = TargetPoint3D()
            t1.id = 1
            t1.n = self.phase1_target[0]
            t1.e = self.phase1_target[1]
            t1.d = self.phase1_target[2]
            t1.required_uuvs = self.uuv_count
            
            self.current_targets = [t1]
            self.director.publish_missions(self.current_targets)
            
            # 使用青色，大球，清空旧标记
            self.director.publish_target_markers(self.current_targets, custom_color=(0.0, 1.0, 1.0), clear_old=True, scale=10.0)
            self.phase = 1

        # Phase 1: 等待至 60 秒，下发分群任务
        elif self.phase == 1 and current_time > 60.0:
            rospy.loginfo(f"[Scenario] Phase 2: 发布 {len(self.phase2_targets)} 个目标点，触发 CBAA 自动分群...")
            
            self.current_targets = []
            target_id_counter = 2 # 基础物理 ID 依次递增
            
            for i in range(len(self.phase2_targets)):
                pt = self.phase2_targets[i]
                req = self.phase2_reqs[i] if i < len(self.phase2_reqs) else 1
                
                t = TargetPoint3D()
                t.id = target_id_counter
                t.n = pt[0]
                t.e = pt[1]
                t.d = pt[2]
                t.required_uuvs = req
                
                self.current_targets.append(t)
                target_id_counter += 1

            self.director.publish_missions(self.current_targets)
            
            # 使用橙色，大球，清空第一阶段的标记
            self.director.publish_target_markers(self.current_targets, custom_color=(1.0, 0.5, 0.0), clear_old=True, scale=10.0)
            self.phase = 2



class ScenarioFactory:
    @staticmethod
    def create(name, director):
        if name == "swarm_nav":
            return SwarmNavScenario(director)
        elif name == "cbaa_test":                 # 【新增】：注册测试剧本
            return CBAATestScenario(director)
        else:
            rospy.logwarn(f"Unknown scenario name: {name}, using default.")
            return ScenarioBase(director)