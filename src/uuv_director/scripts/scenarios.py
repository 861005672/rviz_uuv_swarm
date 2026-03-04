#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from uuv_interface.msg import TargetPoint3D
from uuv_interface.srv import SetTargetPoint3D

class ScenarioBase:
    """剧本基类"""
    def __init__(self, director):
        self.director = director # 持有主节点的引用，以便调用服务或获取数据

    def update(self, current_time, swarm_states):
        """每轮循环执行的逻辑，由子类实现"""
        raise NotImplementedError

class SwarmNavScenario(ScenarioBase):
    """
    示例剧本：集群导航测试
    5s 时所有 UUV 出发去点 A，30s 时切换到点 B
    """
    def __init__(self, director):
        super().__init__(director)
        self.phase = 0
        rospy.loginfo("[Scenario] SwarmNavScenario Initialized.")

    def update(self, current_time, swarm_states):
        # 阶段 1: 5秒时统一触发启动
        if self.phase == 0 and current_time > 5.0:
            rospy.loginfo("[Scenario] Phase 1: Sending Swarm to Target A...")
            target = TargetPoint3D(id=1, n=50.0, e=50.0, d=10.0)
            self.director.send_swarm_target(target)
            self.phase = 1

        # 阶段 2: 30秒时变换目标
        elif self.phase == 1 and current_time > 30.0:
            rospy.loginfo("[Scenario] Phase 2: Switching Swarm to Target B...")
            target = TargetPoint3D(id=2, n=100.0, e=-20.0, d=25.0)
            self.director.send_swarm_target(target)
            self.phase = 2

class ScenarioFactory:
    @staticmethod
    def create(name, director):
        if name == "swarm_nav":
            return SwarmNavScenario(director)
        else:
            rospy.logwarn(f"Unknown scenario name: {name}, using default.")
            return ScenarioBase(director)