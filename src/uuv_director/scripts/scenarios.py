#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from uuv_interface.msg import TargetPoint3D
from uuv_interface.srv import SetTargetPoint3D

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
        rospy.loginfo("[Scenario] SwarmNavScenario Initialized.")

    def update(self, current_time, manager):
        if self.phase == 0 and current_time > 15.0:
            rospy.loginfo("[Scenario] Phase 1: Broadcasting Swarm Target A...")
            t = TargetPoint3D(id=1, n=100.0, e=0.0, d=0.0)
            # 【修改】：放入列表中并通过广播接口下发
            self.director.publish_missions([t]) 
            self.phase = 1

        # elif self.phase == 1 and current_time > 30.0:
        #     rospy.loginfo("[Scenario] Phase 2: Switching Swarm Target B...")
        #     t = TargetPoint3D(id=2, n=100.0, e=-20.0, d=25.0)
        #     # 【修改】：放入列表中并通过广播接口下发
        #     self.director.publish_missions([t])
        #     self.phase = 2

class ScenarioFactory:
    @staticmethod
    def create(name, director):
        if name == "swarm_nav":
            return SwarmNavScenario(director)
        else:
            rospy.logwarn(f"Unknown scenario name: {name}, using default.")
            return ScenarioBase(director)