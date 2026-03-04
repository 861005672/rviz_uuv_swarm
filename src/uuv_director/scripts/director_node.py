#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from uuv_interface.msg import State3D, State3DArray, TargetPoint3D, CollisionInfoArray
from uuv_interface.srv import SetTargetPoint3D
from scenarios import ScenarioFactory

class DirectorNode:
    def __init__(self):
        rospy.init_node('director_node')
        
        # 1. 参数与状态记录
        self.uuv_targets = {}  # 存储每个UUV的服务客户端 { 'uuv_0': rospy.ServiceProxy, ... }
        self.swarm_states = {} # 存储最新的全群状态 { 'uuv_0': State3D, ... }
        self.start_ros_time = rospy.get_time()
        
        # 2. 订阅汇总状态信息 (来自 StateAggregatorNodelet)
        # 这样比单独订阅50个State更高效
        rospy.Subscriber("/swarm/all_states", State3DArray, self.swarm_states_callback)
        rospy.Subscriber("/swarm/collision_info", CollisionInfoArray, self.collision_callback)

        # 3. 动态识别 UUV 并建立服务连接
        self.scan_and_connect_uuvs()

        # 4. 初始化剧本
        scenario_name = rospy.get_param("~scenario", "swarm_nav")
        self.scenario = ScenarioFactory.create(scenario_name, self)

        # 5. 主循环控制
        rate = rospy.Rate(1) # 剧本调控频率不需要太高，1Hz 足够
        while not rospy.is_shutdown():
            current_sim_time = rospy.get_time() - self.start_ros_time
            # 执行剧本逻辑
            self.scenario.update(current_sim_time, self.swarm_states)
            rate.sleep()

    def scan_and_connect_uuvs(self):
        """扫描当前所有以 uuv_ 开头的服务并连接"""
        rospy.loginfo("[Director] Scanning for UUV guidance services...")
        # 简单暴力：假设 UUV 编号连续或从 master 获取已发布的 service 列表
        all_services = rospy.get_published_topics() # 仅示意，实际可用 rosservice list 逻辑
        # 这里的逻辑建议参考你的 uuv_loader.py
        # 这里演示连接前 50 个
        for i in range(50):
            ns = f"uuv_{i}"
            srv_name = f"/{ns}/set_guidance_target"
            try:
                rospy.wait_for_service(srv_name, timeout=0.5)
                self.uuv_targets[ns] = rospy.ServiceProxy(srv_name, SetTargetPoint3D)
                rospy.loginfo(f"[Director] Connected to {srv_name}")
            except:
                continue

    def swarm_states_callback(self, msg):
        for state in msg.states:
            self.swarm_states[f"uuv_{state.id}"] = state

    def collision_callback(self, msg):
        # 如果有剧本需要根据碰撞信息做决策，可以在这里处理
        pass

    def send_swarm_target(self, target_msg):
        """一键为全群下发目标"""
        for ns, client in self.uuv_targets.items():
            try:
                client.call(target_msg)
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed for {ns}: {e}")

if __name__ == '__main__':
    try:
        DirectorNode()
    except rospy.ROSInterruptException:
        pass