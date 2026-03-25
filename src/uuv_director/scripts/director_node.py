#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from uuv_interface.msg import TargetPoint3D, TargetPoint3DArray
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

# 引入我们刚刚改造好的两个核心组件
from uuv_manager import UUVManager
from uuv_loader import UUVLoader
from scenarios import ScenarioFactory

class DirectorNode:
    def __init__(self):
        rospy.init_node('director_node')
        
        # 1. 实例化核心组件 (内存级零延迟交互)
        self.manager = UUVManager()
        self.loader = UUVLoader()
        
        self.start_ros_time = rospy.get_time()

        self.mission_pub = rospy.Publisher('/swarm/mission_targets', TargetPoint3DArray, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher('/scenario/target_markers', MarkerArray, queue_size=1, latch=True)

        # 3. 初始化任务剧本
        scenario_name = rospy.get_param("~scenario", "swarm_nav")
        self.scenario = ScenarioFactory.create(scenario_name, self)
        
        # 注册退出清理钩子
        rospy.on_shutdown(self.on_shutdown)

    def on_shutdown(self):
        rospy.loginfo("[Director] Shutting down... cleaning up spawned processes.")
        self.loader.cleanup()

    def publish_missions(self, target_list):
        """提供给剧本的接口：一键广播任务列表"""
        msg = TargetPoint3DArray()
        msg.targets = target_list
        self.mission_pub.publish(msg)
        rospy.loginfo(f"[Director] Broadcasted {len(target_list)} mission(s) to the entire swarm.")

    def publish_target_markers(self, targets, reached_count=0, custom_color=None, clear_old=False, scale=5.0):
        marker_array = MarkerArray()
        
        if clear_old:
            delete_marker = Marker()
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)

        for i, target in enumerate(targets):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "scenario_targets"
            marker.id = target.id  # 直接使用物理目标的 ID
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # NED 转 ENU (供Rviz显示): x=E, y=N, z=-D
            marker.pose.position.x = target.e
            marker.pose.position.y = target.n
            marker.pose.position.z = -target.d
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = scale
            
            if custom_color:
                marker.color = ColorRGBA(r=custom_color[0], g=custom_color[1], b=custom_color[2], a=0.7)
            else:
                if i < reached_count:
                    marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.6) # 绿色已到达
                else:
                    marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5) # 红色未到达
            marker.frame_locked = True
            marker_array.markers.append(marker)
            
        self.marker_pub.publish(marker_array)

    def run(self):
        """
        双频控制核心：
        - 物理层 (manager): 60 Hz 保证邻居发现和碰撞检测的实时性
        - 决策层 (scenario): 1 Hz 保证宏观编队决策的稳定性
        """
        rate = rospy.Rate(50) # 60Hz
        tick = 0
        
        while not rospy.is_shutdown():
            current_sim_time = rospy.get_time() - self.start_ros_time
            
            # 【物理层】：60Hz 更新一次 UUV 的邻居和碰撞
            self.manager.update()
            
            # 【决策层】：每 10 个 tick (即 1 秒) 更新一次剧本
            if tick % 10 == 0:
                # 把整个 manager 传给剧本，让剧本拥有上帝视角！
                self.scenario.update(current_sim_time, self.manager)
                
            tick += 1
            rate.sleep()

if __name__ == '__main__':
    try:
        node = DirectorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass