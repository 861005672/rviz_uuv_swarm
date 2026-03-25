#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from std_msgs.msg import Float64
from uuv_interface.msg import Neighborhood3D, State3D

class DebugPlotter:
    def __init__(self):
        rospy.init_node('debug_plotter', anonymous=True)
        
        # 1. 发布测距话题 (相对极坐标)
        self.pub_real_dist = rospy.Publisher('/debug/distance/real', Float64, queue_size=10)
        self.pub_passive_dist = rospy.Publisher('/debug/distance/passive', Float64, queue_size=10)
        
        # 2. 发布方位角 (Azimuth/Yaw) 话题
        self.pub_real_azimuth = rospy.Publisher('/debug/azimuth/real', Float64, queue_size=10)
        self.pub_passive_azimuth = rospy.Publisher('/debug/azimuth/passive', Float64, queue_size=10)
        
        # 3. 发布俯仰角 (Elevation/Pitch) 话题
        self.pub_real_elevation = rospy.Publisher('/debug/elevation/real', Float64, queue_size=10)
        self.pub_passive_elevation = rospy.Publisher('/debug/elevation/passive', Float64, queue_size=10)
        
        # --- 新增：发布全局 NED 坐标话题 (绝对笛卡尔坐标) ---
        # N (North) 对应 state.x
        self.pub_real_n = rospy.Publisher('/debug/n/real', Float64, queue_size=10)
        self.pub_passive_n = rospy.Publisher('/debug/n/passive', Float64, queue_size=10)
        
        # E (East) 对应 state.y
        self.pub_real_e = rospy.Publisher('/debug/e/real', Float64, queue_size=10)
        self.pub_passive_e = rospy.Publisher('/debug/e/passive', Float64, queue_size=10)
        
        # D (Down) 对应 state.z
        self.pub_real_d = rospy.Publisher('/debug/d/real', Float64, queue_size=10)
        self.pub_passive_d = rospy.Publisher('/debug/d/passive', Float64, queue_size=10)
        
        self.self_state = None
        
        # 订阅自身的真实状态（用于计算真实相对方位）
        rospy.Subscriber('/uuv_0/state', State3D, self.state_callback)
        # 订阅包含传感数据的邻居消息
        rospy.Subscriber('/uuv_0/neighborhood', Neighborhood3D, self.nb_callback)
        
        rospy.loginfo("Debug plotter started. Ready to plot Relative (Dist, Az, El) and Global (N, E, D) metrics...")

    def state_callback(self, msg):
        self.self_state = msg
        
    def calculate_true_relative_angles(self, self_state, nb_state):
        """与 uuv_manager 中完全一致的局部坐标系解算公式"""
        dx = nb_state.x - self_state.x
        dy = nb_state.y - self_state.y
        dz = nb_state.z - self_state.z
        
        # 绕 Z, Y, X 轴逆旋转，求得自身坐标系下的相对位置
        x1 = dx * math.cos(self_state.yaw) + dy * math.sin(self_state.yaw)
        y1 = -dx * math.sin(self_state.yaw) + dy * math.cos(self_state.yaw)
        z1 = dz
        
        x2 = x1 * math.cos(self_state.pitch) - z1 * math.sin(self_state.pitch)
        y2 = y1
        z2 = x1 * math.sin(self_state.pitch) + z1 * math.cos(self_state.pitch)
        
        rel_x = x2
        rel_y = y2 * math.cos(self_state.roll) + z2 * math.sin(self_state.roll)
        rel_z = -y2 * math.sin(self_state.roll) + z2 * math.cos(self_state.roll)
        
        # 视线方位角解算
        rel_yaw = math.atan2(rel_y, rel_x)
        rel_pitch = math.atan2(-rel_z, math.hypot(rel_x, rel_y))
        
        return rel_yaw, rel_pitch

    def nb_callback(self, msg):
        if self.self_state is None:
            return
            
        for nb in msg.neighbors:
            # 锁定观察目标，假设你在看 uuv_1
            if nb.uuv_name == 'uuv_1':
                # --- 1. 发布相对距离 ---
                self.pub_real_dist.publish(nb.distance)
                self.pub_passive_dist.publish(nb.passive_distance)
                
                # --- 2. 计算并发布相对角度 ---
                true_azimuth, true_elevation = self.calculate_true_relative_angles(self.self_state, nb.state)
                
                self.pub_real_azimuth.publish(true_azimuth)
                self.pub_passive_azimuth.publish(nb.passive_direction_azimuth)
                
                self.pub_real_elevation.publish(true_elevation)
                self.pub_passive_elevation.publish(nb.passive_direction_elevation)
                
                # --- 3. 新增：发布全局 NED 坐标 (x, y, z) ---
                # 发布 N (North / X轴)
                self.pub_real_n.publish(nb.state.x)
                self.pub_passive_n.publish(nb.passive_state.x)
                
                # 发布 E (East / Y轴)
                self.pub_real_e.publish(nb.state.y)
                self.pub_passive_e.publish(nb.passive_state.y)
                
                # 发布 D (Down / Z轴)
                self.pub_real_d.publish(nb.state.z)
                self.pub_passive_d.publish(nb.passive_state.z)
                
                break

if __name__ == '__main__':
    try:
        DebugPlotter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass