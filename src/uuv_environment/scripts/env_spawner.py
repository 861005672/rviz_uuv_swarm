#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray

def create_marker(obs_idx, obs_data):
    """根据字典数据生成单个 RViz Marker"""
    marker = Marker()
    marker.header.stamp = rospy.Time()
    marker.ns = "environment_obstacles"
    marker.id = obs_idx
    
    # 解析并设置独立的参考坐标系
    # 如果没填，默认使用 'map'
    ref_coord = obs_data.get('ref_coord', 'map')
    
    # 建立映射：为了方便配置文件书写，如果是 'ROS' 就映射到系统标准的 'map'
    # 如果是 'NED'，就映射到我们自定义的 'ned'，其余的原样照搬
    if ref_coord.upper() == 'ROS':
        marker.header.frame_id = 'map'
    elif ref_coord.upper() == 'NED':
        marker.header.frame_id = 'ned'
    else:
        marker.header.frame_id = ref_coord # 支持任意自定义 TF 名称
        
    # 解析形状类型
    obs_type = obs_data.get('type', 'box')
    if obs_type == 'sphere': marker.type = Marker.SPHERE
    elif obs_type == 'cylinder': marker.type = Marker.CYLINDER
    else: marker.type = Marker.CUBE
        
    marker.action = Marker.ADD
    
    # 解析位置
    pos = obs_data.get('position', [0.0, 0.0, 0.0])
    marker.pose.position.x = float(pos[0])
    marker.pose.position.y = float(pos[1])
    marker.pose.position.z = float(pos[2])
    
    # 默认姿态
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    
    # 解析尺寸
    scale = obs_data.get('scale', [1.0, 1.0, 1.0])
    marker.scale.x = float(scale[0])
    marker.scale.y = float(scale[1])
    marker.scale.z = float(scale[2])
    
    # 解析颜色
    color = obs_data.get('color', [1.0, 1.0, 1.0, 1.0])
    marker.color.r = float(color[0])
    marker.color.g = float(color[1])
    marker.color.b = float(color[2])
    marker.color.a = float(color[3])
    
    marker.lifetime = rospy.Duration(0)
    return marker

def broadcast_world_to_ned():
    """广播 map -> ned 的静态 TF 变换"""
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transform = TransformStamped()
    
    static_transform.header.stamp = rospy.Time.now()
    static_transform.header.frame_id = "map"
    static_transform.child_frame_id = "ned"
    
    static_transform.transform.translation.x = 0.0
    static_transform.transform.translation.y = 0.0
    static_transform.transform.translation.z = 0.0
    
    # ENU 到 NED 的固定旋转四元数 (绕X轴转180度，再绕Z轴转90度)
    static_transform.transform.rotation.x = 0.7071068
    static_transform.transform.rotation.y = 0.7071068
    static_transform.transform.rotation.z = 0.0
    static_transform.transform.rotation.w = 0.0
    
    broadcaster.sendTransform(static_transform)
    rospy.loginfo(">> 已建立基础静态 TF 树: [map] -> [ned]")

def main():
    rospy.init_node('env_spawner_node', anonymous=True)
    marker_pub = rospy.Publisher('/env_markers', MarkerArray, queue_size=10)
    
    # 优先广播世界坐标系关系，为后续所有物体提供参照准绳
    broadcast_world_to_ned()
    
    rospy.sleep(0.5)
    obstacles_cfg = rospy.get_param('~obstacles', [])
    
    if not obstacles_cfg:
        rospy.logwarn("未能读取到障碍物配置，请检查 YAML。")
    else:
        rospy.loginfo(f">> 正在根据各自指定的参考坐标系生成 {len(obstacles_cfg)} 个环境实体。")

    rate = rospy.Rate(1) 
    
    while not rospy.is_shutdown():
        marker_array = MarkerArray()
        for i, obs in enumerate(obstacles_cfg):
            marker = create_marker(i, obs)
            marker_array.markers.append(marker)
            
        marker_pub.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass