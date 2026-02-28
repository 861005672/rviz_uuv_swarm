#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
import os
import math
import random
import subprocess
from uuv_director.srv import UUVLoader, UUVLoaderResponse
import datetime

class UUVLoaderNode:
    def __init__(self):
        rospy.init_node('uuv_loader_node', anonymous=True)
        self.active_processes = []
        self.all_spawned_poses = []
        self.current_group_idx = 0   # 专门用于区分计算资源 (Manager)
        self.global_uuv_idx = 0      # 【核心新增】专门用于全局唯一递增的 UUV ID
        self.launch_count = 0
        self.rospack = rospkg.RosPack()
        
        self.srv = rospy.Service('uuv_loader', UUVLoader, self.handle_loader_request)
        rospy.loginfo("UUV Loader Service is Ready. Flat namespace [uuv_X] enabled!")

    def resolve_path(self, path):
        if path.startswith('/'): return path
        elif path.startswith('./'):
            try:
                pkg_path = self.rospack.get_path('uuv_director')
                ws_root = os.path.dirname(os.path.dirname(pkg_path))
                return os.path.normpath(os.path.join(ws_root, path[2:]))
            except Exception as e:
                return "/tmp/dynamic_swarm.launch"
        return path

    def generate_random_poses(self, total, center, radius, min_dist):
        poses = []
        max_attempts = 2000
        for i in range(total):
            placed = False
            for attempt in range(max_attempts):
                r = radius * (random.random() ** (1.0 / 3.0))
                theta = random.uniform(0, 2 * math.pi)
                phi = math.acos(2 * random.random() - 1)
                
                x = center[0] + r * math.sin(phi) * math.cos(theta)
                y = center[1] + r * math.sin(phi) * math.sin(theta)
                z = center[2] + r * math.cos(phi)
                yaw = random.uniform(-math.pi, math.pi)
                
                collision = False
                for px, py, pz, _ in self.all_spawned_poses + poses:
                    dist = math.sqrt((x - px)**2 + (y - py)**2 + (z - pz)**2)
                    if dist < min_dist:
                        collision = True
                        break
                
                if not collision:
                    poses.append((x, y, z, yaw))
                    placed = True
                    break
                    
            if not placed:
                rospy.logwarn(f"Failed to place UUV after {max_attempts} attempts. Space too crowded.")
                return None
        return poses

    def generate_xml_launch(self, poses, group_size, dyn_type, ctrl_type, gui_type):
        xml_lines = ['<?xml version="1.0"?>', '<launch>']
        total_uuvs = len(poses)
        num_managers = math.ceil(total_uuvs / group_size)
        uuv_idx = 0
        
        for m_idx in range(num_managers):
            group_name = f"group{self.current_group_idx}"
            self.current_group_idx += 1 
            
            current_group_count = min(group_size, total_uuvs - uuv_idx)
            start_global_id = self.global_uuv_idx + uuv_idx
            
            # --- 1. 计算层隔离：只生成 Nodelet Manager 和 Aggregator ---
            xml_lines.append(f'  <group ns="{group_name}">')
            xml_lines.append(f'    <node pkg="nodelet" type="nodelet" name="{group_name}_manager" args="manager" output="screen"/>')
            xml_lines.append(f'    <node pkg="nodelet" type="nodelet" name="state_aggregator" args="load uuv_control/StateAggregatorNodelet {group_name}_manager">')
            xml_lines.append(f'      <param name="group_name" value="{group_name}"/>')
            xml_lines.append(f'      <param name="start_global_id" value="{start_global_id}"/>')
            xml_lines.append(f'      <param name="uuv_count" value="{current_group_count}"/>')
            xml_lines.append('    </node>')
            xml_lines.append('  </group>')
            
            # --- 2. 身份层扁平化：把 UUV 挂在 Launch 最外层 ---
            for local_idx in range(current_group_count):
                x, y, z, yaw = poses[uuv_idx]
                
                # 计算全局绝对 ID
                current_id = self.global_uuv_idx + uuv_idx
                uuv_ns = f"uuv_{current_id}"
                
                xml_lines.append(f'  <group ns="{uuv_ns}">')
                xml_lines.append(f'    <param name="robot_description" command="$(find xacro)/xacro \'$(find uuv_description)/urdf/lauv_model.xacro\' namespace:={uuv_ns} dynamics_type:={dyn_type} controller_type:={ctrl_type} guidance_type:={gui_type}"/>')
                
                # 控制节点直接跨命名空间加载到对应 group 的 Manager 中
                xml_lines.append(f'    <node pkg="nodelet" type="nodelet" name="uuv_control" args="load uuv_control/ControlNodelet /{group_name}/{group_name}_manager">')
                xml_lines.append(f'      <param name="init_x" value="{x:.3f}"/>')
                xml_lines.append(f'      <param name="init_y" value="{y:.3f}"/>')
                xml_lines.append(f'      <param name="init_z" value="{z:.3f}"/>')
                xml_lines.append(f'      <param name="init_yaw" value="{yaw:.3f}"/>')
                xml_lines.append('      <param name="visual_rate" value="20.0"/>')
                xml_lines.append('    </node>')
                
                xml_lines.append('    <node pkg="robot_state_publisher" type="robot_state_publisher" name="rsp" />')
                xml_lines.append('  </group>')
                uuv_idx += 1
                
        # 更新总发射数量
        self.global_uuv_idx += total_uuvs
        
        xml_lines.append('</launch>')
        return "\n".join(xml_lines)

    def handle_loader_request(self, req):
        if req.init_type == "deterministic":
            return UUVLoaderResponse(False, "Deterministic mode is not implemented yet.")
            
        rospy.loginfo(f"Generating {req.total_uuvs} appended UUVs in random mode...")
        poses = self.generate_random_poses(req.total_uuvs, req.center, req.radius, req.min_dist)
        
        if poses is None:
            return UUVLoaderResponse(False, "Failed to generate collision-free poses. Space is too crowded.")
            
        self.all_spawned_poses.extend(poses) 
        
        xml_content = self.generate_xml_launch(poses, req.group_size, req.dynamics_type, req.controller_type, req.guidance_type)
        
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        default_filename = f'./dynamic_launches/swarm_{timestamp}_batch_{self.launch_count}.launch'
        raw_path = rospy.get_param('~launch_save_path', default_filename)
        self.launch_count += 1
        save_path = self.resolve_path(raw_path)
        
        try:
            # 【修改核心】提取文件夹路径，如果不存在则自动创建
            save_dir = os.path.dirname(save_path)
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)
                rospy.loginfo(f"Created new directory for launch files at: {save_dir}")
                
            with open(save_path, 'w') as f:
                f.write(xml_content)
        except Exception as e:
            return UUVLoaderResponse(False, f"Failed to save launch file: {str(e)}")
            
        rospy.loginfo(f"Appending new UUV swarm batch {self.launch_count-1}...")
        proc = subprocess.Popen(["roslaunch", save_path])
        self.active_processes.append(proc)
        
        return UUVLoaderResponse(True, f"Successfully appended {req.total_uuvs} UUVs.")

if __name__ == '__main__':
    try:
        node = UUVLoaderNode()
        rospy.spin()
        for p in node.active_processes:
            p.terminate()
            p.wait()
    except rospy.ROSInterruptException:
        pass