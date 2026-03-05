#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
import os
import math
import random
import subprocess
from uuv_director.srv import UUVLoader as UUVLoaderSrv, UUVLoaderResponse
import datetime

class UUVLoader:
    def __init__(self):
        self.active_processes = []
        self.all_spawned_poses = []
        self.current_group_idx = 0   # 专门用于区分计算资源 (Manager)
        self.global_uuv_idx = 0      # 【核心新增】专门用于全局唯一递增的 UUV ID
        self.rospack = rospkg.RosPack()
        
        self.srv = rospy.Service('uuv_loader', UUVLoaderSrv, self.handle_loader_request)
        rospy.loginfo("UUV Loader Service is Ready. Flat namespace [uuv_X] enabled!")

    def cleanup(self):
        """提供给主节点的清理函数，用于 ROS 退出时杀掉所有衍生的子进程"""
        for p in self.active_processes:
            p.terminate()
            p.wait()

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

    def generate_xml_launch(self, center, poses, group_size, dyn_type, act_type, ctrl_type, gui_type, pla_type, dec_type, control_level, log_dir):
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
                xml_lines.append(f'    <param name="robot_description" command="$(find xacro)/xacro \'$(find uuv_description)/urdf/lauv_model.xacro\' namespace:={uuv_ns} dynamics_type:={dyn_type} actuator_type:={act_type} controller_type:={ctrl_type} guidance_type:={gui_type} planner_type:={pla_type} decision_type:={dec_type}"/>')
                xml_lines.append(f'    <param name="init_x" value="{x:.3f}"/>')
                xml_lines.append(f'    <param name="init_y" value="{y:.3f}"/>')
                xml_lines.append(f'    <param name="init_z" value="{z:.3f}"/>')
                xml_lines.append(f'    <param name="init_yaw" value="{yaw:.3f}"/>')
                xml_lines.append(f'    <param name="start_n" value="{center[0]:.3f}"/>')
                xml_lines.append(f'    <param name="start_e" value="{center[1]:.3f}"/>')
                xml_lines.append(f'    <param name="start_d" value="{center[2]:.3f}"/>')
                xml_lines.append(f'    <param name="control_level" value="{control_level}"/>')
                xml_lines.append(f'    <param name="log_dir" value="{log_dir}"/>')
                xml_lines.append('    <param name="visual_rate" value="20.0"/>')
                # 控制节点直接跨命名空间加载到对应 group 的 Manager 中
                xml_lines.append(f'    <node pkg="nodelet" type="nodelet" name="uuv_control" args="load uuv_control/ControlNodelet /{group_name}/{group_name}_manager" />')
                
                xml_lines.append('    <node pkg="robot_state_publisher" type="robot_state_publisher" name="rsp" />')
                xml_lines.append('  </group>')
                uuv_idx += 1
                
        # 更新总发射数量
        self.global_uuv_idx += total_uuvs
        
        xml_lines.append('</launch>')
        return "\n".join(xml_lines)

    def handle_loader_request(self, req):

        
        total_uuvs = req.total_uuvs if req.total_uuvs!=0 else 1
        group_size = req.group_size if req.group_size!=0 else 10
        init_type = req.init_type if req.init_type!='' else 'random'
        center = req.center if len(req.center)==3 else [0,0,0]
        radius = req.radius if req.radius!=0 else 5
        min_dist = req.min_dist if req.min_dist!=0 else 2
        dynamics_type = req.dynamics_type if req.dynamics_type!='' else 'fossen'
        actuator_type = req.actuator_type if req.actuator_type!='' else 'lauv'
        controller_type = req.controller_type if req.controller_type!='' else 'pid'
        guidance_type = req.guidance_type if req.guidance_type!='' else 'spacial_flocking'
        planner_type = req.planner_type if req.planner_type!='' else 'simple'
        decision_type = req.decision_type if req.decision_type!='' else 'vanguard'
        control_level = req.control_level if req.control_level!='' else 'decision'


        if init_type == "deterministic":
            return UUVLoaderResponse(False, "Deterministic mode is not implemented yet.")

        rospy.loginfo(f"Generating {total_uuvs} appended UUVs in random mode...")
        poses = self.generate_random_poses(total_uuvs, center, radius, min_dist)
        
        if poses is None:
            return UUVLoaderResponse(False, "Failed to generate collision-free poses. Space is too crowded.")
            

        self.all_spawned_poses.extend(poses) 
        # 1. 生成时间戳与批次文件(夹)名称
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        batch_name = f"swarm_{timestamp}_count_{total_uuvs}"

        # 2. 解析 Launch 文件的绝对路径与所属目录 (dynamic_launches)
        default_launch_rel_path = f'./adynamic_launches/{batch_name}.launch'
        raw_launch_path = rospy.get_param('~launch_save_path', default_launch_rel_path)
        
        launch_file_path = self.resolve_path(raw_launch_path)
        dynamic_launch_dir = os.path.dirname(launch_file_path)  

        # 3. 推导工作空间根目录，并生成同级的专属日志目录 (uuv_logs)
        ws_root = os.path.dirname(dynamic_launch_dir)
        log_dir = os.path.join(ws_root, "auuv_logs", batch_name)

        # 4. 确保日志目录存在
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
            rospy.loginfo(f"[LogManager] Created exclusive log directory: {log_dir}")
        
        # 5. 生成 XML 内容 (注入专属日志路径供 C++ 读取)
        xml_content = self.generate_xml_launch(center, poses, group_size, dynamics_type, actuator_type, controller_type, guidance_type, planner_type, decision_type, control_level, log_dir)
        
        # 6. 确保 Launch 目录存在并写入文件
        try:
            if not os.path.exists(dynamic_launch_dir):
                os.makedirs(dynamic_launch_dir)
                rospy.loginfo(f"[LaunchManager] Created launch directory: {dynamic_launch_dir}")
                
            with open(launch_file_path, 'w') as f:
                f.write(xml_content)
        except Exception as e:
            return UUVLoaderResponse(False, f"Failed to save launch file: {str(e)}")
            
        # 7. 打印极其清晰的路径指引并启动进程
        rospy.loginfo(f"Appending new UUV swarm with {total_uuvs} UUVs...")
        rospy.loginfo(f"--> [LAUNCH] configuration saved in: {dynamic_launch_dir}")
        rospy.loginfo(f"--> [LOGS] will be async recorded in: {log_dir}")
        
        proc = subprocess.Popen(["roslaunch", launch_file_path])
        self.active_processes.append(proc)
        
        return UUVLoaderResponse(True, f"Success ({total_uuvs} UUVs). Launch dir: {dynamic_launch_dir} | Log dir: {log_dir}")



