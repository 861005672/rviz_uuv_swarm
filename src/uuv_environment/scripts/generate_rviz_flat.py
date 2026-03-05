#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import random
import sys

def generate_rviz_config(total_uuvs, output_file):
    # ==========================================
    # 1. 基础头部、Grid 与 环境 MarkerArray
    # ==========================================
    rviz_content = """Panels:
  - Class: rviz/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
      Splitter Ratio: 0.5
    Tree Height: 455
  - Class: rviz/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Preferences:
  PromptSaveOnExit: true
Toolbars:
  toolButtonStyle: 2
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 100
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz/MarkerArray
      Enabled: true
      Name: Environment Obstacles (MarkerArray)
      Marker Topic:
        Value: /env_markers
      Value: true
    - Class: rviz/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: false
        ned:
          Value: true
      Marker Scale: 30
      Name: TF
      Show Arrows: false
      Show Axes: true
      Show Names: true
      Update Interval: 0
      Value: true
"""

    # ==========================================
    # 2. 生成 Swarm_Models 大文件夹 (RobotModel)
    # ==========================================
    rviz_content += """    - Class: rviz/Group
      Enabled: true
      Name: Swarm_Models
      Displays:
"""
    for i in range(total_uuvs):
        ns = f"uuv_{i}"
        rviz_content += f"""        - Class: rviz/Group
          Enabled: true
          Name: {ns}
          Displays:
            - Alpha: 1
              Class: rviz/RobotModel
              Collision Enabled: false
              Enabled: true
              Links:
                All Links Enabled: true
                Expand Tree: false
                Expand Upward: false
              Name: RobotModel
              Robot Description: /{ns}/robot_description
              TF Prefix: "" 
              Update Interval: 0
              Value: true
              Visual Enabled: true
"""

    # ==========================================
    # 3. 生成 Swarm_Trajectories 大文件夹 (Path)
    # ==========================================
    rviz_content += """    - Class: rviz/Group
      Enabled: true
      Name: Swarm_Trajectories
      Displays:
"""
    random.seed(42) # 保证每次生成的轨迹颜色不变
    for i in range(total_uuvs):
        ns = f"uuv_{i}"
        r, g, b = random.randint(50, 255), random.randint(50, 255), random.randint(50, 255)
        # 【核心修复】：Topic 必须是一个包含 Value 的字典！
        rviz_content += f"""        - Class: rviz/Group
          Enabled: true
          Name: {ns}
          Displays:
            - Alpha: 1
              Class: rviz/Path
              Color: {r}; {g}; {b}
              Enabled: true
              Line Style: Lines
              Line Width: 0.05
              Name: Path
              Offset:
                X: 0
                Y: 0
                Z: 0
              Pose Color: {r}; {g}; {b}
              Pose Style: None
              Topic:
                Value: /{ns}/trajectory
              Value: true
"""

    # ==========================================
    # 4. 生成 Swarm_Sonars 大文件夹 (LaserScan)
    # ==========================================
    rviz_content += """    - Class: rviz/Group
      Enabled: false
      Name: Swarm_Sonars
      Displays:
"""
    for i in range(total_uuvs):
        ns = f"uuv_{i}"
        rviz_content += f"""        - Class: rviz/Group
          Enabled: true
          Name: {ns}
          Displays:
            - Alpha: 1
              Class: rviz/LaserScan
              Color: 255; 255; 255
              Color Transformer: FlatColor
              Enabled: true
              Name: SonarScan
              Position Transformer: XYZ
              Selectable: true
              Size (Pixels): 3
              Size (m): 0.1
              Style: Points
              Topic:
                Value: /{ns}/sonar_scan
              Use max range: false
              Value: true
"""
        
# ==========================================
    # 4.5 生成 Swarm_Guidance_Forces 大文件夹 (MarkerArray)
    # ==========================================
    # 注意：这里默认将外层 Group 的 Enabled 设为 false，
    # 防止刚打开 RViz 时 50个UUV x 5个Marker = 250个元素同时渲染导致卡顿。
    # 需要看的时候在 RViz 左侧勾选对应 UUV 的钩即可。
    rviz_content += """    - Class: rviz/Group
      Enabled: false
      Name: Swarm_Guidance_Forces
      Displays:
"""
    for i in range(total_uuvs):
        ns = f"uuv_{i}"
        rviz_content += f"""        - Class: rviz/Group
          Enabled: true
          Name: {ns}
          Displays:
            - Alpha: 1
              Class: rviz/MarkerArray
              Enabled: true
              Name: Debug_Markers
              Marker Topic:
                Value: /{ns}/guidance_debug_markers
              Value: true
"""

    # ==========================================
    # 5. Global Options 与 视角设置
    # ==========================================
    rviz_content += """  Enabled: true
  Global Options:
    Background Color: 32; 74; 135
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz/Interact
      Hide Inactive Objects: true
    - Class: rviz/MoveCamera
    - Class: rviz/Select
    - Class: rviz/FocusCamera
    - Class: rviz/Measure
    - Class: rviz/SetInitialPose
      Topic:
        Value: /initialpose
    - Class: rviz/SetGoal
      Topic:
        Value: /move_base_simple/goal
  Views:
    Current:
      Class: rviz/ThirdPersonFollower
      Distance: 20
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Field of View: 0.785
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 0.5
      Target Frame: uuv_0/base_link
      Yaw: 0.785
    Saved:
      - Class: rviz/TopDownOrtho
        Angle: 0
        Name: 1_TopDown_View
        Near Clip Distance: 0.01
        Scale: 50
        Target Frame: ned
        X: 0
        Y: 0
      - Class: rviz/Orbit
        Distance: 40
        Focal Point:
          X: 0
          Y: 0
          Z: 0
        Name: 2_Free_Orbit_NED
        Pitch: 0.5
        Target Frame: ned
        Yaw: 0.785
    
"""
    # 附加上每个UUV的固定追踪视角
    for i in range(total_uuvs):
        ns = f"uuv_{i}"
        rviz_content += f"""      - Class: rviz/ThirdPersonFollower
        Distance: 8.0
        Enable Stereo Rendering:
          Stereo Eye Separation: 0.06
          Stereo Focal Distance: 1
          Swap Stereo Eyes: false
          Value: false
        Field of View: 0.785
        Focal Point:
          X: 0
          Y: 0
          Z: 0
        Invert Z Axis: false
        Name: Track_{ns}
        Near Clip Distance: 0.01
        Pitch: 0.5
        Target Frame: {ns}/base_link
        Yaw: 3.14
"""

    # 结尾窗体配置
    rviz_content += """Window Geometry:
  Displays:
    collapsed: false
  Height: 800
  Hide Left Dock: false
  Hide Right Dock: false
  Views:
    collapsed: false
  Width: 1200
  X: 0
  Y: 0
"""

    with open(output_file, 'w') as f:
        f.write(rviz_content)
    print(f"[SUCCESS] 成功生成带『三级模块化分类目录』的 RViz 配置文件！共包含 {total_uuvs} 个 UUV。")
    print(f"[PATH] 文件保存路径: {output_file}")

if __name__ == '__main__':
    if len(sys.argv) > 1:
        total = int(sys.argv[1])
        output = sys.argv[2] if len(sys.argv) > 2 else f"dynamic_swarm_{total}.rviz"
    else:
        total = 50
        output = "dynamic_swarm_50.rviz"
    generate_rviz_config(total, output)