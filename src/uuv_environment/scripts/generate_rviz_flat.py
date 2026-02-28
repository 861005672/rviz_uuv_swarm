#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import random

def generate_rviz_config(total_uuvs, output_file):
    # ==========================================
    # 1. 基础头部与网格显示 (Grid)
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
  - Class: rviz/Selection
    Name: Selection
  - Class: rviz/Tool Properties
    Expanded:
      - /2D Pose Estimate1
      - /2D Nav Goal1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
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
      Name: Grid
      Plane: XY
      Plane Cell Count: 100
      Reference Frame: <Fixed Frame>
      Value: true
      
    # ==========================================
    # 🏆 总文件夹：Swarm_Cluster 
    # ==========================================
    - Class: rviz/Group
      Enabled: true
      Name: Swarm_Cluster
      Displays:
"""

    # ==========================================
    # 2. 循环生成子文件夹和内部插件 (注意严格的 YAML 缩进)
    # ==========================================
    for i in range(total_uuvs):
        ns = f"uuv_{i}"
        r, g, b = random.randint(50, 255), random.randint(50, 255), random.randint(50, 255)
        
        # 8个空格缩进代表这是属于 Swarm_Cluster 内部的元素
        rviz_content += f"""        # --- 📁 {ns} 子文件夹 ---
        - Class: rviz/Group
          Enabled: true
          Name: {ns}
          Displays:
            - Alpha: 1
              Class: rviz/RobotModel
              Collision Enabled: false
              Enabled: true
              Links:
                All Links Enabled: true
                Expand Joint Details: false
                Expand Link Details: false
                Expand Tree: false
                Link Tree Style: Links in Alphabetic Order
              Name: RobotModel_{ns}
              Robot Description: /{ns}/robot_description
              TF Prefix: ""
              Value: true

            - Angle Tolerance: 0.1
              Class: rviz/Odometry
              Covariance:
                Orientation:
                  Alpha: 0.5
                  Color: 255; 255; 127
                  Color Style: Unique
                  Frame: Local
                  Offset: 1
                  Scale: 1
                  Value: true
                Position:
                  Alpha: 0.3
                  Color: 204; 51; 204
                  Scale: 1
                  Value: true
                Value: true
              Enabled: true
              Keep: 500
              Name: Odometry_{ns}
              Position Tolerance: 0.1
              Shape:
                Alpha: 1
                Axes Length: 1
                Axes Radius: 0.1
                Color: {r}; {g}; {b}
                Head Length: 0.3
                Head Radius: 0.1
                Shaft Length: 1
                Shaft Radius: 0.05
                Value: Arrow
              Topic: /{ns}/odom
              Value: true
"""

    # ==========================================
    # 3. 全局选项与相机视角 (加入倒立反转修复)
    # ==========================================
    rviz_content += """
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Default Light: true
    Fixed Frame: ned
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz/Interact
      Hide Inactive Objects: true
    - Class: rviz/MoveCamera
    - Class: rviz/Select
    - Class: rviz/FocusCamera
  Value: true
  Views:
    Current:
      Class: rviz/ThirdPersonFollower
      Distance: 15.0
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
      Invert Z Axis: true   # <--- 【修复1】反转Z轴，解决NED环境全局倒立问题
      Name: Current View
      Pitch: 0.78
      Target Frame: ned
      Yaw: 0.0
    Saved:
"""

    # ==========================================
    # 4. 为每个 UUV 生成防倒立的跟随视角
    # ==========================================
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
        Invert Z Axis: true   # <--- 【修复2】每个跟随视角也都反转 Z 轴！
        Name: Track_{ns}
        Near Clip Distance: 0.01
        Pitch: 0.5
        Target Frame: {ns}/base_link
        Yaw: 3.14
"""

    # ==========================================
    # 5. 结尾窗体配置
    # ==========================================
    rviz_content += """Window Geometry:
  Displays:
    collapsed: false
  Height: 800
  Hide Left Dock: false
  Hide Right Dock: false
  Selection:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 1200
  X: 0
  Y: 0
"""

    # 写入文件
    with open(output_file, 'w') as f:
        f.write(rviz_content)
    print(f"[SUCCESS] 成功生成带『文件夹折叠结构』的 RViz 配置文件！共 {total_uuvs} 个 UUV。")
    print(f"[PATH] 保存路径: {os.path.abspath(output_file)}")

if __name__ == '__main__':
    base_dir = os.path.dirname(os.path.abspath(__file__))
    rviz_dir = os.path.join(base_dir, '..', 'rviz')
    if not os.path.exists(rviz_dir):
        os.makedirs(rviz_dir)
        
    output_path = os.path.join(rviz_dir, 'dynamic_swarm_50.rviz')
    generate_rviz_config(total_uuvs=50, output_file=output_path)