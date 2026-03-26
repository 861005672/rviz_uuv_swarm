import pygame
import sys
import math
import yaml
import os
import tkinter as tk
from tkinter import simpledialog
import numpy as np

# ================= 配置与初始化 =================
pygame.init()
WIDTH, HEIGHT = 1200, 800
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("UUV 2D Obstacle Map Editor (NED Coordinate System)")
font = pygame.font.SysFont("consolas", 16)
large_font = pygame.font.SysFont("consolas", 20, bold=True)

# 🌟 在这里设置你想要默认载入的 YAML 文件路径
DEFAULT_LOAD_YAML_PATH = "/home/zx567/rvizuuv3d/src/uuv_environment/env/test2还行.yaml"

# ================= 视图与坐标系控制 =================
zoom = 1.0          # 缩放比例 (像素/米)
camera_n = 0.0      # 屏幕中心对应的真实世界北向坐标 (N)
camera_e = 0.0      # 屏幕中心对应的真实世界东向坐标 (E)

def world_to_screen(n, e):
    """将物理世界 NED 坐标映射到屏幕坐标 (N朝上, E朝右)"""
    sx = int((e - camera_e) * zoom + WIDTH / 2)
    sy = int(-(n - camera_n) * zoom + HEIGHT / 2) # Y轴反转，使N朝上
    return sx, sy

def screen_to_world(sx, sy):
    """将屏幕坐标映射到物理世界 NED 坐标"""
    e = (sx - WIDTH / 2) / zoom + camera_e
    n = -(sy - HEIGHT / 2) / zoom + camera_n
    return n, e

# ================= 核心数据结构 =================
obstacles = []      # 存储障碍物字典
MODE_SELECT = 0
MODE_CYLINDER = 1
MODE_BOX = 2
current_mode = MODE_SELECT

# 交互状态
brush_size = 20.0   # 默认放置障碍物的尺寸 (半径或半边长)
dragging_obs = None
panning = False
last_mouse_pos = (0, 0)

# ================= 载入逻辑 (新增) =================
def load_yaml(filepath):
    global obstacles, camera_n, camera_e
    if not os.path.exists(filepath):
        print(f"❌ 找不到文件: {filepath}，请检查路径是否正确！")
        return
    
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            # safe_load 会自动解析 YAML 文件头部的锚点(&)和别名(*)
            data = yaml.safe_load(f)
            
        if not data or 'obstacles' not in data:
            print("❌ YAML 文件格式不符合预期，找不到 'obstacles' 列表。")
            return
            
        loaded_obs = []
        for obs_data in data['obstacles']:
            shape = obs_data.get('type', 'box')
            pos = obs_data.get('position', [0.0, 0.0, 0.0])
            scale = obs_data.get('scale', [20.0, 20.0, 60.0])
            
            n = float(pos[0])
            e = float(pos[1])
            # 尺寸在编辑器中是半径或半边长，YAML中保存的是直径或全长，所以除以 2
            size = float(scale[0]) / 2.0 
            
            loaded_obs.append({
                'type': shape,
                'n': n,
                'e': e,
                'size': size
            })
            
        obstacles.clear()
        obstacles.extend(loaded_obs)
        
        # 自动将相机视角平移到载入障碍物群的几何中心
        if obstacles:
            avg_n = sum(o['n'] for o in obstacles) / len(obstacles)
            avg_e = sum(o['e'] for o in obstacles) / len(obstacles)
            camera_n = avg_n
            camera_e = avg_e
            
        print(f"\n✅ 成功载入文件: {filepath}")
        print(f"   [统计] 共载入 {len(obstacles)} 个障碍物，已自动将视角居中。")
        print(f"   (注意：载入的坐标为绝对坐标，再次保存时若不想发生位移，请将 N/E 偏移量设为 0)")
        
    except Exception as e:
        print(f"❌ 载入 YAML 文件时发生错误: {e}")

# ================= 对话框函数 =================
def ask_3d_params():
    """使用 Tkinter 弹出连续对话框请求 Z 轴、高度和 NE 偏移量参数"""
    root = tk.Tk()
    root.withdraw() # 隐藏主窗口
    root.attributes('-topmost', True) # 置顶
    
    depth = simpledialog.askfloat("3D Export", "【第一步】\n请输入所有障碍物的 Z 坐标 (NED的深度，例如: 300):", initialvalue=300.0)
    if depth is None:
        root.destroy()
        return None, None, None, None
        
    height = simpledialog.askfloat("3D Export", "【第二步】\n请输入所有障碍物的高度 (例如: 60):", initialvalue=60.0)
    if height is None:
        root.destroy()
        return None, None, None, None
        
    n_offset = simpledialog.askfloat("3D Export", "【第三步】\n请输入整体偏移的 N 坐标 (北向偏移量，例如: 0):", initialvalue=0.0)
    if n_offset is None:
        root.destroy()
        return None, None, None, None
        
    e_offset = simpledialog.askfloat("3D Export", "【第四步】\n请输入整体偏移的 E 坐标 (东向偏移量，例如: 0):", initialvalue=0.0)
    
    root.destroy()
    return depth, height, n_offset, e_offset

# ================= 主循环 =================
clock = pygame.time.Clock()
running = True

while running:
    mouse_sx, mouse_sy = pygame.mouse.get_pos()
    mouse_n, mouse_e = screen_to_world(mouse_sx, mouse_sy)
    
    # ---------------- 事件处理 ----------------
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_1: current_mode = MODE_SELECT
            elif event.key == pygame.K_2: current_mode = MODE_CYLINDER
            elif event.key == pygame.K_3: current_mode = MODE_BOX
            
            elif event.key == pygame.K_l:
                # 🌟 按 L 键触发载入功能
                load_yaml(DEFAULT_LOAD_YAML_PATH)
                
            elif event.key == pygame.K_s:
                # 触发导出并解算偏移量
                d_val, h_val, n_off, e_off = ask_3d_params()
                if d_val is not None and h_val is not None and n_off is not None and e_off is not None:
                    
                    filename = DEFAULT_LOAD_YAML_PATH
                    
                    # 采用手动格式化写入，以生成极具可读性的 YAML 宏定义结构！
                    with open(filename, 'w', encoding='utf-8') as f:
                        f.write("# ========================================================\n")
                        f.write("# UUV 离散障碍物环境配置文件\n")
                        f.write("# ========================================================\n\n")
                        
                        f.write("# === 【全局宏定义参数】 (修改这里的数值即可全局生效) ===\n")
                        f.write("definitions:\n")
                        f.write("  - &GLOBAL_COLOR  [0.5, 0.5, 0.5, 1.0]  # 障碍物统一颜色 (R, G, B, A)\n")
                        f.write(f"  - &GLOBAL_DEPTH  {round(d_val, 2)}                 # 统一深度坐标 (NED下的Z轴)\n")
                        f.write(f"  - &GLOBAL_HEIGHT {round(h_val, 2)}                 # 统一物理高度\n\n")
                        
                        f.write("# === 【障碍物列表】 ===\n")
                        f.write("obstacles:\n")
                        
                        for i, obs in enumerate(obstacles):
                            shape = obs['type']
                            # 计算附加偏移量后的实际坐标
                            final_n = round(obs['n'] + n_off, 2)
                            final_e = round(obs['e'] + e_off, 2)
                            size_x = round(obs['size'] * 2, 2)
                            size_y = round(obs['size'] * 2, 2)
                            
                            f.write(f"  - name: obs_{shape}_{i}\n")
                            f.write(f"    ref_coord: NED\n")
                            f.write(f"    type: {shape}\n")
                            f.write(f"    position: [{final_n}, {final_e}, *GLOBAL_DEPTH]\n")
                            f.write(f"    scale: [{size_x}, {size_y}, *GLOBAL_HEIGHT]\n")
                            f.write(f"    color: *GLOBAL_COLOR\n\n")
                            
                    print(f"\n✅ 宏定义版本 YAML 导出成功！保存为: {filename}")
                    print(f"   [宏定义] 深度 Z = {d_val}, 统一高度 = {h_val}")
                    print(f"   [偏移中心] N 偏移 = {n_off}, E 偏移 = {e_off}")
                    print(f"   [统计数据] 共导出了 {len(obstacles)} 个带宏引用的障碍物。")
                    
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1: 
                if current_mode == MODE_SELECT:
                    clicked_obs = None
                    for obs in reversed(obstacles): 
                        dist = math.sqrt((obs['n'] - mouse_n)**2 + (obs['e'] - mouse_e)**2)
                        if dist <= obs['size']:
                            clicked_obs = obs
                            break
                    if clicked_obs:
                        dragging_obs = clicked_obs
                elif current_mode == MODE_CYLINDER:
                    obstacles.append({'type': 'cylinder', 'n': mouse_n, 'e': mouse_e, 'size': brush_size})
                elif current_mode == MODE_BOX:
                    obstacles.append({'type': 'box', 'n': mouse_n, 'e': mouse_e, 'size': brush_size})
                    
            elif event.button == 2: 
                panning = True
                last_mouse_pos = (mouse_sx, mouse_sy)
                
            elif event.button == 3: 
                for obs in reversed(obstacles):
                    dist = math.sqrt((obs['n'] - mouse_n)**2 + (obs['e'] - mouse_e)**2)
                    if dist <= obs['size']:
                        obstacles.remove(obs)
                        break
                        
            elif event.button == 4: 
                if current_mode == MODE_SELECT:
                    hovered = False
                    for obs in reversed(obstacles):
                        if math.sqrt((obs['n'] - mouse_n)**2 + (obs['e'] - mouse_e)**2) <= obs['size']:
                            obs['size'] += 2.0 / zoom
                            hovered = True
                            break
                    if not hovered:
                        zoom_factor = 1.1
                        camera_e = mouse_e - (mouse_sx - WIDTH / 2) / (zoom * zoom_factor)
                        camera_n = mouse_n + (mouse_sy - HEIGHT / 2) / (zoom * zoom_factor)
                        zoom *= zoom_factor
                else:
                    brush_size += 2.0 / zoom
                    
            elif event.button == 5: 
                if current_mode == MODE_SELECT:
                    hovered = False
                    for obs in reversed(obstacles):
                        if math.sqrt((obs['n'] - mouse_n)**2 + (obs['e'] - mouse_e)**2) <= obs['size']:
                            obs['size'] = max(1.0, obs['size'] - 2.0 / zoom)
                            hovered = True
                            break
                    if not hovered:
                        zoom_factor = 0.9
                        camera_e = mouse_e - (mouse_sx - WIDTH / 2) / (zoom * zoom_factor)
                        camera_n = mouse_n + (mouse_sy - HEIGHT / 2) / (zoom * zoom_factor)
                        zoom *= zoom_factor
                else:
                    brush_size = max(1.0, brush_size - 2.0 / zoom)
                    
        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                dragging_obs = None
            elif event.button == 2:
                panning = False
                
        elif event.type == pygame.MOUSEMOTION:
            if panning:
                dx = mouse_sx - last_mouse_pos[0]
                dy = mouse_sy - last_mouse_pos[1]
                camera_e -= dx / zoom
                camera_n += dy / zoom
                last_mouse_pos = (mouse_sx, mouse_sy)
            if dragging_obs:
                dragging_obs['n'] = mouse_n
                dragging_obs['e'] = mouse_e

    # ---------------- 渲染 ----------------
    screen.fill((20, 30, 40)) 
    
    # 1. 动态自适应网格计算
    base_grid = 10.0
    pixel_per_grid = base_grid * zoom
    while pixel_per_grid < 50:
        base_grid *= 5.0 if str(base_grid)[0] == '1' else 2.0
        pixel_per_grid = base_grid * zoom
    while pixel_per_grid > 250:
        base_grid /= 2.0 if str(base_grid)[0] == '1' else 5.0
        pixel_per_grid = base_grid * zoom
        
    grid_step = base_grid

    start_e = math.floor((camera_e - WIDTH/2/zoom) / grid_step) * grid_step
    end_e = math.ceil((camera_e + WIDTH/2/zoom) / grid_step) * grid_step
    start_n = math.floor((camera_n - HEIGHT/2/zoom) / grid_step) * grid_step
    end_n = math.ceil((camera_n + HEIGHT/2/zoom) / grid_step) * grid_step

    e = start_e
    while e <= end_e:
        sx, _ = world_to_screen(0, e)
        pygame.draw.line(screen, (40, 50, 60), (sx, 0), (sx, HEIGHT), 1)
        e += grid_step
        
    n = start_n
    while n <= end_n:
        _, sy = world_to_screen(n, 0)
        pygame.draw.line(screen, (40, 50, 60), (0, sy), (WIDTH, sy), 1)
        n += grid_step

    # 2. 绘制 NED 坐标轴 (原点)
    cx, cy = world_to_screen(0, 0)
    pygame.draw.line(screen, (255, 50, 50), (cx, cy), (cx, cy - 100), 3) 
    pygame.draw.line(screen, (50, 255, 50), (cx, cy), (cx + 100, cy), 3) 
    screen.blit(font.render("N", True, (255, 100, 100)), (cx - 15, cy - 110))
    screen.blit(font.render("E", True, (100, 255, 100)), (cx + 110, cy - 10))

    # 3. 绘制障碍物
    for obs in obstacles:
        sx, sy = world_to_screen(obs['n'], obs['e'])
        radius_px = int(obs['size'] * zoom)
        
        color = (200, 200, 50) if obs == dragging_obs else (150, 150, 150)
        
        if obs['type'] == 'cylinder':
            pygame.draw.circle(screen, color, (sx, sy), radius_px)
            pygame.draw.circle(screen, (0,0,0), (sx, sy), radius_px, 2) 
        elif obs['type'] == 'box':
            rect = pygame.Rect(sx - radius_px, sy - radius_px, radius_px*2, radius_px*2)
            pygame.draw.rect(screen, color, rect)
            pygame.draw.rect(screen, (0,0,0), rect, 2)

    # 4. 绘制光标辅助线 / 笔刷大小
    if current_mode in [MODE_CYLINDER, MODE_BOX]:
        radius_px = int(brush_size * zoom)
        if current_mode == MODE_CYLINDER:
            pygame.draw.circle(screen, (255, 255, 0), (mouse_sx, mouse_sy), radius_px, 1)
        else:
            rect = pygame.Rect(mouse_sx - radius_px, mouse_sy - radius_px, radius_px*2, radius_px*2)
            pygame.draw.rect(screen, (255, 255, 0), rect, 1)

    # 5. UI 信息面板
    ui_texts = [
        "=== UUV Obstacle Map Editor ===",
        "[1] Mode: SELECT / MOVE",
        "[2] Mode: PLACE CYLINDER",
        "[3] Mode: PLACE BOX",
        f"Current Mode: {'SELECT' if current_mode==0 else 'CYLINDER' if current_mode==1 else 'BOX'}",
        "",
        "--- Controls ---",
        "Mouse L: Place / Drag Object",
        "Mouse R: Delete Object",
        "Mouse M (Middle): Pan Map",
        "Wheel (in Select): Resize Object / Zoom Map",
        "Wheel (in Place): Change Brush Size",
        f"Brush Size: {brush_size:.1f} m",
        f"Grid Scale: 1 cell = {grid_step} m",
        "",
        f"Mouse POS: N: {mouse_n:.1f}, E: {mouse_e:.1f}",
        f"Total Obstacles: {len(obstacles)}",
        "",
        "Press [L] to Load YAML",
        "Press [S] to Save & Offset"
    ]
    
    y_offset = 10
    for i, text in enumerate(ui_texts):
        col = (255, 255, 0) if "Current Mode" in text or "[S]" in text or "[L]" in text else (200, 200, 200)
        render_txt = large_font.render(text, True, col) if i == 0 else font.render(text, True, col)
        screen.blit(render_txt, (10, y_offset))
        y_offset += 25 if i == 0 else 20

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
sys.exit()