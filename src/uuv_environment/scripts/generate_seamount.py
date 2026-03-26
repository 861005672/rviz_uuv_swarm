from ursina import *
import yaml
import math
import os
import time

app = Ursina(borderless=False, size=(1280, 800), title="UUV Terrain Sculptor (Terrain + Cylinder Dual Mode)")
window.color = color.rgb(15, 25, 35) 
window.fps_counter.enabled = False   
window.exit_button.visible = False   

# ================= 核心全局数据 =================
current_map_size = 400.0
current_voxel_size = 20.0
current_grid_count = 20

height_matrix = [[2.0 for _ in range(100)] for _ in range(100)] 
entity_pool = [] 
terrain_cubes_count = 0  

# 🌟 独立的圆柱体系统
active_cylinders = []
current_mode = 'Terrain' # 'Terrain' 或 'Cylinder'

rebuild_pending = False
last_interaction_time = 0
last_ctrl_action_time = 0 
ui_interaction_active = False 

# ================= 1. 绝对物理坐标的 NED 轴 =================
axis_center = Entity(model='cube', color=color.white, scale=(5, 5, 5), position=(0, 0, 0))
axis_n = Entity(model='cube', color=color.red)    # 北 (NED的X轴)
axis_e = Entity(model='cube', color=color.green)  # 东 (NED的Y轴)
axis_d = Entity(model='cube', color=color.blue)   # 下 (NED的Z轴)

def update_axes_scale(map_size):
    length = map_size / 2.0
    thickness = max(2.0, map_size / 200.0) 
    axis_n.scale = (thickness, thickness, length)
    axis_n.position = (0, 0, length/2)
    axis_e.scale = (length, thickness, thickness)
    axis_e.position = (length/2, 0, 0)
    axis_d.scale = (thickness, length, thickness)
    axis_d.position = (0, -length/2, 0)


# ================= 2. 图形化界面 (GUI) =================
base_x = -0.55
ui_scale = 0.75 

# 滑动条排版保持最舒适的间距
depth_slider = Slider(min=0, max=1000, default=100, step=10, dynamic=True, text='SeaBed Depth (m)', parent=camera.ui, scale=ui_scale, position=(base_x, 0.45))
map_size_slider = Slider(min=200, max=5000, default=1000, step=100, dynamic=True, text='Map Size (m)', parent=camera.ui, scale=ui_scale, position=(base_x, 0.40))
voxel_size_slider = Slider(min=10, max=200, default=20, step=5, dynamic=True, text='Resolution (m)', parent=camera.ui, scale=ui_scale, position=(base_x, 0.35))
brush_radius_slider = Slider(min=10, max=1000, default=150, step=10, dynamic=True, text='Brush Radius (m)', parent=camera.ui, scale=ui_scale, position=(base_x, 0.30))
detail_slider = Slider(min=0.01, max=1.0, default=0.8, step=0.01, dynamic=True, text='Detail Level (0~1)', parent=camera.ui, scale=ui_scale, position=(base_x, 0.25))
center_n_slider = Slider(min=-5000, max=5000, default=0, step=10, dynamic=True, text='Center N (m)', parent=camera.ui, scale=ui_scale, position=(base_x, 0.20))
center_e_slider = Slider(min=-5000, max=5000, default=0, step=10, dynamic=True, text='Center E (m)', parent=camera.ui, scale=ui_scale, position=(base_x, 0.15))

Text(text="Obstacle Color (RGB):", parent=camera.ui, position=(base_x - 0.2, 0.08), scale=ui_scale)
color_r = Slider(min=0.0, max=1.0, default=0.5, step=0.05, dynamic=True, text='R', parent=camera.ui, scale=ui_scale, position=(base_x, 0.04))
color_g = Slider(min=0.0, max=1.0, default=0.5, step=0.05, dynamic=True, text='G', parent=camera.ui, scale=ui_scale, position=(base_x, -0.01))
color_b = Slider(min=0.0, max=1.0, default=0.5, step=0.05, dynamic=True, text='B', parent=camera.ui, scale=ui_scale, position=(base_x, -0.06))

# 🌟 模式切换按钮和保存按钮：完美并排在底部
mode_btn = Button(text='Mode: Terrain', parent=camera.ui, scale=(0.18, 0.04), color=color.dark_gray, position=(base_x - 0.08, -0.15))
export_btn = Button(text='Save YAML Map', parent=camera.ui, scale=(0.18, 0.04), color=color.azure, position=(base_x + 0.12, -0.15))

def toggle_mode():
    global current_mode
    if current_mode == 'Terrain':
        current_mode = 'Cylinder'
        mode_btn.text = 'Mode: Cylinder'
        mode_btn.color = color.orange
    else:
        current_mode = 'Terrain'
        mode_btn.text = 'Mode: Terrain'
        mode_btn.color = color.dark_gray

mode_btn.on_click = toggle_mode

# 🌟 实时数量分类显示面板 (下移避免拥挤)
cube_count_text = Text(text="Terrain Cubes: 0 | Cylinders: 0", position=(base_x - 0.2, -0.22), scale=1.3, color=color.lime)
status_text = Text(text="", position=(base_x - 0.2, -0.27), scale=1.0, color=color.yellow)

brush_cursor = Entity(model=Circle(resolution=36, mode='line'), color=color.yellow, rotation_x=90, enabled=False, double_sided=True)
# =======================================================

# ================= 3. 实时更新函数 =================
def update_ui_counts():
    cube_count_text.text = f"Terrain Cubes: {terrain_cubes_count} | Cylinders: {len(active_cylinders)}"

def apply_visual_mesh():
    global terrain_cubes_count
    vertical_step = max(0.1, 200.0 * ((1.0 - detail_slider.value) ** 3))
    
    Q = [[0.0 for _ in range(current_grid_count)] for _ in range(current_grid_count)]
    for x in range(current_grid_count):
        for z in range(current_grid_count):
            h = height_matrix[x][z]
            if h <= 2.1: 
                Q[x][z] = 2.0
            else:
                Q[x][z] = round(h / vertical_step) * vertical_step
                
    visited = [[False for _ in range(current_grid_count)] for _ in range(current_grid_count)]
    pool_idx = 0
    
    usr_color = color.rgb(color_r.value * 255, color_g.value * 255, color_b.value * 255)
    offset_n = center_n_slider.value
    offset_e = center_e_slider.value
    target_y_base = -depth_slider.value
    
    for z in range(current_grid_count):
        for x in range(current_grid_count):
            if visited[x][z]: continue
            
            target_h = Q[x][z]
            w = 1
            while x + w < current_grid_count and not visited[x+w][z] and Q[x+w][z] == target_h:
                w += 1
            d = 1
            can_expand = True
            while z + d < current_grid_count and can_expand:
                for i in range(w):
                    if visited[x+i][z+d] or Q[x+i][z+d] != target_h:
                        can_expand = False
                        break
                if can_expand:
                    d += 1
            for i in range(w):
                for j in range(d):
                    visited[x+i][z+j] = True
                    
            chunk_len_x = w * current_voxel_size
            chunk_len_z = d * current_voxel_size
            center_x = (x + w/2.0 - current_grid_count/2.0) * current_voxel_size
            center_z = (z + d/2.0 - current_grid_count/2.0) * current_voxel_size
            
            if pool_idx >= len(entity_pool):
                p = Entity(model='cube', collider='box', origin_y=-0.5, texture='white_cube')
                entity_pool.append(p)
            else:
                p = entity_pool[pool_idx]
                
            p.enabled = True
            p.x = center_x + offset_e
            p.z = center_z + offset_n
            p.y = target_y_base
            p.scale = (chunk_len_x, target_h, chunk_len_z)
            p.color = usr_color
            p.abs_x = center_x
            p.abs_z = center_z
            p.is_cylinder = False
            pool_idx += 1

    for i in range(pool_idx, len(entity_pool)):
        entity_pool[i].enabled = False
        
    terrain_cubes_count = pool_idx
    update_ui_counts()

def mark_needs_rebuild():
    global rebuild_pending, last_interaction_time
    rebuild_pending = True
    last_interaction_time = time.time()
    status_text.text = "Waiting to rebuild physics..."

map_size_slider.on_value_changed = mark_needs_rebuild
voxel_size_slider.on_value_changed = mark_needs_rebuild
detail_slider.on_value_changed = apply_visual_mesh

def update_center_visuals():
    offset_n = center_n_slider.value
    offset_e = center_e_slider.value
    for i in range(terrain_cubes_count):
        p = entity_pool[i]
        p.x = p.abs_x + offset_e
        p.z = p.abs_z + offset_n
    for cyl in active_cylinders:
        cyl.x = cyl.abs_x + offset_e
        cyl.z = cyl.abs_z + offset_n

center_n_slider.on_value_changed = update_center_visuals
center_e_slider.on_value_changed = update_center_visuals

def update_depth_visuals():
    target_y = -depth_slider.value
    for i in range(terrain_cubes_count):
        entity_pool[i].y = target_y
    for cyl in active_cylinders:
        cyl.y = target_y

depth_slider.on_value_changed = update_depth_visuals

def update_terrain_color():
    current_col = color.rgb(color_r.value * 255, color_g.value * 255, color_b.value * 255)
    for i in range(terrain_cubes_count):
        entity_pool[i].color = current_col
    for cyl in active_cylinders:
        cyl.color = current_col

color_r.on_value_changed = update_terrain_color
color_g.on_value_changed = update_terrain_color
color_b.on_value_changed = update_terrain_color

def rebuild_grid():
    global current_map_size, current_voxel_size, current_grid_count, height_matrix
    new_map_size = map_size_slider.value
    new_voxel_size = voxel_size_slider.value
    new_grid_count = int(new_map_size / new_voxel_size)
    if new_grid_count > 100:
        new_voxel_size = new_map_size / 100.0
        voxel_size_slider.value = new_voxel_size
        new_grid_count = 100
        print(f"⚠️ 触发安全锁：分辨率被重置为 {new_voxel_size:.1f}m 以防内存溢出")
    new_height_matrix = [[2.0 for _ in range(new_grid_count)] for _ in range(new_grid_count)]
    for x_idx in range(new_grid_count):
        for z_idx in range(new_grid_count):
            abs_x = (x_idx - new_grid_count/2.0 + 0.5) * new_voxel_size
            abs_z = (z_idx - new_grid_count/2.0 + 0.5) * new_voxel_size
            old_x_idx = int(round(abs_x / current_voxel_size + current_grid_count/2.0 - 0.5))
            old_z_idx = int(round(abs_z / current_voxel_size + current_grid_count/2.0 - 0.5))
            h_phys = 2.0
            if 0 <= old_x_idx < current_grid_count and 0 <= old_z_idx < current_grid_count:
                h_phys = height_matrix[old_x_idx][old_z_idx]
            new_height_matrix[x_idx][z_idx] = h_phys
    current_map_size = new_map_size
    current_voxel_size = new_voxel_size
    current_grid_count = new_grid_count
    height_matrix = new_height_matrix
    update_axes_scale(current_map_size)
    apply_visual_mesh()
    status_text.text = f"Map: {current_map_size}m | Res: {current_voxel_size:.1f}m"

rebuild_grid()

# ================= 4. 极速保存 YAML (合并地形与圆柱) =================
def export_yaml():
    filename = "custom_macro_env.yaml"
    obstacles = []
    usr_color = [float(round(color_r.value, 2)), float(round(color_g.value, 2)), float(round(color_b.value, 2)), 0.99]
    offset_n = center_n_slider.value
    offset_e = center_e_slider.value
    obs_idx = 0
    
    # 导出贪婪合并后的立方体
    for i in range(terrain_cubes_count):
        p = entity_pool[i]
        chunk_len_x, target_h, chunk_len_z = p.scale.x, p.scale.y, p.scale.z
        ned_x = p.abs_z + offset_n
        ned_y = p.abs_x + offset_e
        ned_z = depth_slider.value - (target_h / 2.0)
        obstacles.append({
            "name": f"terrain_box_{obs_idx}",
            "ref_coord": "NED",
            "type": "box",
            "position": [float(round(ned_x, 2)), float(round(ned_y, 2)), float(round(ned_z, 2))],
            "scale": [float(round(chunk_len_z, 2)), float(round(chunk_len_x, 2)), float(round(target_h, 2))],
            "color": usr_color
        })
        obs_idx += 1

    # 导出孤立圆柱体
    for cyl in active_cylinders:
        target_h = cyl.h_phys
        radius = cyl.radius
        ned_x = cyl.abs_z + offset_n
        ned_y = cyl.abs_x + offset_e
        ned_z = depth_slider.value - (target_h / 2.0)
        obstacles.append({
            "name": f"standalone_cyl_{obs_idx}",
            "ref_coord": "NED",
            "type": "cylinder", # 输出标准的 cylinder 类型
            "position": [float(round(ned_x, 2)), float(round(ned_y, 2)), float(round(ned_z, 2))],
            "scale": [float(round(radius * 2, 2)), float(round(radius * 2, 2)), float(round(target_h, 2))],
            "color": usr_color
        })
        obs_idx += 1

    with open(filename, 'w', encoding='utf-8') as f:
        yaml.dump({"obstacles": obstacles}, f, default_flow_style=None, sort_keys=False)
        
    print(f"\n===========================================")
    print(f"✅ 地图极速导出成功！路径: {os.path.abspath(filename)}")
    print(f"   [导出内容]: {terrain_cubes_count} 个山脉方块, {len(active_cylinders)} 个孤立圆柱体")
    print(f"===========================================\n")

export_btn.on_click = export_yaml

# ================= 5. 雕刻系统 (地形笔刷 & 圆柱控制) =================
def apply_terrain_brush(hit_world_x, hit_world_z, action_type):
    radius = brush_radius_slider.value 
    brush_strength = 200.0 
    center_abs_x = hit_world_x - center_e_slider.value
    center_abs_z = hit_world_z - center_n_slider.value
    
    min_x = max(0, int((center_abs_x - radius) / current_voxel_size + current_grid_count/2))
    max_x = min(current_grid_count-1, int((center_abs_x + radius) / current_voxel_size + current_grid_count/2))
    min_z = max(0, int((center_abs_z - radius) / current_voxel_size + current_grid_count/2))
    max_z = min(current_grid_count-1, int((center_abs_z + radius) / current_voxel_size + current_grid_count/2))
    
    changed = False
    for x in range(min_x, max_x + 1):
        for z in range(min_z, max_z + 1):
            cell_abs_x = (x - current_grid_count/2.0 + 0.5) * current_voxel_size
            cell_abs_z = (z - current_grid_count/2.0 + 0.5) * current_voxel_size
            dist = math.sqrt((cell_abs_x - center_abs_x)**2 + (cell_abs_z - center_abs_z)**2)
            if dist <= radius:
                old_h = height_matrix[x][z]
                new_h = old_h
                if action_type == 'raise':
                    falloff = math.cos((dist / radius) * (math.pi / 2))
                    new_h = old_h + brush_strength * falloff * time.dt
                elif action_type == 'lower':
                    falloff = math.cos((dist / radius) * (math.pi / 2))
                    new_h = old_h - brush_strength * falloff * time.dt
                elif action_type == 'add_100':
                    new_h = old_h + 100.0
                elif action_type == 'flatten':
                    new_h = 2.0
                new_h = max(2.0, new_h)
                if old_h != new_h:
                    height_matrix[x][z] = new_h
                    changed = True
    if changed:
        apply_visual_mesh()

def spawn_cylinder(hit_world_x, hit_world_z):
    """在指定位置放置一个全新的圆柱体"""
    radius = brush_radius_slider.value
    current_col = color.rgb(color_r.value * 255, color_g.value * 255, color_b.value * 255)
    
    center_abs_x = hit_world_x - center_e_slider.value
    center_abs_z = hit_world_z - center_n_slider.value
    
    # 🌟【核心修复】：使用 Ursina 的 Cylinder() 网格类，resolution 控制圆柱的圆滑程度 (边数)
    cyl = Entity(model=Cylinder(resolution=24), collider='box', origin_y=-0.5)
    cyl.is_cylinder = True
    cyl.abs_x = center_abs_x
    cyl.abs_z = center_abs_z
    cyl.radius = radius
    cyl.h_phys = 2.0 
    
    cyl.x = center_abs_x + center_e_slider.value
    cyl.z = center_abs_z + center_n_slider.value
    cyl.y = -depth_slider.value
    cyl.scale = (radius * 2, cyl.h_phys, radius * 2)
    cyl.color = current_col
    
    active_cylinders.append(cyl)
    update_ui_counts()

def modify_cylinder(cyl, action_type):
    """对已有的圆柱体执行物理拔高、降低或删除"""
    brush_strength = 200.0
    old_h = cyl.h_phys
    new_h = old_h
    
    if action_type == 'raise':
        new_h = old_h + brush_strength * time.dt
    elif action_type == 'lower':
        new_h = old_h - brush_strength * time.dt
    elif action_type == 'add_100':
        new_h = old_h + 100.0
    elif action_type == 'flatten':
        # 空格键直接摧毁这根圆柱
        active_cylinders.remove(cyl)
        destroy(cyl)
        update_ui_counts()
        return
        
    new_h = max(2.0, new_h)
    cyl.h_phys = new_h
    cyl.scale_y = new_h


# ================= 6. 全局输入与循环引擎 =================
cam = EditorCamera()
cam.position = (0, 800, -800)
cam.zoom_speed = 0 

def input(key):
    global ui_interaction_active
    
    if key == 'left mouse down':
        if mouse.hovered_entity and not hasattr(mouse.hovered_entity, 'abs_x'):
            ui_interaction_active = True
        else:
            # 【圆柱生成逻辑】：如果你在 Cylinder 模式下点到了空地（不是圆柱体自身），就生成一个新的！
            if current_mode == 'Cylinder':
                if mouse.hovered_entity and not getattr(mouse.hovered_entity, 'is_cylinder', False):
                    spawn_cylinder(mouse.world_point.x, mouse.world_point.z)
                    
    elif key == 'left mouse up':
        ui_interaction_active = False

    if key == 'scroll up':
        base_speed = max(50.0, abs(cam.y) * 0.3)
        speed = base_speed * 3 if held_keys['shift'] else base_speed
        cam.position += camera.forward * speed
    elif key == 'scroll down':
        base_speed = max(50.0, abs(cam.y) * 0.3)
        speed = base_speed * 3 if held_keys['shift'] else base_speed
        cam.position -= camera.forward * speed

def update():
    global rebuild_pending, last_ctrl_action_time
    
    current_alt = max(50.0, abs(cam.y))
    cam.pan_speed = Vec2(current_alt * 1.5, current_alt * 1.5)
    cam.move_speed = current_alt * 1.5
    
    if rebuild_pending and (time.time() - last_interaction_time > 0.3):
        rebuild_pending = False
        rebuild_grid()
        
    if ui_interaction_active or (mouse.hovered_entity and not hasattr(mouse.hovered_entity, 'abs_x')):
        brush_cursor.enabled = False
        return
        
    if mouse.world_point:
        brush_cursor.enabled = True
        brush_cursor.x = mouse.world_point.x
        brush_cursor.z = mouse.world_point.z
        brush_cursor.y = mouse.world_point.y + 1.0 
        brush_cursor.scale = brush_radius_slider.value * 2
    else:
        brush_cursor.enabled = False
        
    if mouse.left and not rebuild_pending:
        if mouse.world_point:
            is_ctrl = held_keys['control']
            is_shift = held_keys['shift']
            is_space = held_keys['space'] 
            
            action_type = 'raise'
            if is_space: action_type = 'flatten'
            elif is_ctrl: action_type = 'add_100'
            elif is_shift: action_type = 'lower'
            
            # 分流逻辑：根据当前模式调用不同的笔刷
            if current_mode == 'Terrain':
                if action_type in ['add_100', 'flatten']:
                    if time.time() - last_ctrl_action_time > 0.2:
                        apply_terrain_brush(mouse.world_point.x, mouse.world_point.z, action_type)
                        last_ctrl_action_time = time.time()
                else:
                    apply_terrain_brush(mouse.world_point.x, mouse.world_point.z, action_type)
            
            elif current_mode == 'Cylinder':
                # 如果鼠标正悬停在某根圆柱体上，就可以长按进行拉伸或压低！
                if mouse.hovered_entity and getattr(mouse.hovered_entity, 'is_cylinder', False):
                    if action_type in ['add_100', 'flatten']:
                        if time.time() - last_ctrl_action_time > 0.2:
                            modify_cylinder(mouse.hovered_entity, action_type)
                            last_ctrl_action_time = time.time()
                    else:
                        modify_cylinder(mouse.hovered_entity, action_type)

print("\n" + "="*50)
print("🚀 UUV 终极造景引擎：山脉/圆柱双模式全解锁！")
print("   - 点击左侧顶部按钮切换【Terrain(山脉)】和【Cylinder(圆柱)】模式")
print("   - [圆柱模式下]：点击地面放置一根新圆柱；按住左键/Shift+左键 指向圆柱体即可自由伸缩！")
print("   - [圆柱模式下]：指向不想要的圆柱体，按住【空格键+左键】瞬间摧毁！")
print("="*50 + "\n")

app.run()