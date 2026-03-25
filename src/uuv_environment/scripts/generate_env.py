import random
import math
import yaml
import os

def generate_obstacle_env(filename, center, R, size, dense, is_2d=False):
    """
    生成复杂障碍物环境
    :param filename: 输出的 YAML 文件名
    :param center: 生成区域的中心点 [x, y, z] (NED坐标系)
    :param R: 在半径为 R 的球形区域内生成障碍物 (若 is_2d=True，则为圆形平面区域)
    :param size: 障碍物的平均基准尺寸
    :param dense: 密集程度（直接代表生成的障碍物总数量）
    :param is_2d: 是否在同一平面内生成障碍物 (所有障碍物 z=0, 且自身高度统一)
    """
    obstacles = []
    
    # 针对 2D 平面实验，最好剔除球体，只保留横截面上下一致的圆柱和长方体
    types = ["cylinder", "box"] if is_2d else ["sphere", "cylinder", "box"]
    
    for i in range(dense):
        # ==========================================
        # 1. 生成坐标
        # ==========================================
        if is_2d:
            # 2D 圆形平面内均匀生成坐标
            theta = 2.0 * math.pi * random.random()
            # 使用开平方根以保证在圆面积内均匀分布，防止中心扎堆
            r = R * math.sqrt(random.random()) 
            
            pos_x = center[0] + r * math.cos(theta)
            pos_y = center[1] + r * math.sin(theta)
            pos_z = 0.0  # 强制 z 坐标为 0
        else:
            # 3D 球体内均匀生成坐标
            u = random.random()
            v = random.random()
            w = random.random()
            
            theta = 2.0 * math.pi * u
            phi = math.acos(2.0 * v - 1.0)
            r = R * (w ** (1.0 / 3.0)) # 使用开立方根以保证体积均匀分布
            
            pos_x = center[0] + r * math.sin(phi) * math.cos(theta)
            pos_y = center[1] + r * math.sin(phi) * math.sin(theta)
            pos_z = center[2] + r * math.cos(phi)
        
        # ==========================================
        # 2. 生成障碍物类型与缩放比例
        # ==========================================
        obs_type = random.choice(types)
        
        if is_2d:
            # 2D情况：x, y 方向尺寸随机，z 方向高度固定为基准尺寸 size
            scale = [
                size * random.uniform(0.5, 1.5),
                size * random.uniform(0.5, 1.5),
                size  # <--- 高度统一
            ]
        else:
            # 3D情况：全方向随机缩放
            scale = [
                size * random.uniform(0.5, 1.5),
                size * random.uniform(0.5, 1.5),
                size * random.uniform(0.5, 1.5)
            ]
        
        # 随机生成颜色 (RGBA)
        # color = [round(random.random(), 2), round(random.random(), 2), round(random.random(), 2), 1.0]
        color = [0.5, 0.5, 0.5, 1.0]
        
        # ==========================================
        # 3. 构造单个障碍物的字典
        # ==========================================
        obs_dict = {
            "name": f"obs_{obs_type}_{i}",
            "ref_coord": "NED",
            "type": obs_type,
            "position": [round(pos_x, 2), round(pos_y, 2), round(pos_z, 2)],
            "scale": [round(scale[0], 2), round(scale[1], 2), round(scale[2], 2)],
            "color": color
        }
        obstacles.append(obs_dict)

    # ==========================================
    # 4. 写入 YAML 文件
    # ==========================================
    data = {"obstacles": obstacles}
    
    # 确保输出目录存在
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    
    with open(filename, 'w', encoding='utf-8') as f:
        # 使用 PyYAML 写入，禁止默认的 {} 挤在一起的流式格式，保证高可读性
        yaml.dump(data, f, default_flow_style=None, sort_keys=False)
        
    print(f"✅ 成功生成 {dense} 个障碍物至文件: {filename}")
    print(f"   - 中心点: {center}")
    print(f"   - 散布半径: {R} 米")
    print(f"   - 障碍物基准尺寸: {size} 米")
    if is_2d:
        print("   - 模式: 2D平面障碍物 (所有障碍物 z=0 且高度统一)")
    else:
        print("   - 模式: 3D立体障碍物")

if __name__ == "__main__":
    # ================= 参数配置区 =================
    # 生成默认的 3D 随机障碍物
    # generate_obstacle_env(
    #     filename="env/random_3d.yaml", 
    #     center=[0.0, 0.0, 0.0], 
    #     R=100.0, 
    #     size=8.0, 
    #     dense=20, 
    #     is_2d=False
    # )
    
    # 生成用于平面验证的 2D 障碍物
    generate_obstacle_env(
        filename="/home/zx567/rvizuuv3d/src/uuv_environment/env/newMap.yaml", 
        center=[50.0, 50.0, 0.0], 
        R=50.0, 
        size=6.0, 
        dense=10, 
        is_2d=True  # <--- 将此处设为 True 即可生成平面障碍物
    )