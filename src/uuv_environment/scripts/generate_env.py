import random
import math
import yaml
import os

def generate_obstacle_env(filename, center, R, size, dense):
    """
    生成复杂障碍物环境
    :param filename: 输出的 YAML 文件名
    :param center: 生成区域的中心点 [x, y, z] (NED坐标系)
    :param R: 在半径为 R 的球形区域内生成障碍物
    :param size: 障碍物的平均基准尺寸
    :param dense: 密集程度（直接代表生成的障碍物总数量）
    """
    obstacles = []
    types = ["sphere", "cylinder", "box"]
    
    for i in range(dense):
        # ==========================================
        # 1. 在半径为 R 的球体内生成均匀分布的随机坐标
        # ==========================================
        # 使用球坐标系随机生成法，保证在球体积内均匀分布
        u = random.random()
        v = random.random()
        w = random.random()
        
        theta = 2.0 * math.pi * u
        phi = math.acos(2.0 * v - 1.0)
        r = R * (w ** (1.0 / 3.0)) # 加上开立方根以保证体积均匀
        
        pos_x = center[0] + r * math.sin(phi) * math.cos(theta)
        pos_y = center[1] + r * math.sin(phi) * math.sin(theta)
        pos_z = center[2] + r * math.cos(phi)
        
        # ==========================================
        # 2. 随机选择障碍物类型与颜色
        # ==========================================
        obs_type = random.choice(types)
        
        # 随机生成比较好看的颜色 (Alpha 统一设为 0.9，带点透明质感)
        color = [
            round(random.uniform(0.2, 0.8), 2),
            round(random.uniform(0.2, 0.8), 2),
            round(random.uniform(0.2, 0.8), 2),
            0.9 
        ]
        
        # ==========================================
        # 3. 严格生成符合物理逻辑的 Scale 尺寸
        # ==========================================
        # 引入一定的尺寸随机性 (在基准 size 的 50% 到 150% 之间波动)
        s = size * random.uniform(0.5, 1.5)
        
        if obs_type == "sphere":
            # 严格的正球体：x, y, z 必须完全相同
            scale = [s, s, s]
        
        elif obs_type == "cylinder":
            # 严格的正圆柱：底面的 x 和 y 必须完全相同，高度 z 可以不同
            height = s * random.uniform(1.0, 3.0) # 圆柱通常比较长
            scale = [s, s, height]
        
        elif obs_type == "box":
            # 长方体 (OBB/SAT算法支持任何比例)：长宽高可以完全随机
            scale = [
                s * random.uniform(0.5, 2.0),
                s * random.uniform(0.5, 2.0),
                s * random.uniform(0.5, 2.0)
            ]
            
        # ==========================================
        # 4. 构建字典并加入列表
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
    # 5. 写入 YAML 文件
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

if __name__ == "__main__":
    # ================= 参数配置区 =================
    # 输出文件路径 (你可以将其直接指向你的 env 文件夹)
    output_file = "src/uuv_environment/env/test1.yaml"
    
    # 中心坐标 [x, y, z] (假设你在正前方 150 米处生成雷区)
    center_pos = [300.0, 0.0, 0.0]
    
    # 在多大半径的球体内生成？
    radius = 200.0
    
    # 障碍物的平均大小 (直径/边长)
    base_size = 35.0
    
    # 障碍物数量 (密集度)
    density = 100
    # ==============================================
    
    generate_obstacle_env(output_file, center_pos, radius, base_size, density)