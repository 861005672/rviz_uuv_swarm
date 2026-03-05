import os
import re

def surgical_fix():
    input_file = "/home/zx567/rvizuuv3d/src/uuv_environment/scripts/dynamic_swarm_50.rviz"
    output_file = "dynamic_swarm_50_final.rviz"
    
    if not os.path.exists(input_file):
        print(f"❌ 错误：找不到文件 {input_file}")
        return

    with open(input_file, 'r') as f:
        lines = f.readlines()

    new_lines = []
    current_uuv_id = None
    
    for line in lines:
        # 1. 替换类名（保留原有缩进）
        if "Class: rviz/Path" in line:
            new_line = line.replace("rviz/Path", "rviz/Marker")
            new_lines.append(new_line)
            continue
        
        # 2. 从 Topic 中提取 ID 并替换为 Marker Topic
        if "Topic: /uuv_" in line and "/trajectory" in line:
            match = re.search(r'uuv_(\d+)', line)
            if match:
                current_uuv_id = match.group(1)
            new_line = line.replace("Topic:", "Marker Topic:")
            new_lines.append(new_line)
            continue
            
        # 3. 替换名称，并在下方注入 Namespace 选项
        if "Name: Path" in line and current_uuv_id is not None:
            # 替换名字
            indent = line[:line.find("Name")]
            new_lines.append(f"{indent}Name: Trajectory_uuv_{current_uuv_id}\n")
            # 核心注入：在 Name 下方紧跟 Namespaces 配置，必须对齐缩进
            new_lines.append(f"{indent}Namespaces:\n")
            new_lines.append(f"{indent}  uuv_trajectory: true\n")
            current_uuv_id = None # 处理完一个，重置
            continue

        # 其他行原封不动保留
        new_lines.append(line)

    with open(output_file, 'w') as f:
        f.writelines(new_lines)
    
    print(f"✅ 修复完成！")
    print(f"📄 新文件已生成: {output_file}")
    print(f"💡 提示：如果打开还报错，请检查第 1 步生成的 C++ 节点是否已经 Catkin_make 成功。")

if __name__ == "__main__":
    surgical_fix()