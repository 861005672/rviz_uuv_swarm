#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rosbag
import matplotlib.pyplot as plt
import sys
import argparse
import re
import numpy as np
import os

# 尝试导入 scipy
try:
    from scipy.signal import savgol_filter
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    print("[Warning] 未检测到 scipy 库，将回退到简单的滑动平均。")

def get_nested_attribute(obj, attr_path):
    attributes = attr_path.split('.')
    for attr in attributes:
        if hasattr(obj, attr):
            obj = getattr(obj, attr)
        else:
            return None
    return obj

def smooth_data(y_values, window_size):
    """ Savitzky-Golay 平滑 """
    if window_size <= 2: return y_values
    if window_size % 2 == 0: window_size += 1
    if len(y_values) <= window_size: return y_values

    if SCIPY_AVAILABLE:
        try:
            return savgol_filter(y_values, window_size, 3)
        except:
            return y_values
    else:
        window = np.ones(window_size) / window_size
        return np.convolve(y_values, window, mode='same')

def align_angles_to_consensus(all_series_data):
    """
    [修改版] 锚点对齐算法
    """
    if not all_series_data:
        return

    # 1. 寻找一个有效的“锚点” (Anchor)
    anchor_ref = None
    for item in all_series_data:
        if len(item['val']) > 0:
            anchor_ref = item['val'][-1] # 取末端值作为基准
            break
            
    if anchor_ref is None:
        return

    # 2. 强制所有其他曲线向锚点对齐
    for item in all_series_data:
        if len(item['val']) == 0:
            continue

        current_final = item['val'][-1]
        diff = current_final - anchor_ref
        k = int(np.round(diff / (2 * np.pi)))
        
        if k != 0:
            shift = k * 2 * np.pi
            item['val'] -= shift

def calculate_average_curve(field_series_list):
    """
    将所有非对齐时间序列插值到同一时间轴并求平均
    返回: (common_t, mean_val, std_val)
    """
    if not field_series_list:
        return None, None, None

    # 1. 确定公共时间范围
    t_mins = [item['t'][0] for item in field_series_list if len(item['t']) > 0]
    t_maxs = [item['t'][-1] for item in field_series_list if len(item['t']) > 0]
    
    if not t_mins: return None, None, None
    
    start_t = min(t_mins)
    end_t = max(t_maxs)
    
    # 2. 确定采样点数 (取拥有最多数据点的那个序列长度，保证分辨率)
    max_len = max([len(item['t']) for item in field_series_list])
    common_t = np.linspace(start_t, end_t, num=max_len)
    
    # 3. 插值重采样
    interpolated_values = []
    for item in field_series_list:
        if len(item['t']) > 1:
            # np.interp(x_new, x_old, y_old)
            y_interp = np.interp(common_t, item['t'], item['val'])
            interpolated_values.append(y_interp)
            
    if not interpolated_values:
        return None, None, None

    # 4. 计算平均值和标准差
    data_matrix = np.vstack(interpolated_values)
    mean_val = np.mean(data_matrix, axis=0)
    std_val = np.std(data_matrix, axis=0)
    
    return common_t, mean_val, std_val

def main():
    parser = argparse.ArgumentParser(description="ROS Bag 集群曲线分析 (自动解缠 + 共识对齐)")
    parser.add_argument('bag_file', help='rosbag 路径')
    parser.add_argument('-t', '--topic', required=True, type=str, help='(话题正则, 例如: "/(.*)/state2D")')
    parser.add_argument('-f', '--fields', required=True, nargs='+', help='字段列表 例如 u psi')
    parser.add_argument('-s', '--smooth', type=int, default=51, help='平滑窗口， 值越大，越平滑')
    parser.add_argument('-y', '--ylabel', type=str, default="none", help='y轴名称')
    parser.add_argument('-a', '--average', action='store_true', help='开启平均模式：计算所有Agent的平均值并绘制单条曲线(带方差阴影)')
    parser.add_argument('-b', '--begintime', type=float, default=0.0, help='从bag的第几秒开始读取数据 (默认: 0.0)')
    parser.add_argument('-d', '--duringtime', type=float, default=-1.0, help='读取的时间段长度(秒), -1表示读到bag末尾 (默认: -1.0)')
    parser.add_argument('--title', type=str, default="Swarm Analysis", help='标题')
    parser.add_argument('-o', '--output', action='store_true', help='开启后，将图表自动保存为图片到本 Python 脚本所在的目录')
    
    args = parser.parse_args()

    if args.ylabel == "none":
        args.ylabel = args.fields

    # --- 1. 读取数据 ---
    print(f"正在读取: {args.bag_file}, ylabel: {args.ylabel} ...")
    try:
        bag = rosbag.Bag(args.bag_file)
    except Exception as e:
        print(f"[Error] {e}")
        return

    # 话题过滤
    all_topics = bag.get_type_and_topic_info()[1].keys()
    pattern = re.compile(args.topic)
    target_topics = sorted([t for t in all_topics if pattern.match(t)])

    if not target_topics:
        print("[Error] 无匹配话题")
        return

    # 数据容器
    data = {t: {f: {'t': [], 'val': []} for f in args.fields} for t in target_topics}
    bag_start_time = None  # 用于记录bag的第一条消息的绝对时间
    print(f"提取 {len(target_topics)} 个话题的数据 (开始时间: {args.begintime}s, 时长: {'全部' if args.duringtime < 0 else str(args.duringtime) + 's'})...")
    for topic, msg, t in bag.read_messages(topics=target_topics):
        # 1. 记录包的绝对起始时间
        if bag_start_time is None: 
            bag_start_time = t.to_sec()
        # 2. 计算当前消息相对于包起始时间的相对时间(秒)
        current_time = t.to_sec() - bag_start_time
        # 3. [过滤逻辑] 如果还没到设定的开始时间，直接跳过
        if current_time < args.begintime:
            continue
        # 4. [截断逻辑] 如果设置了持续时间，且当前时间已经超出了 (开始时间 + 持续时间)，则提前结束读取！
        if args.duringtime > 0 and current_time > (args.begintime + args.duringtime):
            break  # 提前终止循环，极大提高大文件的读取速度
        # 5. 提取数据
        for field in args.fields:
            val = get_nested_attribute(msg, field)
            if val is not None:
                try:
                    # 注意：此时追加到列表里的时间是从 args.begintime 开始的，图表的X轴也会从这个时间点开始
                    data[topic][field]['t'].append(current_time)
                    data[topic][field]['val'].append(float(val))
                except: pass
    bag.close()


    # --- 2. 处理与绘图 ---
    print("正在处理与绘图...")
    num_fields = len(args.fields)
    fig, axes = plt.subplots(num_fields, 1, figsize=(12, 5 * num_fields), sharex=True)
    if num_fields == 1: axes = [axes]

    # 生成颜色表 (仅在非平均模式下用于区分Agent)
    colors = plt.cm.jet(np.linspace(0, 1, len(target_topics)))
    topic_color_map = {t: c for t, c in zip(target_topics, colors)}
    
    # 定义需要特殊处理的角度字段
    ANGLE_FIELDS = ['psi', 'theta', 'yaw', 'heading', 'roll', 'pitch']

    for i, field in enumerate(args.fields):
        ax = axes[i]
        
        # --- 步骤 A: 收集该字段下所有 Topic 的数据 ---
        field_series_list = []
        for topic in target_topics:
            raw_t = np.array(data[topic][field]['t'])
            raw_y = np.array(data[topic][field]['val'])
            if len(raw_t) > 0:
                field_series_list.append({
                    'topic': topic,
                    't': raw_t,
                    'val': raw_y
                })

        # --- 步骤 B: 数据预处理 (解缠 -> 对齐 -> 平滑) ---
        is_angle = field in ANGLE_FIELDS
        
        # 1. 解缠 (Unwrap)
        for item in field_series_list:
            if is_angle:
                item['val'] = np.unwrap(item['val'])

        # 2. 共识对齐 (Alignment)
        if is_angle and len(field_series_list) > 1:
            align_angles_to_consensus(field_series_list)

        # 3. 平滑 (Smooth)
        for item in field_series_list:
            if args.smooth > 2 and len(item['val']) > args.smooth:
                item['val'] = smooth_data(item['val'], args.smooth)

        # --- 步骤 C: 绘图 (分支逻辑) ---
        
        if args.average:
            # === 模式 1: 平均模式 ===
            print(f"  [{field}] Calculating average across {len(field_series_list)} agents...")
            c_t, c_mean, c_std = calculate_average_curve(field_series_list)
            
            if c_t is not None:
                # 绘制平均线
                ax.plot(c_t, c_mean, color='blue', linewidth=2.5, label=f'Mean {field}')
                # 绘制标准差阴影 (Mean ± Std)
                ax.fill_between(c_t, c_mean - c_std, c_mean + c_std, color='blue', alpha=0.2, label='Std Dev')
                
                final_val = c_mean[-1]
                ax.text(c_t[-1], final_val, f" {final_val:.2f}", verticalalignment='center')
            else:
                print(f"  [Warning] No valid data to average for {field}")

        else:
            # === 模式 2: 原始多曲线模式 ===
            for item in field_series_list:
                topic = item['topic']
                label = topic.split('/')[1] if len(topic.split('/')) > 1 else topic
                ax.plot(item['t'], item['val'], 
                        color=topic_color_map[topic], 
                        alpha=0.6, linewidth=1.2, label=label)

        # 设置标题和图例
        if len(args.fields) == 1:
            ax.set_title(' ')
        else:
            ax.set_title(f'{field}')
        ax.set_ylabel(args.ylabel)
        # ax.grid(True, alpha=0.3)
        # 关闭栅格
        ax.grid(False)
        # 隐藏四面边框 (Spines)
        ax.spines['top'].set_visible(False)    # 隐藏顶部边框
        ax.spines['right'].set_visible(False)  # 隐藏右侧边框
        # ax.legend(loc='upper right')

    axes[-1].set_xlabel('Time (s)')
    fig.suptitle(f"{args.title} {'(Average View)' if args.average else ''}", fontsize=14)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])


    # 自动保存图片
    if args.output:
        # 获取当前 Python 脚本所在的绝对目录路径
        script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # 将 title 中的空格或斜杠替换为下划线，防止作为文件名时报错
        safe_title = args.title.replace(' ', '_').replace('/', '_')
        mode_suffix = "_average" if args.average else "_raw"
        
        # 拼接最终的保存路径 (例如: Swarm_Analysis_average.png)
        save_filename = f"{safe_title}{mode_suffix}.png"
        save_path = os.path.join(script_dir, save_filename)
        
        # 保存高分辨率图片 (dpi=300)
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"\n[\033[92mSuccess\033[0m] 图表已成功保存至: {save_path}")



    plt.show()

if __name__ == "__main__":
    main()