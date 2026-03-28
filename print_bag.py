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
    [全局中位数归位算法]
    利用中位数(Median)代表曲线的“大部分”分布区间。
    将所有曲线强制归位到 [0, 2pi) 范围内，并实现相互对齐。
    """
    valid_series = [item for item in all_series_data if len(item['val']) > 10]
    if not valid_series: 
        return

    # 1. 找基准：取数据点最多的那条线
    anchor = max(valid_series, key=lambda x: len(x['val']))
    anchor_median = np.median(anchor['val'])
    
    # 2. 算目标：计算基准线的主体需要平移几个 2pi 才能落入 [0, 2pi) 之间
    k_anchor = -int(np.floor(anchor_median / (2 * np.pi)))
    target_median = anchor_median + k_anchor * 2 * np.pi
    
    # 3. 强行拉回：遍历所有曲线，把它们的中位数全都强行吸附到 target_median 附近
    for item in all_series_data:
        if len(item['val']) == 0: 
            continue
            
        curr_median = np.median(item['val'])
        
        # 计算当前曲线中位数与目标中位数的差值
        diff = target_median - curr_median
        
        # 四舍五入，算出相差了几个完整的 2pi
        k = int(np.round(diff / (2 * np.pi)))
        
        if k != 0:
            item['val'] += k * 2 * np.pi
            print(f"  [\033[92m自动归位\033[0m] {item['topic']} 曲线整体平移了 {k} 个 2pi，已归位！")

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
    parser.add_argument('-o', '--output', nargs='?', const='AUTO', default=None, 
                        help='指定保存的图片文件名(例如: result.png)。如果只输入 -o 不加文件名，则自动生成名字并保存。')
    # === [新增] 图表尺寸控制参数 ===
    parser.add_argument('--width', type=float, default=12.0, help='图表的整体宽度(英寸), 默认: 12.0。论文双栏推荐设为 6.0 或 7.0')
    parser.add_argument('--height', type=float, default=5.0, help='每个子图的高度(英寸), 默认: 5.0。')

    parser.add_argument('-k', '--k_2pi', type=int, default=0, help='将所有角度曲线整体平移 k 个 2pi (例如输入 -1 将 6.28 拉回 0)')

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
    fig, axes = plt.subplots(num_fields, 1, figsize=(args.width, args.height * num_fields), sharex=True)
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
        if is_angle and len(field_series_list) > 0:
            align_angles_to_consensus(field_series_list)

        # [新增] 3. 手动 2pi 平移
        if is_angle and args.k_2pi != 0:
            for item in field_series_list:
                item['val'] += args.k_2pi * 2 * np.pi
                print(f"  [\033[93m手动调整\033[0m] {item['topic']} 曲线已平移 {args.k_2pi} 个 2pi。")

        # 4. 平滑 (Smooth)
        for item in field_series_list:
            if args.smooth > 2 and len(item['val']) > args.smooth:
                item['val'] = smooth_data(item['val'], args.smooth)

        # --- 步骤 C: 绘图 (分支逻辑) ---
        
        if args.average:
            # === 模式 1: 平均模式 ===
            print(f"  [{field}] Calculating average across {len(field_series_list)} agents...")
            c_t, c_mean, c_std = calculate_average_curve(field_series_list)
            
            if c_t is not None:
                # 绘制平均线 (修改了更加清晰的 label)
                ax.plot(c_t, c_mean, color='blue', linewidth=2.5, label='Mean value')
                # 绘制标准差阴影 (修改了更加清晰的 label)
                ax.fill_between(c_t, c_mean - c_std, c_mean + c_std, color='blue', alpha=0.2, label='±1 Std Dev')
                
                final_val = c_mean[-1]
                # 计算y轴偏移量（基于当前轴的y范围，取5%作为偏移，避免硬编码）
                y_range = ax.get_ylim()
                y_offset = (y_range[1] - y_range[0]) * 0.02  # 向下偏移5%的y轴范围（可调整比例）
                # 文字放在曲线末端下方，垂直对齐顶部（保证文字整体在偏移后的位置下方）
                ax.text(c_t[-1], final_val - y_offset, f" {final_val:.2f}", 
                        verticalalignment='top',  # 文字顶部对齐偏移后的y坐标（避免文字再往下掉）
                        horizontalalignment='right')  # 文字右对齐x坐标（贴合曲线末端）
                
                # === [修改：放弃不靠谱的 'best'，使用基于终点位置的智能规避算法] ===
                # 获取当前图表的 Y 轴上下边界
                y_min, y_max = ax.get_ylim()
                # 计算 Y 轴的中线
                y_mid = (y_min + y_max) / 2.0
                
                # 核心逻辑：判断曲线最终收敛在图表的上半区还是下半区
                if final_val > y_mid:
                    # 如果曲线最终跑到上方去了，那我们就把图例稳稳地放在右下角
                    smart_loc = 'lower right'
                else:
                    # 如果曲线最终停在下方，那图例就放在右上角
                    smart_loc = 'upper right'

                ax.legend(
                    loc=smart_loc,       # 使用我们智能计算出的绝对安全角落
                    fontsize=14,        
                    framealpha=0.9,      # 背景半透明，增加一点呼吸感
                    edgecolor='none',    # 隐藏外边框线，符合顶级期刊极简审美
                    borderaxespad=1.0    # 离坐标轴边缘稍微留出 1.0 的边距，不要贴太死
                )
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
    
    # === [新增：强制锁定 X 轴范围，保证多图对比绝对公平] ===
    if args.duringtime > 0:
        axes[-1].set_xlim(args.begintime, args.begintime + args.duringtime)

    fig.suptitle(f"{args.title} {'' if args.average else ''}", fontsize=14)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])


    # 自动保存图片
    if args.output:
        if args.output == 'AUTO':
            # 模式 A: 用户只输入了 -o，保持原来的自动生成名字逻辑
            script_dir = os.path.dirname(os.path.abspath(__file__))
            safe_title = args.title.replace(' ', '_').replace('/', '_')
            mode_suffix = "_average" if args.average else "_raw"
            save_filename = f"{safe_title}{mode_suffix}.png"
            save_path = os.path.join(script_dir, save_filename)
        else:
            # 模式 B: 用户指定了具体的文件名或路径 (例如: -o my_plot.png)
            save_path = args.output
            
        # 保存高分辨率图片 (dpi=300)
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"\n[\033[92mSuccess\033[0m] 图表已成功保存至: {save_path}")


    plt.show()

if __name__ == "__main__":
    main()
print_bag.py