#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
import math
import sys
import select
import termios
import tty
from sensor_msgs.msg import Joy

class SpectatorCamera:
    def __init__(self):
        rospy.init_node('spectator_camera', anonymous=True)
        
        # ==========================================
        # 1. 从 Launch 文件读取动态配置参数
        # ==========================================
        # 键盘控制参数
        self.kb_base_linear = rospy.get_param('~kb_base_linear_speed', 15.0)    # 键盘基础平移速度
        self.kb_boost_linear = rospy.get_param('~kb_boost_linear_speed', 50.0)  # 键盘 Shift 加速平移速度
        self.kb_base_angular = rospy.get_param('~kb_base_angular_speed', 1.0)   # 键盘基础旋转角速度
        self.kb_boost_angular = rospy.get_param('~kb_boost_angular_speed', 2.5) # 键盘 Shift 加速旋转角速度
        
        # 手柄控制参数
        self.joy_max_linear = rospy.get_param('~joy_max_linear_speed', 40.0)    # 手柄摇杆推满时的最大平移速度
        self.joy_max_angular = rospy.get_param('~joy_max_angular_speed', 2.0)   # 手柄摇杆推满时的最大旋转速度
        self.joy_boost_button = rospy.get_param('~joy_boost_button', 5)         # 手柄加速按键索引（默认RB键，可根据手柄调整）
        
        # ==========================================
        # 2. 初始位姿 (俯视/侧视位置)
        # ==========================================
        self.x = 0.0
        self.y = -20.0
        self.z = 20.0  # NED系中，Z为负代表在水面上方
        self.roll = 0.0
        self.pitch = 0.6
        self.yaw = 1.59
        
        # 存储最新的手柄轴状态
        self.joy_axes = []
        self.joy_buttons = []
        
        rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # 获取终端初始设置 (用于非阻塞键盘读取)
        self.terminal_settings = termios.tcgetattr(sys.stdin)
        self.last_time = rospy.Time.now().to_sec()

    def joy_callback(self, msg):
        self.joy_axes = msg.axes
        self.joy_buttons = msg.buttons

    def get_key(self, timeout=0.02):
        """ 非阻塞读取键盘按键 """
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.terminal_settings)
        return key

    def run(self):
        msg = f"""
        🎮🎥 电影级运镜摄像机 (Cinematic Spectator) 已启动！
        -----------------------------------------------------
        【键盘控制】 (需保持此终端窗口激活)
           W / S : 前进 / 后退 (沿摄像机X轴)
           A / D : 左平移 / 右平移 (沿摄像机Y轴)
           Space / C : 上升 / 下降 (沿摄像机Z轴)
           Q / E : 左转头 / 右转头 (Yaw)
           I / J : 抬头 / 低头 (Pitch)
           🔥 按住 Shift + 上述按键可触发极速模式 (Boost)！
           按 Ctrl+C 退出
           
        【手柄控制】 (支持模拟量线性调速)
           左摇杆 (L-Stick) : 沿摄像机自身XYZ轴平移 (完全跟随视角)
           右摇杆 (R-Stick) : 抬头/低头 & 左转/右转 (旋转速度不变)
           扳机键 (LT/RT)   : 下降/上升 (兼容保留)
           🔥 按下 Boost 键 ({self.joy_boost_button}号键) + 摇杆可触发移动速度加倍！
        -----------------------------------------------------
        """
        print(msg)
        
        rate = rospy.Rate(50)  # 50Hz 保证视觉极其丝滑
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()
            dt = current_time - self.last_time
            self.last_time = current_time
            
            # ==========================================
            # 1. 解析键盘指令 (带 Shift 加速判定)
            # ==========================================
            key = self.get_key()
            kb_cmd_fwd, kb_cmd_str, kb_cmd_up, kb_cmd_yaw, kb_cmd_pitch = 0.0, 0.0, 0.0, 0.0, 0.0
            
            if key == '\x03':  # Ctrl+C 退出
                break
                
            if key:
                is_boost = key.isupper() # 如果是大写字母，说明按了 Shift 或 CapsLock
                key_lower = key.lower()
                
                # 动态分配当前速度
                v_lin = self.kb_boost_linear if is_boost else self.kb_base_linear
                v_ang = self.kb_boost_angular if is_boost else self.kb_base_angular
                
                if key_lower == 'w': kb_cmd_fwd = v_lin
                elif key_lower == 's': kb_cmd_fwd = -v_lin
                
                if key_lower == 'a': kb_cmd_str = v_lin
                elif key_lower == 'd': kb_cmd_str = -v_lin
                
                if key == ' ': kb_cmd_up = v_lin          # 空格键上升
                elif key_lower == 'c': kb_cmd_up = -v_lin # C 键下降
                
                if key_lower == 'q': kb_cmd_yaw = v_ang
                elif key_lower == 'e': kb_cmd_yaw = -v_ang
                
                if key_lower == 'i': kb_cmd_pitch = v_ang
                elif key_lower == 'j': kb_cmd_pitch = -v_ang

            # ==========================================
            # 2. 解析手柄指令 (模拟量 * 最大速度)
            # ==========================================
            joy_cmd_fwd, joy_cmd_str, joy_cmd_up, joy_cmd_yaw, joy_cmd_pitch = 0.0, 0.0, 0.0, 0.0, 0.0
            
            # 手柄加速按键判定 (仅影响移动速度，旋转速度不变)
            joy_boost = 0.0
            if len(self.joy_buttons) > self.joy_boost_button:
                joy_boost = self.joy_buttons[self.joy_boost_button]  # 按键按下为1，未按为0
            joy_linear_scale = 5.0 if joy_boost else 1.0  # 按下加速键则移动速度加倍
            joy_anguler_scale = 3.0 if joy_boost else 1.0  # 按下加速键则移动速度加倍
            
            if len(self.joy_axes) >= 5:
                # 左摇杆：沿摄像机自身坐标系平移 (乘加速倍率，旋转速度不乘)
                joy_cmd_str = -self.joy_axes[0] * self.joy_max_linear * joy_linear_scale  # 左摇杆 X (摄像机Y轴)
                joy_cmd_fwd = self.joy_axes[1] * self.joy_max_linear * joy_linear_scale  # 左摇杆 Y (摄像机X轴)
                joy_cmd_yaw = self.joy_axes[3] * self.joy_max_angular * joy_anguler_scale   # 右摇杆 X (转头，速度不变)
                joy_cmd_pitch = self.joy_axes[4] * self.joy_max_angular * joy_anguler_scale # 右摇杆 Y (抬头低头，速度不变)
                
                # 扳机键升降 (兼容保留，乘加速倍率)
                if len(self.joy_axes) >= 6:
                    rt_val = (1.0 - self.joy_axes[2]) / 2.0 # 转换为 0~1 的按压深度
                    lt_val = (1.0 - self.joy_axes[5]) / 2.0
                    joy_cmd_up = (rt_val - lt_val) * self.joy_max_linear * joy_linear_scale

            # ==========================================
            # 3. 指令融合与运动学积分 (核心修改：基于摄像机自身坐标系)
            # ==========================================
            # 任一设备输入均可生效，实现无缝热切换
            cmd_fwd = joy_cmd_fwd + kb_cmd_fwd
            cmd_str = joy_cmd_str + kb_cmd_str
            cmd_up  = joy_cmd_up + kb_cmd_up
            cmd_yaw = joy_cmd_yaw + kb_cmd_yaw
            cmd_pitch = joy_cmd_pitch + kb_cmd_pitch

            if dt > 0:
                # 视角旋转积分
                self.yaw += cmd_yaw * dt
                self.pitch += cmd_pitch * dt
                # 限制俯仰角，防止视角“倒翻”造成万向节死锁和眩晕
                self.pitch = max(min(self.pitch, math.pi/2 - 0.05), -math.pi/2 + 0.05)
                
                # ==========================================
                # 核心修改：基于摄像机自身坐标系的3D移动
                # 1. 构建从摄像机坐标系到世界坐标系的旋转矩阵
                # 2. 将摄像机本地移动向量转换为世界坐标系移动向量
                # ==========================================
                # 欧拉角转旋转矩阵 (roll->pitch->yaw 顺序，匹配ROS tf)
                rot_matrix = tf.transformations.euler_matrix(self.roll, self.pitch, self.yaw)
                # 摄像机本地移动向量 (X:前向, Y:横向, Z:垂直)
                local_move = [
                    cmd_fwd,  # X轴：前进/后退
                    -cmd_str, # Y轴：左/右平移 (取反匹配摇杆方向)
                    -cmd_up,  # Z轴：上升/下降 (取反适配NED坐标系)
                    0.0       # 齐次坐标占位符
                ]
                # 本地向量转换到世界坐标系
                world_move = rot_matrix.dot(local_move)
                # 提取世界坐标系下的移动增量
                dx = world_move[0]
                dy = world_move[1]
                dz = world_move[2]
                
                # 位置积分
                self.x += dx * dt
                self.y += dy * dt
                self.z += dz * dt
                
            # ==========================================
            # 4. 广播虚拟摄像机 TF
            # ==========================================
            quaternion = tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)
            self.tf_broadcaster.sendTransform(
                (self.x, self.y, self.z),
                quaternion,
                rospy.Time.now(),
                "spectator_link",  # 我们的不可见摄像机坐标系
                "map"              # 锚定在全局坐标系
            )
            
            rate.sleep()

if __name__ == '__main__':
    try:
        cam = SpectatorCamera()
        cam.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # 退出时务必恢复终端设置，否则终端会乱码
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))