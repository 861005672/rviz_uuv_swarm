#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rosservice
from sensor_msgs.msg import Joy
from uuv_interface.srv import SetWrench3D, SetWrench3DRequest

class UUVJoyController:
    def __init__(self):
        rospy.init_node('uuv_joy_controller', anonymous=True)

        # ================= 物理输出上限设定 =================
        # 仅保留欠驱动 UUV 的三个有效自由度：X轴推力, 俯仰力矩(Pitch), 偏航力矩(Yaw)
        self.max_force_x = rospy.get_param('~max_force_x', 200.0)          # N
        self.max_torque_pitch = rospy.get_param('~max_torque_pitch', 50.0) # N.m
        self.max_torque_yaw = rospy.get_param('~max_torque_yaw', 30.0)     # N.m

        self.target_forces = [0.0] * 6  # [fx, fy, fz, tx, ty, tz]
        self.active_uuv_idx = 0
        self.available_services = []
        self.current_proxy = None
        
        # 状态记录
        self.last_button_state = 0
        
        # 【核心修复】：用于修复 Linux 系统下手柄扳机初始值为 0.0 的 Bug
        self.lt_initialized = False
        self.rt_initialized = False

        # 订阅手柄硬件输入
        rospy.Subscriber('/joy', Joy, self.joy_callback)
        # 定时器：每两秒扫描一次服务，自动发现新的 UUV
        rospy.Timer(rospy.Duration(2.0), self.scan_services)
        # 控制循环 (20Hz): 持续向当前接管的 UUV 发送力矩指令
        rospy.Timer(rospy.Duration(0.1), self.control_loop)

        rospy.loginfo("[JoyController] Started! Awaiting manual override...")

    def scan_services(self, event=None):
        try:
            services = rosservice.get_service_list()
            actuator_srvs = [s for s in services if s.endswith('/set_actuator_input')]
            actuator_srvs.sort()

            # 如果发现了网络中 UUV 数量变化，更新列表
            if actuator_srvs != self.available_services:
                self.available_services = actuator_srvs
                rospy.loginfo(f"[JoyController] Discovered {len(self.available_services)} UUVs in network.")
                
                # 安全越界保护
                if self.active_uuv_idx >= len(self.available_services):
                    self.active_uuv_idx = 0
                if self.current_proxy is None and len(self.available_services) > 0:
                    self._update_proxy()
        except Exception as e:
            pass

    def _update_proxy(self):
        if len(self.available_services) == 0:
            self.current_proxy = None
            return
        srv_name = self.available_services[self.active_uuv_idx]
        self.current_proxy = rospy.ServiceProxy(srv_name, SetWrench3D)
        
        uuv_name = srv_name.split('/')[1]
        rospy.loginfo(f"\033[92m[JoyController] 🎮 Control Transferred To: {uuv_name}\033[0m")

    def stop_current_uuv(self):
        """切走控制权前，给当前 UUV 发送清零指令（急停）"""
        if self.current_proxy is not None:
            try:
                # 默认初始化的 Request 的 6 个自由度全为 0.0
                req = SetWrench3DRequest()
                self.current_proxy(req)
                uuv_name = self.available_services[self.active_uuv_idx].split('/')[1]
                rospy.loginfo(f"\033[93m[JoyController] 🛑 Emergency STOP sent to {uuv_name} before switching.\033[0m")
            except rospy.ServiceException:
                pass

    def joy_callback(self, msg):
        if len(msg.axes) < 6 or len(msg.buttons) < 1:
            return

        # ================== 1. 切换 UUV 及停机逻辑 ==================
        switch_btn = msg.buttons[0] # 通常为 Xbox 的 'A' 键 或 PS 的 'X' 键
        # 边缘触发 (防抖): 只有在按下瞬间才切换
        if switch_btn == 1 and self.last_button_state == 0:
            if len(self.available_services) > 0:
                # 在转移控制权前，强行停掉正在被控制的 UUV！
                self.stop_current_uuv()
                
                # 切换到下一台 UUV
                self.active_uuv_idx = (self.active_uuv_idx + 1) % len(self.available_services)
                self._update_proxy()
        self.last_button_state = switch_btn

        # ================== 2. 欠驱动手柄映射 ==================
        def deadzone(val):
            return val if abs(val) > 0.05 else 0.0

        # 1. 主推力 (Surge, X轴)：左摇杆前后 (Axes 1)
        fx = deadzone(msg.axes[1]) * self.max_force_x

        # 2. 俯仰力矩 (Pitch, Y轴)：右摇杆前后 (Axes 4)
        ty = deadzone(-msg.axes[4]) * self.max_torque_pitch

        # 3. 偏航力矩 (Yaw, Z轴)：RT 和 LT 的综合控制
        # 拦截 Linux 手柄刚插上时的无脑 0.0 初始值
        if msg.axes[2] != 0.0:
            self.lt_initialized = True
        if msg.axes[5] != 0.0:
            self.rt_initialized = True
            
        # 如果从插上手柄到现在都没被按过，强行视为未按下状态 (1.0)
        raw_lt = msg.axes[2] if self.lt_initialized else 1.0
        raw_rt = msg.axes[5] if self.rt_initialized else 1.0
        
        # 换算扳机行程：从 [1.0(松开), -1.0(按到底)] 映射到 [0.0, 1.0] 物理按压深度
        lt_pressed = (1.0 - raw_lt) / 2.0 
        rt_pressed = (1.0 - raw_rt) / 2.0 
        
        # 假设 RT 是右转(正向力矩)，LT 是左转(负向力矩)
        yaw_cmd = rt_pressed - lt_pressed
        tz = deadzone(yaw_cmd) * self.max_torque_yaw

        # 4. 强制屏蔽其余欠驱动维度 (Sway, Heave, Roll)
        fy = 0.0
        fz = 0.0
        tx = 0.0

        self.target_forces = [fx, fy, fz, tx, ty, tz]

    def control_loop(self, event):
        if self.current_proxy is None:
            return

        # 组装请求指令
        req = SetWrench3DRequest()
        req.force_x = self.target_forces[0]
        req.force_y = self.target_forces[1]
        req.force_z = self.target_forces[2]
        req.torque_x = self.target_forces[3]
        req.torque_y = self.target_forces[4]
        req.torque_z = self.target_forces[5]

        try:
            # 通过服务硬性覆盖 C++ 层的期望力/力矩
            self.current_proxy(req)
        except rospy.ServiceException:
            pass

if __name__ == '__main__':
    try:
        UUVJoyController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass