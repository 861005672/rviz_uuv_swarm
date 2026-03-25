import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ==========================================
# 1. 基础数学与 2D 杜宾斯逻辑 (复刻自 src/dubinsmaneuver2d.jl)
# ==========================================
def mod2pi(theta):
    return theta - 2.0 * np.pi * np.floor(theta / (2.0 * np.pi))

class DubinsManeuver2D:
    def __init__(self, qi, qf, rhomin):
        self.qi = np.array(qi)  # [x, y, heading]
        self.qf = np.array(qf)
        self.rhomin = rhomin
        self.maneuver = self._solve()

    def _solve(self):
        dx, dy = self.qf[0] - self.qi[0], self.qf[1] - self.qi[1]
        D = np.sqrt(dx**2 + dy**2)
        d = D / self.rhomin
        
        rot = mod2pi(np.arctan2(dy, dx))
        a, b = mod2pi(self.qi[2] - rot), mod2pi(self.qf[2] - rot)
        sa, ca, sb, cb = np.sin(a), np.cos(a), np.sin(b), np.cos(b)

        # 尝试所有组合
        paths = [
            self._LSL(a, b, d, sa, ca, sb, cb), self._RSR(a, b, d, sa, ca, sb, cb),
            self._LSR(a, b, d, sa, ca, sb, cb), self._RSL(a, b, d, sa, ca, sb, cb),
            self._RLR(a, b, d, sa, ca, sb, cb), self._LRL(a, b, d, sa, ca, sb, cb)
        ]
        valid_paths = [p for p in paths if p['length'] != float('inf')]
        return min(valid_paths, key=lambda x: x['length']) if valid_paths else None

    # 各类 2D 杜宾斯几何解 (与 src/dubinsmaneuver2d.jl 公式一致)
    def _LSL(self, a, b, d, sa, ca, sb, cb):
        aux = np.arctan2(cb - ca, d + sa - sb)
        t, q = mod2pi(-a + aux), mod2pi(b - aux)
        p = np.sqrt(max(0, 2 + d**2 - 2*np.cos(a-b) + 2*d*(sa-sb)))
        return {'t': t, 'p': p, 'q': q, 'length': (t+p+q)*self.rhomin, 'case': "LSL"}

    def _RSR(self, a, b, d, sa, ca, sb, cb):
        aux = np.arctan2(ca - cb, d - sa + sb)
        t, q = mod2pi(a - aux), mod2pi(-b + aux)
        p = np.sqrt(max(0, 2 + d**2 - 2*np.cos(a-b) + 2*d*(sb-sa)))
        return {'t': t, 'p': p, 'q': q, 'length': (t+p+q)*self.rhomin, 'case': "RSR"}

    def _LSR(self, a, b, d, sa, ca, sb, cb):
        aux1 = -2 + d**2 + 2*np.cos(a-b) + 2*d*(sa+sb)
        if aux1 < 0: return {'length': float('inf')}
        p = np.sqrt(aux1)
        aux2 = np.arctan2(-ca-cb, d+sa+sb) - np.arctan2(-2.0, p)
        t, q = mod2pi(-a + aux2), mod2pi(-b + aux2)
        return {'t': t, 'p': p, 'q': q, 'length': (t+p+q)*self.rhomin, 'case': "LSR"}

    def _RSL(self, a, b, d, sa, ca, sb, cb):
        aux1 = d**2 - 2 + 2*np.cos(a-b) - 2*d*(sa+sb)
        if aux1 < 0: return {'length': float('inf')}
        p = np.sqrt(aux1)
        aux2 = np.arctan2(ca+cb, d-sa-sb) - np.arctan2(2.0, p)
        t, q = mod2pi(a - aux2), mod2pi(b - aux2)
        return {'t': t, 'p': p, 'q': q, 'length': (t+p+q)*self.rhomin, 'case': "RSL"}

    def _RLR(self, a, b, d, sa, ca, sb, cb):
        aux = (6 - d**2 + 2*np.cos(a-b) + 2*d*(sa-sb))/8
        if abs(aux) > 1: return {'length': float('inf')}
        p = mod2pi(-np.arccos(aux))
        t = mod2pi(a - np.arctan2(ca-cb, d-sa+sb) + p/2)
        q = mod2pi(a - b - t + p)
        return {'t': t, 'p': p, 'q': q, 'length': (t+p+q)*self.rhomin, 'case': "RLR"}

    def _LRL(self, a, b, d, sa, ca, sb, cb):
        aux = (6 - d**2 + 2*np.cos(a-b) + 2*d*(-sa+sb))/8
        if abs(aux) > 1: return {'length': float('inf')}
        p = mod2pi(-np.arccos(aux))
        t = mod2pi(-a + np.arctan2(-ca+cb, d+sa-sb) + p/2)
        q = mod2pi(b - a - t + p)
        return {'t': t, 'p': p, 'q': q, 'length': (t+p+q)*self.rhomin, 'case': "LRL"}

    def get_pos(self, offset):
        noffset = offset / self.rhomin
        qi = [0., 0., self.qi[2]]
        l1, l2 = self.maneuver['t'], self.maneuver['p']
        case = self.maneuver['case']
        q1 = self._seg(l1, qi, case[0])
        q2 = self._seg(l2, q1, case[1])
        q = self._seg(noffset, qi, case[0]) if noffset < l1 else (self._seg(noffset-l1, q1, case[1]) if noffset < l1+l2 else self._seg(noffset-l1-l2, q2, case[2]))
        return [q[0]*self.rhomin + self.qi[0], q[1]*self.rhomin + self.qi[1], mod2pi(q[3] if len(q)>3 else q[2])]

    def _seg(self, off, qi, c):
        if c == 'L': return [qi[0] + np.sin(qi[2]+off) - np.sin(qi[2]), qi[1] - np.cos(qi[2]+off) + np.cos(qi[2]), qi[2] + off]
        if c == 'R': return [qi[0] - np.sin(qi[2]-off) + np.sin(qi[2]), qi[1] + np.cos(qi[2]-off) - np.cos(qi[2]), qi[2] - off]
        return [qi[0] + np.cos(qi[2])*off, qi[1] + np.sin(qi[2])*off, qi[2]]

# ==========================================
# 2. 3D 杜宾斯耦合逻辑 (复刻自 src/dubinsmaneuver3d.jl)
# ==========================================
class DubinsManeuver3D:
    def __init__(self, qi, qf, rhomin, pitchlims):
        self.qi, self.qf = np.array(qi), np.array(qf) # [x,y,z,yaw,pitch]
        self.rhomin = rhomin
        self.pitchlims = pitchlims
        self.path = self._solve()
        self.length = self.path[1].maneuver['length'] if self.path else -1.0

    def _solve(self):
        # 迭代寻找最佳水平半径
        b, step = 1.0, 0.1
        while len(self._try(self.rhomin * b)) < 2 and b < 1000: b *= 2.0
        best_fb = self._try(self.rhomin * b)
        
        # 局部优化
        curr_b = b
        while abs(step) > 1e-10:
            c = max(1.0, curr_b + step)
            fc = self._try(self.rhomin * c)
            if len(fc) > 0 and fc[1].maneuver['length'] < best_fb[1].maneuver['length']:
                curr_b, best_fb, step = c, fc, step * 2.
            else:
                step *= -0.1
        return best_fb

    def _try(self, h_rad):
        # 核心：1/rho^2 = 1/Rh^2 + 1/Rv^2 -> Rv = 1 / sqrt(1/rho^2 - 1/Rh^2)
        Dlat = DubinsManeuver2D(self.qi[[0,1,3]], self.qf[[0,1,3]], h_rad)
        if not Dlat.maneuver: return []
        
        v_curv_sq = (1.0/self.rhomin)**2 - (1.0/h_rad)**2
        if v_curv_sq < 1e-10: return []
        v_rad = 1.0 / np.sqrt(v_curv_sq)

        # 纵向 SZ 投影规划
        qi3D, qf3D = [0., self.qi[2], self.qi[4]], [Dlat.maneuver['length'], self.qf[2], self.qf[4]]
        Dlon = DubinsManeuver2D(qi3D, qf3D, v_rad)
        
        if not Dlon.maneuver or Dlon.maneuver['case'] in ["RLR", "LRL"]: return []
        
        # 俯仰角限幅检查
        p_check = self.qi[4] + (Dlon.maneuver['t'] if Dlon.maneuver['case'][0]=='L' else -Dlon.maneuver['t'])
        if p_check < self.pitchlims[0] or p_check > self.pitchlims[1]: return []
        return [Dlat, Dlon]

    def sampling(self, num=500):
        # 将纵横曲线映射回 3D
        Dlat, Dlon = self.path
        l_total = Dlon.maneuver['length']
        offsets = np.linspace(0, l_total, num)
        pts = []
        for off in offsets:
            qSZ = Dlon.get_pos(off)
            qXY = Dlat.get_pos(qSZ[0])
            pts.append([qXY[0], qXY[1], qSZ[1], qXY[2], qSZ[2], qSZ[0]])
        return np.array(pts)
    
    # 在 DubinsManeuver3D 类中添加此方法
    def sampling_by_spacing(self, spacing=2.0):
        """
        基于固定间距进行采样 (复刻自 Julia 版 src/dubinsmaneuver2d.jl 的 getSamplingPoints 逻辑)
        """
        Dlat, Dlon = self.path
        l_total = Dlon.maneuver['length']
        
        # 使用 np.arange 生成固定间距的偏移量
        # arange 不包含终点，所以我们需要手动加上终点以保证轨迹闭合
        offsets = np.arange(0, l_total, spacing)
        if offsets[-1] < l_total:
            offsets = np.append(offsets, l_total)
            
        pts = []
        for off in offsets:
            qSZ = Dlon.get_pos(off)
            qXY = Dlat.get_pos(qSZ[0])
            pts.append([qXY[0], qXY[1], qSZ[1], qXY[2], qSZ[2], qSZ[0]])
        return np.array(pts)

# ==========================================
# 3. 验证与全航路点输出
# ==========================================
if __name__ == "__main__":
    # 格式: [北(m), 东(m), 下(m), 航向角(rad), 俯仰角(rad)]
    qi = [100.0, 300.0, 500.0, np.radians(180.0), np.radians(-30.0)]
    qf = [500.0, 550.0, 100.0, np.radians(90.0), np.radians(30.0)]
    rhomin = 50.0
    pitchlims = np.radians([-30.0, 30.0])

    dubins = DubinsManeuver3D(qi, qf, rhomin, pitchlims)
    
    print("\n" + "="*80)
    print(f"{'3D DUBINS REPLICATION REPORT':^80}")
    print("-" * 80)
    print(f"L [Python]: {dubins.length:9.3f} m")
    print(f"L [Julia ]: 467.700 m")
    print("-" * 80)

    # 高采样用于生成平滑轨迹
    smooth_path = dubins.sampling(1000)
    # 低采样用于显示航路点
    waypoint_spacing = 100.0
    # 打印所有航路点
    waypoint_path = dubins.sampling_by_spacing(spacing=waypoint_spacing)
    
    print(f"{'Index':>5} | {'X [m]':>10} | {'Y [m]':>10} | {'Z [m]':>10} | {'Yaw [rad]':>12} | {'Pitch [rad]':>12}")
    print("-" * 80)
    for i, pt in enumerate(waypoint_path):
        # pt 格式为 [x, y, z, yaw, pitch]
        print(f"{i:5d} | {pt[0]:10.3f} | {pt[1]:10.3f} | {pt[2]:10.3f} | {pt[3]:12.6f} | {pt[4]:12.6f}")
    
    print("="*80 + "\n")


    # --- 4. 可视化部分 (新增垂直解耦剖视图) ---
    # 设置 1x3 的宽幅画布
    fig = plt.figure(figsize=(22, 7))
    
# ==========================================
    # 子图 1: 3D NED 总体视图
    # ==========================================
    ax1 = fig.add_subplot(131, projection='3d')
    # A. 绘制路径与阴影曲面
    # 1. 绘制在起始深度平面的 2D 杜宾斯底线 (绿色半透明实线)
    z_start_plane = np.full_like(smooth_path[:, 2], qi[2])
    ax1.plot(smooth_path[:, 0], smooth_path[:, 1], z_start_plane, 
             color='blue', lw=2, alpha=0.4, label='2D Path on Start Plane')

    # 2. 绘制 SZ 平面的半透明投影阴影 (绿色半透明曲面墙)
    # 使用 np.vstack 组合顶部和底部边缘的坐标点来生成网格面
    X_surf = np.vstack([smooth_path[:, 0], smooth_path[:, 0]])
    Y_surf = np.vstack([smooth_path[:, 1], smooth_path[:, 1]])
    Z_surf = np.vstack([z_start_plane, smooth_path[:, 2]])
    ax1.plot_surface(X_surf, Y_surf, Z_surf, color='blue', alpha=0.15, shade=False)

    # 3. 绘制最终的 3D 规划曲线与离散航路点
    # 这里将原来的虚线改为了蓝色实线，以便在阴影背景上更清晰
    ax1.plot(smooth_path[:, 0], smooth_path[:, 1], smooth_path[:, 2], 
             'b-', lw=1.5, alpha=0.8, label='3D Final Path')
    ax1.plot(waypoint_path[:, 0], waypoint_path[:, 1], waypoint_path[:, 2], 
             'r-o', markersize=4, lw=1.5, label='Waypoints')
    # B. 绘制起终点 (保持原样)
    ax1.scatter(qi[0], qi[1], qi[2], color='green', s=150, marker='^', label='Start')

    # 3D 姿态方向箭头
    arrow_len = 80.0
    ax1.quiver(qi[0], qi[1], qi[2], arrow_len*np.cos(qi[4])*np.cos(qi[3]), arrow_len*np.cos(qi[4])*np.sin(qi[3]), arrow_len*np.sin(qi[4]), color='green', lw=2.5)
    ax1.quiver(qf[0], qf[1], qf[2], arrow_len*np.cos(qf[4])*np.cos(qf[3]), arrow_len*np.cos(qf[4])*np.sin(qf[3]), arrow_len*np.sin(qf[4]), color='darkred', lw=2.5)

    ax1.set_xlabel('North (X) [m]'); ax1.set_ylabel('East (Y) [m]'); ax1.set_zlabel('Down (Depth) [m]')
    ax1.invert_zaxis() 
    ax1.view_init(elev=-135, azim=45) # 经典的 NED 俯瞰视角
    
    # 等比例缩放
    all_n, all_e, all_d = smooth_path[:, 0], smooth_path[:, 1], smooth_path[:, 2]
    max_range = np.array([all_n.max()-all_n.min(), all_e.max()-all_e.min(), all_d.max()-all_d.min()]).max() / 2.0
    mid_n, mid_e, mid_d = (all_n.max()+all_n.min())/2, (all_e.max()+all_e.min())/2, (all_d.max()+all_d.min())/2
    ax1.set_xlim(mid_n - max_range, mid_n + max_range)
    ax1.set_ylim(mid_e - max_range, mid_e + max_range)
    ax1.set_zlim(mid_d - max_range, mid_d + max_range)
    ax1.set_title('3D Dubins Trajectory (NED)')
    ax1.legend()

    # ==========================================
    # 子图 2: 水平面 2D 俯视图 (North-East Plane)
    # ==========================================
    ax2 = fig.add_subplot(132)
    # 横轴东(Y)，纵轴北(X)
    ax2.plot(smooth_path[:, 1], smooth_path[:, 0], 'b--', lw=1.2, alpha=0.7)
    ax2.plot(waypoint_path[:, 1], waypoint_path[:, 0], 'r-o', markersize=4)
    ax2.scatter(qi[1], qi[0], color='green', s=150, marker='^')
    ax2.scatter(qf[1], qf[0], color='darkred', s=150, marker='*')

    # 水平面偏航角箭头 (只看 Yaw)
    ax2.quiver(qi[1], qi[0], np.sin(qi[3]), np.cos(qi[3]), color='green', scale=15, width=0.008)
    ax2.quiver(qf[1], qf[0], np.sin(qf[3]), np.cos(qf[3]), color='darkred', scale=15, width=0.008)

    ax2.set_xlabel('East (Y) [m]'); ax2.set_ylabel('North (X) [m]')
    ax2.set_title('Horizontal Plane (NE)')
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.axis('equal') # 保证你看得出正圆弧

    # ==========================================
    # 子图 3: 垂直面 2D 剖视图 (ArcLength-Depth Plane)
    # ==========================================
    ax3 = fig.add_subplot(133)
    # 横轴是水平累积弧长 S (我们在修改类时追加的第 6 列数据，索引为 5)
    # 纵轴是深度 Z
    ax3.plot(smooth_path[:, 5], smooth_path[:, 2], 'b--', lw=1.2, alpha=0.7)
    ax3.plot(waypoint_path[:, 5], waypoint_path[:, 2], 'r-o', markersize=4)
    
    S_start, S_goal = smooth_path[0, 5], smooth_path[-1, 5]
    ax3.scatter(S_start, qi[2], color='green', s=150, marker='^')
    ax3.scatter(S_goal, qf[2], color='darkred', s=150, marker='*')

    # 垂直面俯仰角箭头 (看 Pitch)
    # 沿弧长方向为 cos(pitch)，深度方向为 sin(pitch)
    ax3.quiver(S_start, qi[2], np.cos(qi[4]), -np.sin(qi[4]), color='green', scale=15, width=0.008)
    ax3.quiver(S_goal, qf[2], np.cos(qf[4]), -np.sin(qf[4]), color='darkred', scale=15, width=0.008)

    ax3.set_xlabel('Horizontal Arc Length (S) [m]')
    ax3.set_ylabel('Down (Depth, Z) [m]')
    ax3.set_title('Vertical Plane (SZ)')
    ax3.grid(True, linestyle=':', alpha=0.6)
    
    # 核心：因为 Z 是 Down（深度向下），这里翻转 Y 轴，让深水区在下面
    ax3.invert_yaxis() 
    ax3.axis('equal') # 同样保持等比例，看出真实的垂直起伏

    plt.tight_layout()
    plt.show()