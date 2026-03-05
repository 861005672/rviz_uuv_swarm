#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <deque>

namespace uuv_interface {

    // 角度归一化函数：确保角度误差在 [-PI, PI] 之间，走最短路径转弯
    inline double wrapAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // 将体坐标系下的 3D 向量转换到 NED 世界坐标系下
    inline Eigen::Vector3d bodyToWorld(const Eigen::Vector3d& vec_body, double roll, double pitch, double yaw) {
        // 使用 Eigen 提供的轴角构造旋转矩阵 (注意 NED 坐标系下的 Z-Y-X 旋转顺序)
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
        // 四元数相乘代表连续旋转
        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
        // 返回旋转后的世界坐标系向量
        return q * vec_body;
    }

    // 将 NED 世界坐标系下的 3D 向量转换到体坐标系下
    inline Eigen::Vector3d worldToBody(const Eigen::Vector3d& vec_world, double roll, double pitch, double yaw) {
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
        // 四元数的逆，代表逆向旋转
        return q.inverse() * vec_world;
    }

    // 平滑取小值 (用于限制上限)
    inline double smoothMin(double a, double b, double k) {
        double h = std::max(k - std::abs(a - b), 0.0) / k;
        return std::min(a, b) - h * h * k * 0.25;
    }

    // 平滑取大值 (用于限制下限)
    inline double smoothMax(double a, double b, double k) {
        double h = std::max(k - std::abs(a - b), 0.0) / k;
        return std::max(a, b) + h * h * k * 0.25;
    }

/**
     * @brief 标量区间平滑限幅 (独立控制上下界是否平滑)
     * @param value 要限幅的原始值
     * @param min_val 下界
     * @param max_val 上界
     * @param transition_width 平滑过渡区宽度 (k)
     * @param smooth_bottom 是否在下界(min_val)附近进行平滑 (如果是误差/控制力，建议设为false)
     * @param smooth_top 是否在上界(max_val)附近进行平滑
     */
    inline double softClampScl(double value, double min_val, double max_val, double transition_width = 2.0, bool smooth_bottom = false, bool smooth_top = true) {
        // 下界处理：若不平滑，则使用绝对硬限幅 max()
        double clamped_bottom;
        if (smooth_bottom) {
            clamped_bottom = smoothMax(value, min_val, transition_width);
        } else {
            clamped_bottom = std::max(value, min_val);
        }
        
        // 上界处理：若不平滑，则使用绝对硬限幅 min()
        if (smooth_top) {
            return smoothMin(clamped_bottom, max_val, transition_width);
        } else {
            return std::min(clamped_bottom, max_val);
        }
    }

    /**
     * @brief 向量模长平滑限幅 (仅限制最大模长，可选择是否平滑原点)
     * @param vec 输入向量
     * @param max_norm 最大模长限幅
     * @param transition_width 平滑过渡区宽度 (k)
     * @param smooth_origin 是否在原点(模长接近0)附近平滑 (控制系统中强烈建议设为 false！)
     */
    inline Eigen::Vector3d softClampVec(const Eigen::Vector3d& vec, double max_norm, double transition_width = 2.0, bool smooth_origin = false) {
        double norm = vec.norm();
        if (norm < 1e-6) return vec;
        
        // 向量的下限天然是 0。通过 smooth_origin 参数控制下界平滑，上界默认启用平滑。
        double smooth_norm = softClampScl(norm, 0.0, max_norm, transition_width, smooth_origin, true);
        
        return vec * (smooth_norm / norm);
    }
    
/**
 * @brief 3D 轨迹最小二乘法速度预测器
 * @details 封装了历史记录队列的维护和速度预测算法，避免污染调用者的业务逻辑
 */
class LeastSquaresPredictor3D {
private:
    std::deque<double> times_;
    std::deque<Eigen::Vector3d> positions_;
    size_t max_size_;

public:
    // 构造函数，默认记录5帧历史
    LeastSquaresPredictor3D(size_t max_size = 5) : max_size_(max_size) {}

    // 动态调整历史窗口大小
    void setMaxSize(size_t size) { max_size_ = size; }

    // 添加新的状态记录
    void append(double t, const Eigen::Vector3d& p) {
        times_.push_back(t);
        positions_.push_back(p);
        if (times_.size() > max_size_) {
            times_.pop_front();
            positions_.pop_front();
        }
    }

    // 利用最小二乘法预测当前速度
    Eigen::Vector3d predict() const {
        int n = times_.size();
        if (n < 2) return Eigen::Vector3d::Zero();

        double sum_t = 0.0, sum_t2 = 0.0;
        Eigen::Vector3d sum_p = Eigen::Vector3d::Zero();
        Eigen::Vector3d sum_tp = Eigen::Vector3d::Zero();

        // 使用相对时间 t0 增加浮点数计算精度
        double t0 = times_.front(); 
        for (int i = 0; i < n; ++i) {
            double t = times_[i] - t0;
            Eigen::Vector3d p = positions_[i];

            sum_t += t;
            sum_t2 += t * t;
            sum_p += p;
            sum_tp += t * p;
        }

        double denominator = n * sum_t2 - sum_t * sum_t;
        
        // 防止除零或时间戳太近导致退化
        if (std::abs(denominator) < 1e-6) {
            double total_dt = times_.back() - times_.front();
            if (total_dt < 1e-4) return Eigen::Vector3d::Zero();
            return (positions_.back() - positions_.front()) / total_dt;
        }

        // 线性回归斜率公式
        return (n * sum_tp - sum_t * sum_p) / denominator;
    }
};
    
}

#endif