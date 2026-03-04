#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

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
     * @brief 标量区间平滑限幅 (可自定义最小值和最大值)
     * @param value 要限幅的原始值
     * @param min_val 下界
     * @param max_val 上界
     * @param transition_width 平滑过渡区宽度 (k)
     */
    inline double softClampScl(double value, double min_val, double max_val, double transition_width = 2.0) {
        // 先用 smoothMax 托底，再用 smoothMin 盖帽
        double clamped_bottom = smoothMax(value, min_val, transition_width);
        return smoothMin(clamped_bottom, max_val, transition_width);
    }


    /**
     * @brief 向量模长平滑限幅 (仅限制最大模长)
     */
    inline Eigen::Vector3d softClampVec(const Eigen::Vector3d& vec, double max_norm, double transition_width = 2.0) {
        double norm = vec.norm();
        if (norm < 1e-6) return vec;
        // 向量一般只限制上限，下限为 0
        double smooth_norm = softClampScl(norm, 0.0, max_norm, transition_width);
        return vec * (smooth_norm / norm);
    }
}

#endif