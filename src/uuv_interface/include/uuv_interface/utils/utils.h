#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <algorithm>

namespace uuv_interface {

    // 角度归一化函数：确保角度误差在 [-PI, PI] 之间，走最短路径转弯
    inline double wrapAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

}

#endif