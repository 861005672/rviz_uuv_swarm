#ifndef PID_H
#define PID_H

#include <cmath>
#include <algorithm>
#include <string>
#include <ros/ros.h>

namespace uuv_control {

    // 独立封装的 PID 控制器类
    class PID {
    private:
        double kp_;
        double ki_;
        double kd_;
        
        // 限幅参数
        double max_output_;
        double max_integral_;

        // 内部状态
        double integral_;
        double prev_error_;

        bool first_run_;

    public:
        PID() : kp_(0.0), ki_(0.0), kd_(0.0), max_output_(0.0), max_integral_(0.0), integral_(0.0), prev_error_(0.0), first_run_(true) {}

        // 初始化参数
        void init(double kp, double ki, double kd, double max_output, double max_integral) {
            kp_ = kp;
            ki_ = ki;
            kd_ = kd;
            max_output_ = max_output;
            max_integral_ = max_integral;
            integral_ = 0.0;
            prev_error_ = 0.0;
            first_run_ = true;
        }

        // 计算 PID 输出
        double compute(double error, double dt) {
            if (dt <= 0.0) return 0.0;

            // 【核心安全机制】：消除第一帧的微分冲击
            if (first_run_) {
                prev_error_ = error; // 强行让第一帧的微分为 0
                first_run_ = false;
            }

            // 1. 积分项累加
            integral_ += error * dt;

            // 2. 积分限幅 (Anti-windup)
            if (max_integral_ > 0) {
                integral_ = std::max(-max_integral_, std::min(integral_, max_integral_));
            }

            // 3. 微分项计算
            double derivative = (error - prev_error_) / dt;

            // 4. 总输出计算
            double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

            // 记录当前误差供下次计算使用
            prev_error_ = error;

            // 5. 输出终极限幅
            output = std::max(-max_output_, std::min(output, max_output_));

            return output;
        }

        // 动态更新参数（不清空内部积分状态，保证调参平滑）
        void setParams(std::string info, double kp, double ki, double kd, double max_output, double max_integral) {
            kp_ = kp;
            ki_ = ki;
            kd_ = kd;
            max_output_ = max_output;
            max_integral_ = max_integral;

            ROS_INFO_STREAM("[PID] "<<info<<": new parameters: kp:"<<kp_<<", ki:"<<ki_<<", kd:"<<kd_<<", max_output:"<<max_output_<<", max_integral:"<<max_integral_);
        }

        // 重置内部状态
        void reset() {
            integral_ = 0.0;
            prev_error_ = 0.0;
            first_run_ = true; // 【核心新增】
        }
    };

} // namespace uuv_control

#endif // PID_H