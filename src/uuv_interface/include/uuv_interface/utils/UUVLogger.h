#ifndef UUV_LOGGER_H
#define UUV_LOGGER_H

#include <fstream>
#include <sstream>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <iomanip>
#include <ros/ros.h>

namespace uuv_interface {

// ==========================================
// 日志级别枚举
// ==========================================
enum class LogLevel { INFO, WARN, ERROR };

// ==========================================
// 异步双缓冲日志核心器
// ==========================================
class UUVLogger {
private:
    std::ofstream file_;
    
    // 双缓冲设计：一个供主线程极速写入，一个供后台线程慢慢刷盘
    std::string active_buffer_;
    std::string flush_buffer_;
    size_t flush_threshold_;

    // 异步多线程组件
    std::thread bg_thread_;
    std::mutex mtx_;
    std::condition_variable cv_;
    std::atomic<bool> running_;

    // 后台 I/O 线程的专属业务逻辑
    void processIOLoop() {
        while (running_) {
            std::unique_lock<std::mutex> lock(mtx_);
            
            // 线程休眠，直到：缓存达到阈值，或者超时 1 秒（防止少量日志久不刷盘），或者系统要退出
            cv_.wait_for(lock, std::chrono::seconds(1), [this] {
                return !running_ || active_buffer_.size() >= flush_threshold_;
            });

            if (active_buffer_.empty()) continue;

            // 【性能魔法】：指针级内存交换（耗时几纳秒），极速释放锁，让主线程不用等！
            flush_buffer_.swap(active_buffer_);
            lock.unlock(); 

            // 锁释放后，后台线程慢慢把 flush_buffer_ 里的海量数据写进物理硬盘
            if (file_.is_open()) {
                file_ << flush_buffer_;
                file_.flush();
            }
            flush_buffer_.clear(); // 清空内容，但 reserve 预留的内存空间不会被销毁
        }
        
        // 当节点被销毁时，进行最后一次强行刷盘，保证一滴数据都不丢
        std::unique_lock<std::mutex> lock(mtx_);
        if (!active_buffer_.empty() && file_.is_open()) {
            file_ << active_buffer_;
            file_.flush();
        }
    }

public:
    // 默认缓存阈值为 512KB
    UUVLogger(const std::string& filepath, size_t flush_threshold = 1024 * 512)
        : flush_threshold_(flush_threshold), running_(true) {
        
        file_.open(filepath, std::ios::out | std::ios::trunc);
        
        // 提前在堆区开辟好物理内存，彻底消灭运行中的 malloc 开销
        active_buffer_.reserve(flush_threshold_ + 4096);
        flush_buffer_.reserve(flush_threshold_ + 4096);
        
        // 启动专属的后台 I/O 线程
        bg_thread_ = std::thread(&UUVLogger::processIOLoop, this);
    }

    ~UUVLogger() {
        running_ = false;
        cv_.notify_one(); // 敲钟叫醒后台线程，准备下班
        if (bg_thread_.joinable()) {
            bg_thread_.join(); // 等待后台线程安全把最后一点数据写完
        }
        if (file_.is_open()) file_.close();
    }

    // 提供给代理类的线程安全追加接口
    void append(const std::string& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        active_buffer_ += msg;
        if (active_buffer_.size() >= flush_threshold_) {
            cv_.notify_one(); // 数据满了，敲钟让后台线程起来干活
        }
    }
};

// ==========================================
// 代理流类：捕获时间戳、级别，并实现 << 拼接
// ==========================================
class LogStream {
private:
    std::shared_ptr<UUVLogger> logger_;
    LogLevel level_;
    double time_;
    std::ostringstream oss_;

public:
    LogStream(std::shared_ptr<UUVLogger> logger, LogLevel level, double time) 
        : logger_(logger), level_(level), time_(time) {}
    
    // 代理对象生命周期结束时（即一行 UUV_INFO 代码执行完毕时），组装字符串并提交
    ~LogStream() {
        if (logger_) {
            std::ostringstream final_msg;
            // 1. 压入跨线程/进程完全对齐的 ROS 时间戳
            final_msg << std::fixed << std::setprecision(4) << "[" << time_ << "] ";
            
            // 2. 根据级别压入不同的 ANSI 颜色代码与文字标签
            switch(level_) {
                case LogLevel::INFO:  final_msg << "[INFO] "; break;  // 绿色
                case LogLevel::WARN:  final_msg << "[WARN] "; break;  // 黄色
                case LogLevel::ERROR: final_msg << "[ERROR] "; break; // 红色
            }
            
            // 3. 压入用户真正打印的内容，并在末尾压入颜色复位码 \033[0m 与换行符
            final_msg << oss_.str() << "\n"; 
            
            // 提交给核心日志器
            logger_->append(final_msg.str());
        }
    }

    template <typename T>
    LogStream& operator<<(const T& value) {
        oss_ << value;
        return *this;
    }
};

} // namespace uuv_interface

// ==========================================
// 对外暴露的极简调用宏 (自动获取当前精确到毫秒的 ros::Time)
// ==========================================
#define UUV_INFO  uuv_interface::LogStream(this->uuv_logger_, uuv_interface::LogLevel::INFO, ros::Time::now().toSec())
#define UUV_WARN  uuv_interface::LogStream(this->uuv_logger_, uuv_interface::LogLevel::WARN, ros::Time::now().toSec())
#define UUV_ERROR uuv_interface::LogStream(this->uuv_logger_, uuv_interface::LogLevel::ERROR, ros::Time::now().toSec())

#endif