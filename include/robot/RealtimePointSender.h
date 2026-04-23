#pragma once

#include "robot/RobotUdpClient.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace cri {

// 三次多项式插值实时发送器。
// 输入离散路径点后，内部生成平滑轨迹并按周期发送到机器人控制UDP端口。
class RealtimePointSender {
public:
    explicit RealtimePointSender(std::shared_ptr<RobotUdpClient> udpClient);
    ~RealtimePointSender();

    void setCycleMs(uint8_t durationMs);    // 设置发送周期[1,16]ms
    void setStartBuffer(uint8_t points);    // 设置预填充缓冲点数[1,100]
    void setCommandType(uint8_t type);      // 0=joint, 1=cartesian

    // 装载路径点并预生成插值后的控制点序列。
    bool loadTrajectory(const std::vector<Waypoint>& points); // 加载并离散化轨迹
    // 启动后台发送线程。
    bool startStreaming();
    void stopStreaming();
    bool isStreaming() const;

    // 测试/诊断接口：从内部缓冲取下一条已生成控制点。
    bool tryGenerateNext(CommandData* out); // 仅从内部缓存取点，不触发网络发送

private:
    static bool validateWaypoints(const std::vector<Waypoint>& points); // 轨迹合法性检查
    void sendLoop(); // 后台定时发送循环
    void buildPlannedCommands(const std::vector<Waypoint>& points); // 三次插值并生成离散控制点

    std::shared_ptr<RobotUdpClient> udpClient_; // 实际发送承载
    mutable std::mutex mutex_; // 保护参数与缓存
    std::vector<CommandData> plannedCommands_; // 预生成控制点序列
    size_t nextCommandIndex_ {0}; // tryGenerateNext 的读取游标

    uint8_t cycleMs_ {1}; // 控制周期
    uint8_t startBuffer_ {3}; // 启动缓冲点数
    uint8_t commandType_ {0}; // 指令类型：关节/笛卡尔

    std::atomic<bool> streaming_ {false}; // 发送线程状态
    std::thread sendThread_; // 后台发送线程
};

} // namespace cri
