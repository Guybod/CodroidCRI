#include "robot/RealtimePointSender.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace cri {

// 构造实时发送器，持有UDP客户端用于后续控制点发送。
RealtimePointSender::RealtimePointSender(std::shared_ptr<RobotUdpClient> udpClient)
    : udpClient_(std::move(udpClient)) {}

// 析构时确保发送线程已安全停止。
RealtimePointSender::~RealtimePointSender() {
    stopStreaming();
}

// 设置发送周期（ms），超出范围时自动钳制到[1,16]。
void RealtimePointSender::setCycleMs(uint8_t durationMs) {
    std::lock_guard<std::mutex> lock(mutex_);
    cycleMs_ = static_cast<uint8_t>(std::clamp<int>(durationMs, 1, 16));
}

// 设置启动缓冲点数量，超出范围时自动钳制到[1,100]。
void RealtimePointSender::setStartBuffer(uint8_t points) {
    std::lock_guard<std::mutex> lock(mutex_);
    startBuffer_ = static_cast<uint8_t>(std::clamp<int>(points, 1, 100));
}

// 设置控制类型：0关节空间，非0统一按末端空间处理。
void RealtimePointSender::setCommandType(uint8_t type) {
    std::lock_guard<std::mutex> lock(mutex_);
    commandType_ = (type == 0) ? 0 : 1;
}

// 装载轨迹点并预生成插值控制序列。
bool RealtimePointSender::loadTrajectory(const std::vector<Waypoint>& points) {
    if (!validateWaypoints(points)) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    buildPlannedCommands(points);
    nextCommandIndex_ = 0;
    return !plannedCommands_.empty();
}

// 启动后台发送线程；若已在发送中则直接返回成功。
bool RealtimePointSender::startStreaming() {
    bool expected = false;
    if (!streaming_.compare_exchange_strong(expected, true)) {
        return true;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (!udpClient_ || plannedCommands_.empty()) {
        streaming_ = false;
        return false;
    }

    sendThread_ = std::thread(&RealtimePointSender::sendLoop, this);
    return true;
}

// 停止后台发送线程并等待线程退出。
void RealtimePointSender::stopStreaming() {
    bool expected = true;
    if (!streaming_.compare_exchange_strong(expected, false)) {
        return;
    }
    if (sendThread_.joinable()) {
        sendThread_.join();
    }
}

// 查询当前是否处于发送状态。
bool RealtimePointSender::isStreaming() const {
    return streaming_.load();
}

// 取出下一条已生成控制点（主要用于测试或离线验证）。
bool RealtimePointSender::tryGenerateNext(CommandData* out) {
    if (!out) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (nextCommandIndex_ >= plannedCommands_.size()) {
        return false;
    }
    *out = plannedCommands_[nextCommandIndex_++];
    return true;
}

// 校验轨迹点合法性：至少2点，且每段时长必须大于0。
bool RealtimePointSender::validateWaypoints(const std::vector<Waypoint>& points) {
    if (points.size() < 2) {
        return false;
    }
    for (const auto& p : points) {
        if (p.segmentTimeSec <= 1e-6) {
            return false;
        }
    }
    return true;
}

// 发送线程主体：先预填充，再按周期发送剩余控制点。
void RealtimePointSender::sendLoop() {
    std::vector<CommandData> localCommands;
    uint8_t localStartBuffer = 3;
    uint8_t localCycleMs = 1;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        localCommands = plannedCommands_;
        localStartBuffer = startBuffer_;
        localCycleMs = cycleMs_;
    }

    if (localCommands.empty() || !udpClient_) {
        streaming_ = false;
        return;
    }

    // 先预填充启动缓冲点，避免机器人因缓冲不足导致起步抖动。
    size_t prefill = std::min<size_t>(localStartBuffer, localCommands.size());
    for (size_t i = 0; i < prefill && streaming_.load(); ++i) {
        if (!udpClient_->sendCommand(localCommands[i])) {
            streaming_ = false;
            return;
        }
    }

    auto nextWake = std::chrono::steady_clock::now();
    for (size_t i = prefill; i < localCommands.size() && streaming_.load(); ++i) {
        nextWake += std::chrono::milliseconds(localCycleMs);
        if (!udpClient_->sendCommand(localCommands[i])) {
            streaming_ = false;
            return;
        }
        std::this_thread::sleep_until(nextWake);
    }

    streaming_ = false;
}

// 根据路径点构建三次插值轨迹，并离散为固定周期的控制点序列。
void RealtimePointSender::buildPlannedCommands(const std::vector<Waypoint>& points) {
    plannedCommands_.clear();
    const double dt = static_cast<double>(cycleMs_) / 1000.0;
    uint64_t tickCounter = 0;

    // 使用三次多项式生成轨迹：
    // p(t)=a0+a1*t+a2*t^2+a3*t^3，约束条件为 p(0), p(T), v(0), v(T)。
    for (size_t seg = 0; seg + 1 < points.size(); ++seg) {
        const auto& start = points[seg];
        const auto& end = points[seg + 1];
        const double T = start.segmentTimeSec;
        const size_t samples = std::max<size_t>(1, static_cast<size_t>(std::ceil(T / dt)));

        for (size_t k = 0; k < samples; ++k) {
            const double t = std::min(static_cast<double>(k) * dt, T);
            CommandData cmd;
            cmd.timestamp = static_cast<int64_t>(tickCounter++ * cycleMs_);
            cmd.type = commandType_;

            for (size_t axis = 0; axis < 6; ++axis) {
                const double p0 = start.position[axis];
                const double p1 = end.position[axis];
                const double v0 = start.velocity[axis];
                const double v1 = end.velocity[axis];

                const double a0 = p0;
                const double a1 = v0;
                const double a2 = (3.0 * (p1 - p0) / (T * T)) - ((2.0 * v0 + v1) / T);
                const double a3 = (-2.0 * (p1 - p0) / (T * T * T)) + ((v0 + v1) / (T * T));

                cmd.position[axis] = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
            }
            plannedCommands_.push_back(cmd);
        }
    }

    // 强制追加终点，确保末端停在目标姿态。
    CommandData finalCmd;
    finalCmd.type = commandType_;
    finalCmd.timestamp = static_cast<int64_t>(tickCounter * cycleMs_);
    for (size_t axis = 0; axis < 6; ++axis) {
        finalCmd.position[axis] = points.back().position[axis];
    }
    plannedCommands_.push_back(finalCmd);
}

} // namespace cri
