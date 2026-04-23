#pragma once

#include "robot/CriRealtimeParser.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace cri {

class RobotUdpClient {
public:
    using FrameCallback = std::function<void(const RealtimeFrame&)>;
    using RawPacketCallback = std::function<void(const uint8_t*, size_t)>;

    // localReceive: 本地接收推流的绑定地址
    // robotControlTarget: 实时控制指令发送目标（通常机器人IP:9030）
    RobotUdpClient(UdpEndpoint localReceive, UdpEndpoint robotControlTarget);
    ~RobotUdpClient();

    // 启动UDP接收线程，用于接收机器人实时推流。
    bool startReceive();
    void stopReceive();
    bool isReceiving() const;

    // 序列化并发送一条实时控制指令到控制端口（默认9030）。
    bool sendCommand(const CommandData& cmd);
    // 通用UDP发送接口，可发任意二进制数据到目标端点。
    bool sendRaw(const uint8_t* data, size_t size, const UdpEndpoint& target);

    // 绑定解析器：接收线程会自动解析数据并触发回调。
    void bindParser(std::shared_ptr<CriRealtimeParser> parser); // 绑定解析器后自动做包解析
    void setOnFrame(FrameCallback callback); // 解析成功后触发的高层帧回调
    void setOnRawPacket(RawPacketCallback callback); // 收到原始包后立即触发的底层回调

private:
    void receiveLoop(); // 后台接收线程主循环
    static std::vector<uint8_t> serializeCommand(const CommandData& cmd); // CommandData -> 二进制协议包

    UdpEndpoint localReceive_; // 本地监听地址
    UdpEndpoint controlTarget_; // 控制包目标地址
    std::shared_ptr<CriRealtimeParser> parser_; // 可选解析器
    FrameCallback onFrame_; // 解析帧回调
    RawPacketCallback onRawPacket_; // 原始包回调

    std::atomic<bool> running_ {false}; // 接收线程运行状态
    std::thread receiveThread_; // UDP接收线程
    mutable std::mutex socketMutex_; // 保护收发socket
    mutable std::mutex callbackMutex_; // 保护回调与parser绑定关系

    struct Impl;
    Impl* impl_;
};

} // namespace cri
