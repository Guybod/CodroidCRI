#pragma once

#include "robot/DataPushConfigStore.h"

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>

namespace cri {

class RobotTcpClient {
public:
    explicit RobotTcpClient(TcpEndpoint endpoint = {});
    ~RobotTcpClient();

    // 连接机器人TCP指令通道（默认 192.168.1.136:9001）。
    // timeout 用于 request 的收发超时控制；connect 本身仅做快速建立。
    bool connect(std::chrono::milliseconds timeout = std::chrono::milliseconds(1500));
    void disconnect();
    bool isConnected() const;

    // 发送一行JSON并等待一行响应JSON（换行分隔协议）。
    // 返回true表示收发成功且读到完整一行响应。
    bool request(std::string_view requestJson, std::string* responseJson,
        std::chrono::milliseconds timeout = std::chrono::milliseconds(1500));

    // 以下为CRI协议语义化接口封装。
    // 模式与使能控制接口：
    // 建议顺序：toAuto -> toRemote -> switchOn。
    bool toAuto(std::string* responseJson = nullptr);   // Robot/toAuto
    bool toRemote(std::string* responseJson = nullptr); // Robot/toRemote
    bool switchOn(std::string* responseJson = nullptr); // Robot/switchOn

    // 实时推流与控制接口：
    bool startDataPush(const StartDataPushRequest& req, std::string* responseJson = nullptr);
    bool stopDataPush(const std::optional<UdpEndpoint>& target, std::string* responseJson = nullptr);
    bool startControl(const StartControlRequest& req, std::string* responseJson = nullptr);
    bool stopControl(std::string* responseJson = nullptr);

    // 设置“推流配置生效”回调。
    // 当 StartDataPush 成功后会回调最新布局，可用于触发解析器热更新。
    void setOnLayoutApplied(std::function<void(const FrameLayoutConfig&)> callback);

    // 访问当前推流配置缓存。
    // 常见用途：在业务层读取当前mask/精度/轴配置，或手工调整轴信息。
    DataPushConfigStore& configStore();
    const DataPushConfigStore& configStore() const;

private:
    // 统一发送请求并校验响应ty字段，减少各协议接口的重复代码。
    bool sendAndValidateType(const std::string& requestJson, const char* expectedType, std::string* responseJson);

    TcpEndpoint endpoint_; // 机器人TCP控制端点
    mutable std::mutex socketMutex_; // 保护socket串行读写
    std::function<void(const FrameLayoutConfig&)> onLayoutApplied_; // StartDataPush成功后的布局回调
    DataPushConfigStore configStore_; // 本地缓存的推流布局配置
    std::atomic<uint32_t> nextId_ {1}; // 请求ID自增计数器
    struct Impl;
    Impl* impl_;
};

} // namespace cri
