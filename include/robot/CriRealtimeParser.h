#pragma once

#include "robot/model/CriTypes.h"

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>

namespace cri {

class CriRealtimeParser {
public:
    // UDP载荷解析结果。
    enum class ParseResult {
        Ok,             // 成功解析并更新 latestFrame
        LayoutNotReady, // 布局未初始化（expectedSize=0）
        SizeMismatch,   // 收到包长与布局期望长度不一致
        DecodeError     // 字段解码失败（越界/字节序/剩余字节异常）
    };

    // 累计解析统计，可用于监控丢包与同步健康度。
    struct ParseStats {
        uint64_t totalPackets {0};            // 总收到并尝试解析的数据包数
        uint64_t parsedPackets {0};           // 成功解析的数据包数
        uint64_t droppedBySizeMismatch {0};   // 因包长不匹配被丢弃
        uint64_t droppedByDecodeError {0};    // 因字段解码失败被丢弃
        uint32_t continuousMismatch {0};      // 当前连续包长失配计数
    };

    explicit CriRealtimeParser(const FrameLayoutConfig& layout = {});

    // 热更新解析布局。
    // 推荐在TCP StartDataPush成功后立即调用，实现mask动态切换。
    void updateLayout(const FrameLayoutConfig& layout);
    FrameLayoutConfig layout() const; // 获取当前生效布局快照
    size_t expectedPacketSize() const; // 获取当前布局的期望UDP包长（字节）

    // 按当前布局解析一个UDP载荷。
    // 解析成功后会原子替换 latestFrame 缓存，供业务线程随时读取。
    ParseResult parse(const uint8_t* data, size_t size);

    // 读取最近一次成功解析的实时帧。
    std::optional<RealtimeFrame> latestFrame() const; // 最近一次成功解析帧（线程安全快照）
    ParseStats stats() const; // 当前统计信息快照

    // 包长连续不匹配达到阈值后，解析器进入“需要重同步”状态。
    void setMismatchThreshold(uint32_t threshold);
    bool isResyncNeeded() const;
    void clearResyncState();

private:
    struct LayoutRuntime {
        FrameLayoutConfig cfg {};
        size_t scalarSize {8};
        size_t tcpPoseDim {6};
        size_t expectedSize {0};
    };

    static size_t computeExpectedSize(const FrameLayoutConfig& cfg);
    static StatusData1 decodeStatusData1(uint16_t raw);
    static StatusData2 decodeStatusData2(uint16_t raw);

    template <typename T>
    static bool readScalar(const uint8_t*& cursor, const uint8_t* end, bool littleEndian, T* out);

    bool readVector(const uint8_t*& cursor, const uint8_t* end, size_t count, std::vector<double>* out) const;

    mutable std::mutex mutex_;
    LayoutRuntime runtime_ {};
    std::optional<RealtimeFrame> latestFrame_ {};
    ParseStats stats_ {};
    uint32_t mismatchThreshold_ {5};
    bool resyncNeeded_ {false};
};

} // namespace cri
