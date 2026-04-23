#include "robot/CriRealtimeParser.h"

#include <algorithm>
#include <cstring>

namespace cri {

namespace {

// 对任意标量执行字节翻转，用于大小端转换。
template <typename T>
T byteSwap(T value) {
    T out {};
    auto* src = reinterpret_cast<const uint8_t*>(&value);
    auto* dst = reinterpret_cast<uint8_t*>(&out);
    for (size_t i = 0; i < sizeof(T); ++i) {
        dst[i] = src[sizeof(T) - 1 - i];
    }
    return out;
}

// 判断当前主机字节序是否为小端。
bool hostLittleEndian() {
    static const uint16_t marker = 0x1;
    return *reinterpret_cast<const uint8_t*>(&marker) == 0x1;
}

} // namespace

// 构造解析器并按给定布局初始化内部运行时参数。
CriRealtimeParser::CriRealtimeParser(const FrameLayoutConfig& layout) {
    updateLayout(layout);
}

// 热更新解析布局：重算标量大小、TCP维度、期望包长并清理失配状态。
void CriRealtimeParser::updateLayout(const FrameLayoutConfig& layout) {
    std::lock_guard<std::mutex> lock(mutex_);
    runtime_.cfg = layout;
    runtime_.scalarSize = layout.highPrecision ? sizeof(double) : sizeof(float);
    runtime_.tcpPoseDim = (layout.jointCount == 7) ? 7 : 6;
    runtime_.expectedSize = computeExpectedSize(layout);
    stats_.continuousMismatch = 0;
    resyncNeeded_ = false;
}

// 获取当前解析布局快照。
FrameLayoutConfig CriRealtimeParser::layout() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return runtime_.cfg;
}

// 返回当前布局下期望的UDP数据包长度。
size_t CriRealtimeParser::expectedPacketSize() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return runtime_.expectedSize;
}

// 按当前布局解析一帧UDP载荷，并在成功时更新 latestFrame 缓存。
CriRealtimeParser::ParseResult CriRealtimeParser::parse(const uint8_t* data, size_t size) {
    if (!data || size == 0) {
        return ParseResult::DecodeError;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    stats_.totalPackets++;
    if (runtime_.expectedSize == 0) {
        stats_.droppedByDecodeError++;
        return ParseResult::LayoutNotReady;
    }

    if (size != runtime_.expectedSize) {
        stats_.droppedBySizeMismatch++;
        stats_.continuousMismatch++;
        if (stats_.continuousMismatch >= mismatchThreshold_) {
            resyncNeeded_ = true;
        }
        return ParseResult::SizeMismatch;
    }

    const uint8_t* cursor = data;
    const uint8_t* end = data + size;
    RealtimeFrame frame;
    const auto& cfg = runtime_.cfg;
    const uint16_t mask = cfg.mask;

    auto resetMismatch = [&]() {
        stats_.continuousMismatch = 0;
        resyncNeeded_ = false;
    };

    if (mask & (1u << 0)) {
        if (!readScalar(cursor, end, cfg.littleEndian, &frame.timestampMs)) {
            stats_.droppedByDecodeError++;
            return ParseResult::DecodeError;
        }
    }

    if (mask & (1u << 1)) {
        uint16_t raw = 0;
        if (!readScalar(cursor, end, cfg.littleEndian, &raw)) {
            stats_.droppedByDecodeError++;
            return ParseResult::DecodeError;
        }
        frame.status1 = decodeStatusData1(raw);
    }

    if (mask & (1u << 2)) {
        uint16_t raw = 0;
        if (!readScalar(cursor, end, cfg.littleEndian, &raw)) {
            stats_.droppedByDecodeError++;
            return ParseResult::DecodeError;
        }
        frame.status2 = decodeStatusData2(raw);
    }

    if (mask & (1u << 8)) {
        if (!readVector(cursor, end, cfg.jointCount, &frame.jointPos)) {
            stats_.droppedByDecodeError++;
            return ParseResult::DecodeError;
        }
    }

    if (mask & (1u << 9)) {
        if (!readVector(cursor, end, cfg.jointCount, &frame.jointVel)) {
            stats_.droppedByDecodeError++;
            return ParseResult::DecodeError;
        }
    }

    if (mask & (1u << 10)) {
        if (!readVector(cursor, end, runtime_.tcpPoseDim, &frame.tcpPose)) {
            stats_.droppedByDecodeError++;
            return ParseResult::DecodeError;
        }
    }

    if (mask & (1u << 11)) {
        if (!readVector(cursor, end, 6, &frame.tcpVel)) {
            stats_.droppedByDecodeError++;
            return ParseResult::DecodeError;
        }
    }

    if (mask & (1u << 12)) {
        std::vector<double> scalar;
        if (!readVector(cursor, end, 1, &scalar)) {
            stats_.droppedByDecodeError++;
            return ParseResult::DecodeError;
        }
        frame.tcpLinearSpeed = scalar[0];
    }

    if (mask & (1u << 13)) {
        if (!readVector(cursor, end, cfg.jointCount, &frame.jointTorqueOut)) {
            stats_.droppedByDecodeError++;
            return ParseResult::DecodeError;
        }
    }

    if (mask & (1u << 14)) {
        if (!readVector(cursor, end, cfg.jointCount, &frame.jointExternalTorque)) {
            stats_.droppedByDecodeError++;
            return ParseResult::DecodeError;
        }
    }

    if (mask & (1u << 15)) {
        if (!readVector(cursor, end, cfg.extAxisCount, &frame.extAxisPos)) {
            stats_.droppedByDecodeError++;
            return ParseResult::DecodeError;
        }
    }

    if (cursor != end) {
        stats_.droppedByDecodeError++;
        return ParseResult::DecodeError;
    }

    latestFrame_ = std::move(frame);
    stats_.parsedPackets++;
    resetMismatch();
    return ParseResult::Ok;
}

// 线程安全地读取最近一次成功解析的实时帧。
std::optional<RealtimeFrame> CriRealtimeParser::latestFrame() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latestFrame_;
}

// 返回解析统计信息快照。
CriRealtimeParser::ParseStats CriRealtimeParser::stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}

// 设置连续长度失配阈值，最小强制为1。
void CriRealtimeParser::setMismatchThreshold(uint32_t threshold) {
    std::lock_guard<std::mutex> lock(mutex_);
    mismatchThreshold_ = std::max<uint32_t>(1, threshold);
}

// 查询解析器当前是否处于“需要重同步”状态。
bool CriRealtimeParser::isResyncNeeded() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return resyncNeeded_;
}

// 清除失配计数与重同步状态标志。
void CriRealtimeParser::clearResyncState() {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.continuousMismatch = 0;
    resyncNeeded_ = false;
}

// 根据 mask/精度/轴数计算当前布局对应的帧总字节数。
size_t CriRealtimeParser::computeExpectedSize(const FrameLayoutConfig& cfg) {
    size_t size = 0;
    const size_t scalar = cfg.highPrecision ? sizeof(double) : sizeof(float);
    const size_t tcpPoseDim = (cfg.jointCount == 7) ? 7 : 6;

    if (cfg.mask & (1u << 0)) {
        size += sizeof(int64_t);
    }
    if (cfg.mask & (1u << 1)) {
        size += sizeof(uint16_t);
    }
    if (cfg.mask & (1u << 2)) {
        size += sizeof(uint16_t);
    }
    if (cfg.mask & (1u << 8)) {
        size += scalar * cfg.jointCount;
    }
    if (cfg.mask & (1u << 9)) {
        size += scalar * cfg.jointCount;
    }
    if (cfg.mask & (1u << 10)) {
        size += scalar * tcpPoseDim;
    }
    if (cfg.mask & (1u << 11)) {
        size += scalar * 6;
    }
    if (cfg.mask & (1u << 12)) {
        size += scalar;
    }
    if (cfg.mask & (1u << 13)) {
        size += scalar * cfg.jointCount;
    }
    if (cfg.mask & (1u << 14)) {
        size += scalar * cfg.jointCount;
    }
    if (cfg.mask & (1u << 15)) {
        size += scalar * cfg.extAxisCount;
    }
    return size;
}

// 解析状态数据1（UInt16）到语义化布尔位字段。
StatusData1 CriRealtimeParser::decodeStatusData1(uint16_t raw) {
    StatusData1 out;
    out.projectRunning = (raw & (1u << 0)) != 0;
    out.projectStopped = (raw & (1u << 1)) != 0;
    out.projectPaused = (raw & (1u << 2)) != 0;
    out.enabling = (raw & (1u << 3)) != 0;
    out.disabled = (raw & (1u << 4)) != 0;
    out.manualMode = (raw & (1u << 5)) != 0;
    out.dragging = (raw & (1u << 6)) != 0;
    out.moving = (raw & (1u << 7)) != 0;

    out.collisionStop = (raw & (1u << 8)) != 0;
    out.inSafePosition = (raw & (1u << 9)) != 0;
    out.hasAlarm = (raw & (1u << 10)) != 0;
    out.simulationMode = (raw & (1u << 11)) != 0;
    out.estopPressed = (raw & (1u << 12)) != 0;
    out.rescueMode = (raw & (1u << 13)) != 0;
    out.autoMode = (raw & (1u << 14)) != 0;
    out.remoteMode = (raw & (1u << 15)) != 0;
    return out;
}

// 解析状态数据2（UInt16）：高8位错误码 + 低8位模式位。
StatusData2 CriRealtimeParser::decodeStatusData2(uint16_t raw) {
    StatusData2 out;
    out.realtimeErrorCode = static_cast<uint8_t>((raw >> 8) & 0xFF);
    out.realtimeControlMode = (raw & 0x01u) != 0;
    return out;
}

// 从二进制游标读取一个标量值，并按目标字节序转换。
template <typename T>
bool CriRealtimeParser::readScalar(const uint8_t*& cursor, const uint8_t* end, bool littleEndian, T* out) {
    if (!out || !cursor || cursor + sizeof(T) > end) {
        return false;
    }
    std::memcpy(out, cursor, sizeof(T));
    cursor += sizeof(T);

    // 统一字节序，保证后续逻辑始终使用主机字节序值。
    if (littleEndian != hostLittleEndian()) {
        *out = byteSwap(*out);
    }
    return true;
}

// 读取 count 个浮点标量到 double 向量，自动适配 float/double 两种精度。
bool CriRealtimeParser::readVector(
    const uint8_t*& cursor, const uint8_t* end, size_t count, std::vector<double>* out) const {
    if (!out) {
        return false;
    }

    out->clear();
    out->reserve(count);

    if (runtime_.cfg.highPrecision) {
        for (size_t i = 0; i < count; ++i) {
            double value = 0.0;
            if (!readScalar(cursor, end, runtime_.cfg.littleEndian, &value)) {
                return false;
            }
            out->push_back(value);
        }
        return true;
    }

    for (size_t i = 0; i < count; ++i) {
        float value = 0.0f;
        if (!readScalar(cursor, end, runtime_.cfg.littleEndian, &value)) {
            return false;
        }
        out->push_back(static_cast<double>(value));
    }
    return true;
}

} // namespace cri
