#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace cri {

// TCP指令通道地址（用于下发JSON控制指令）。
struct TcpEndpoint {
    std::string ip {"192.168.1.136"};
    uint16_t port {9001};
};

// UDP端点定义（既可用于本地监听，也可用于远端发送目标）。
struct UdpEndpoint {
    std::string ip;
    uint16_t port {0};
};

// UDP实时帧解析配置。
// 注意：该配置必须与TCP下发的 StartDataPush 参数保持一致，
// 是解析器计算帧长度与字段偏移的唯一依据。
struct FrameLayoutConfig {
    uint16_t mask {0xFFFF};        // 推流字段掩码，bit位含义遵循CRI协议定义
    bool highPrecision {true};     // true=double(8字节), false=float(4字节)
    uint8_t jointCount {6};        // 机器人关节轴数（常见6或7）
    uint8_t extAxisCount {0};      // 外部轴数量（导轨/变位机等）
    bool littleEndian {true};      // 推流字节序，解析器据此做主机端转换
};

// CRI/StartDataPush 请求参数。
struct StartDataPushRequest {
    UdpEndpoint localPushTarget {"192.168.1.150", 18888}; // 机器人向该地址推送UDP实时帧
    uint16_t durationMs {2};                               // 推流周期，单位ms，范围[1,1000]
    bool highPrecision {true};                             // 推流精度：double/float
    uint16_t mask {0xFFFF};                                // 推流字段mask
};

// CRI/StartControl 请求参数。
struct StartControlRequest {
    uint8_t filterType {0};       // 滤波类型：0..3
    uint8_t durationMs {1};       // 指令间隔，单位ms，范围[1,16]
    uint8_t startBuffer {3};      // 启动缓冲点数，范围[1,100]
};

// 实时状态字1（由协议中的16位状态位展开而来）。
struct StatusData1 {
    bool projectRunning {false};
    bool projectStopped {false};
    bool projectPaused {false};
    bool enabling {false};
    bool disabled {false};
    bool manualMode {false};
    bool dragging {false};
    bool moving {false};

    bool collisionStop {false};
    bool inSafePosition {false};
    bool hasAlarm {false};
    bool simulationMode {false};
    bool estopPressed {false};
    bool rescueMode {false};
    bool autoMode {false};
    bool remoteMode {false};
};

// 实时状态字2（错误码/模式位）。
struct StatusData2 {
    uint8_t realtimeErrorCode {0}; // 高8位实时控制错误码
    bool realtimeControlMode {false}; // 低8位bit0：是否实时控制模式
};

// 解析后的单帧实时数据。
// 各字段是否有值由 mask 决定：未开启的字段保持默认值或为空容器。
struct RealtimeFrame {
    int64_t timestampMs {0};   // 控制器时间戳（ms）
    StatusData1 status1 {};    // 状态字1拆分结果
    StatusData2 status2 {};    // 状态字2拆分结果

    std::vector<double> jointPos;             // 关节位置
    std::vector<double> jointVel;             // 关节速度
    std::vector<double> tcpPose; // 末端位姿：6轴机器人6维，7轴机器人可含第7维
    std::vector<double> tcpVel;  // 末端速度：协议定义为6维
    std::optional<double> tcpLinearSpeed;     // TCP线速度标量
    std::vector<double> jointTorqueOut;       // 关节输出扭矩
    std::vector<double> jointExternalTorque;  // 关节外部扭矩估计
    std::vector<double> extAxisPos;           // 外部轴位置
};

// 发送到实时控制UDP端口（默认9030）的控制包。
// 字节布局严格遵循协议定义的 CommandData 结构。
struct CommandData {
    int64_t timestamp {0};              // 指令时间戳（ms）
    std::array<double, 6> position {};  // 目标点（关节角或末端位姿前6维）
    uint8_t type {0};                   // 0：关节空间，1：末端空间
    std::array<uint8_t, 7> reserved {}; // 协议保留字节，默认置0
};

// 轨迹离散点定义（输入给 RealtimePointSender）。
struct Waypoint {
    std::array<double, 6> position {};  // 位置（关节模式下为6轴角度；笛卡尔模式下为位姿分量）
    std::array<double, 6> velocity {};  // 对应维度的目标速度（用于三次插值端点约束）
    double segmentTimeSec {0.02};       // 本点到下一点的期望段时长（秒）
};

} // namespace cri
