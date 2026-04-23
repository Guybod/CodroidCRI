#include "robot/CriRealtimeParser.h"
#include "robot/RealtimePointSender.h"
#include "robot/RobotTcpClient.h"
#include "robot/RobotUdpClient.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

// 关节控制演示：从当前关节点出发，依次运动到A点、B点，再回A点。
int main() {
    using namespace cri;

    // 角度转弧度工具，协议中的关节量统一使用弧度。
    auto degToRad = [](double degree) {
        constexpr double kPi = 3.14159265358979323846;
        return degree * kPi / 180.0;
    };

    // 1) 创建TCP/UDP/解析器对象。
    RobotTcpClient tcp({ "192.168.1.136", 9001 });
    auto parser = std::make_shared<CriRealtimeParser>();
    auto udp = std::make_shared<RobotUdpClient>(
        UdpEndpoint{"192.168.1.150", 18888},
        UdpEndpoint{"192.168.1.136", 9030});
    udp->bindParser(parser);

    // 2) 当 StartDataPush 生效后，立刻热更新解析布局（支持动态 mask）。
    tcp.setOnLayoutApplied([parser](const FrameLayoutConfig& layout) {
        parser->updateLayout(layout);
        std::cout << "[布局更新] 期望包长: " << parser->expectedPacketSize() << " bytes" << std::endl;
    });

    if (!tcp.connect()) {
        std::cerr << "TCP连接失败，请检查机器人IP/端口。" << std::endl;
        return 1;
    }

    // TCP建立后先切换模式并上使能：自动 -> 远程 -> 上使能。
    if (!tcp.toAuto()) {
        std::cerr << "切换自动模式失败（Robot/toAuto）。" << std::endl;
        return 1;
    }
    if (!tcp.toRemote()) {
        std::cerr << "切换远程模式失败（Robot/toRemote）。" << std::endl;
        return 1;
    }
    if (!tcp.switchOn()) {
        std::cerr << "上使能失败（Robot/switchOn）。" << std::endl;
        return 1;
    }

    if (!udp->startReceive()) {
        std::cerr << "UDP接收启动失败，请检查本机IP/端口绑定。" << std::endl;
        return 1;
    }

    // 3) 开启实时数据推流：至少开启关节位置(bit8)，用于打印当前关节角。
    StartDataPushRequest pushReq;
    pushReq.localPushTarget = {"192.168.1.150", 18888};
    pushReq.durationMs = 2;
    pushReq.highPrecision = true;
    pushReq.mask = static_cast<uint16_t>((1u << 0) | (1u << 1) | (1u << 2) | (1u << 8));
    if (!tcp.startDataPush(pushReq)) {
        std::cerr << "StartDataPush失败。" << std::endl;
        return 1;
    }

    // 4) 开启实时控制模式，发送周期与推流周期统一为2ms。
    StartControlRequest controlReq;
    controlReq.filterType = 1;
    controlReq.durationMs = 2;
    controlReq.startBuffer = 5;
    if (!tcp.startControl(controlReq)) {
        std::cerr << "StartControl失败。" << std::endl;
        return 1;
    }

    // 5) 启动打印线程：每秒打印一次当前关节位置（rad）。
    std::atomic<bool> printRunning {true};
    std::thread printer([&]() {
        while (printRunning.load()) {
            auto frame = parser->latestFrame();
            if (frame.has_value() && !frame->jointPos.empty()) {
                std::cout << "[关节位置rad] ";
                for (double v : frame->jointPos) {
                    std::cout << v << " ";
                }
                std::cout << std::endl;
            } else {
                std::cout << "[关节位置rad] 暂无有效数据" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });

    // 6) 主线程发送实时轨迹：
    // 当前点 -> [0,0,90,0,90,0] -> [0,0,0,0,0,0] -> [0,0,90,0,90,0]
    std::array<double, 6> currentJointRad {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    bool gotCurrentPose = false;
    for (int i = 0; i < 50; ++i) { // 最多等待约5秒获取当前关节位置
        auto frame = parser->latestFrame();
        if (frame.has_value() && frame->jointPos.size() >= 6) {
            for (size_t j = 0; j < 6; ++j) {
                currentJointRad[j] = frame->jointPos[j];
            }
            gotCurrentPose = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!gotCurrentPose) {
        std::cerr << "未获取到当前关节位置，默认以全0作为起点。" << std::endl;
    }

    Waypoint current;
    current.position = currentJointRad;
    current.segmentTimeSec = 2.0;

    Waypoint poseA;
    poseA.position = {0.0, 0.0, degToRad(90.0), 0.0, degToRad(90.0), 0.0};
    poseA.segmentTimeSec = 3.0;

    Waypoint poseB;
    poseB.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    poseB.segmentTimeSec = 3.0;

    Waypoint poseAEnd;
    poseAEnd.position = poseA.position;
    poseAEnd.segmentTimeSec = 3.0;

    // 速度规划策略：
    // - 首末点速度置0，保证起停平稳；
    // - 中间过渡点速度由相邻点差分估算，避免每段都“停-走-停”。
    current.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    poseA.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    poseB.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    poseAEnd.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    for (size_t axis = 0; axis < 6; ++axis) {
        poseA.velocity[axis] = (poseB.position[axis] - current.position[axis]) /
            (current.segmentTimeSec + poseA.segmentTimeSec);
        poseB.velocity[axis] = (poseAEnd.position[axis] - poseA.position[axis]) /
            (poseA.segmentTimeSec + poseB.segmentTimeSec);
    }

    RealtimePointSender sender(udp);
    sender.setCycleMs(2);
    sender.setStartBuffer(5);
    sender.setCommandType(0);
    if (!sender.loadTrajectory(std::vector<Waypoint>{current, poseA, poseB, poseAEnd})) {
        std::cerr << "轨迹装载失败。" << std::endl;
    } else if (!sender.startStreaming()) {
        std::cerr << "轨迹发送启动失败。" << std::endl;
    } else {
        while (sender.isStreaming()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    // 7) 轨迹发送结束后，停止线程与CRI会话。
    printRunning = false;
    if (printer.joinable()) {
        printer.join();
    }

    sender.stopStreaming();
    tcp.stopControl();
    tcp.stopDataPush(pushReq.localPushTarget);
    udp->stopReceive();
    tcp.disconnect();

    std::cout << "关节演示流程结束。" << std::endl;
    return 0;
}
