#include "robot/CriRealtimeParser.h"
#include "robot/RealtimePointSender.h"
#include "robot/RobotTcpClient.h"
#include "robot/RobotUdpClient.h"

#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

// 笛卡尔控制演示：从当前末端位姿出发，Z轴上抬200mm，再回到原始位姿。
int main() {
    using namespace cri;

    // 1) 创建TCP/UDP/解析器对象。
    RobotTcpClient tcp({ "192.168.1.136", 9001 });
    auto parser = std::make_shared<CriRealtimeParser>();
    auto udp = std::make_shared<RobotUdpClient>(
        UdpEndpoint{"192.168.1.150", 18888},
        UdpEndpoint{"192.168.1.136", 9030});
    udp->bindParser(parser);

    // 2) 推流配置生效后同步更新解析布局。
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

    // 3) 开启实时推流：需要末端位置(bit10)用于获取当前笛卡尔位姿。
    StartDataPushRequest pushReq;
    pushReq.localPushTarget = {"192.168.1.150", 18888};
    pushReq.durationMs = 2;
    pushReq.highPrecision = true;
    pushReq.mask = static_cast<uint16_t>((1u << 0) | (1u << 1) | (1u << 2) | (1u << 10));
    if (!tcp.startDataPush(pushReq)) {
        std::cerr << "StartDataPush失败。" << std::endl;
        return 1;
    }

    // 4) 开启实时控制，后续按笛卡尔点位下发（type=1）。
    StartControlRequest controlReq;
    controlReq.filterType = 1;
    controlReq.durationMs = 2;
    controlReq.startBuffer = 5;
    if (!tcp.startControl(controlReq)) {
        std::cerr << "StartControl失败。" << std::endl;
        return 1;
    }

    // 5) 打印线程：每秒输出一次当前末端位姿（x,y,z,rx,ry,rz）。
    std::atomic<bool> printRunning {true};
    std::thread printer([&]() {
        while (printRunning.load()) {
            auto frame = parser->latestFrame();
            if (frame.has_value() && frame->tcpPose.size() >= 6) {
                std::cout << "[末端位姿] ";
                for (size_t i = 0; i < 6; ++i) {
                    std::cout << frame->tcpPose[i] << " ";
                }
                std::cout << std::endl;
            } else {
                std::cout << "[末端位姿] 暂无有效数据" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });

    // 6) 等待当前末端位姿可用，作为轨迹起点。
    std::array<double, 6> currentPose {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    bool gotCurrentPose = false;
    for (int i = 0; i < 50; ++i) { // 最多等待约5秒
        auto frame = parser->latestFrame();
        if (frame.has_value() && frame->tcpPose.size() >= 6) {
            for (size_t j = 0; j < 6; ++j) {
                currentPose[j] = frame->tcpPose[j];
            }
            gotCurrentPose = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!gotCurrentPose) {
        std::cerr << "未获取到当前末端位姿，默认使用全0位姿作为起点。" << std::endl;
    }

    // 7) 构造笛卡尔轨迹：当前位姿 -> Z上抬0.2m -> 回原位。
    Waypoint start;
    start.position = currentPose;
    start.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    start.segmentTimeSec = 2.0;

    Waypoint zUp;
    zUp.position = currentPose;
    zUp.position[2] += 0.2; // Z轴上抬200mm（0.2m）
    zUp.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    zUp.segmentTimeSec = 2.0;

    Waypoint back;
    back.position = currentPose;
    back.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    back.segmentTimeSec = 2.0;

    RealtimePointSender sender(udp);
    sender.setCycleMs(2);
    sender.setStartBuffer(5);
    sender.setCommandType(1); // 末端控制模式
    if (!sender.loadTrajectory(std::vector<Waypoint>{start, zUp, back})) {
        std::cerr << "笛卡尔轨迹装载失败。" << std::endl;
    } else if (!sender.startStreaming()) {
        std::cerr << "笛卡尔轨迹发送启动失败。" << std::endl;
    } else {
        while (sender.isStreaming()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    // 8) 收尾：停止线程并关闭CRI会话。
    printRunning = false;
    if (printer.joinable()) {
        printer.join();
    }

    sender.stopStreaming();
    tcp.stopControl();
    tcp.stopDataPush(pushReq.localPushTarget);
    udp->stopReceive();
    tcp.disconnect();

    std::cout << "笛卡尔抬升演示结束。" << std::endl;
    return 0;
}
