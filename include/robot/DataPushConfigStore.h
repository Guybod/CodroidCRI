#pragma once

#include "robot/model/CriTypes.h"

#include <mutex>

namespace cri {

// 当前推流配置的集中存储。
// 建议流程：TCP下发 StartDataPush 成功后先更新本类，再通知解析器热更新。
class DataPushConfigStore {
public:
    // 设置轴配置（通常在已知机器人型号后调用一次）。
    void setAxisInfo(uint8_t jointCount, uint8_t extAxisCount);
    // 根据最近一次 StartDataPush 请求同步 mask/精度。
    void updateFromStartDataPush(const StartDataPushRequest& req);
    // 线程安全返回当前布局快照。
    FrameLayoutConfig currentLayout() const;

private:
    mutable std::mutex mutex_;
    FrameLayoutConfig layout_ {}; // 当前生效或准备生效的解析布局
};

} // namespace cri
