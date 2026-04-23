#include "robot/DataPushConfigStore.h"

#include <algorithm>

namespace cri {

// 设置机器人轴信息，用于后续根据轴数计算实时帧长度与字段数量。
void DataPushConfigStore::setAxisInfo(uint8_t jointCount, uint8_t extAxisCount) {
    std::lock_guard<std::mutex> lock(mutex_);
    layout_.jointCount = jointCount;
    layout_.extAxisCount = extAxisCount;
}

// 根据 StartDataPush 请求更新当前生效的 mask 与精度配置。
void DataPushConfigStore::updateFromStartDataPush(const StartDataPushRequest& req) {
    std::lock_guard<std::mutex> lock(mutex_);
    layout_.mask = req.mask;
    layout_.highPrecision = req.highPrecision;
}

// 获取当前布局配置的线程安全快照，供解析器热更新使用。
FrameLayoutConfig DataPushConfigStore::currentLayout() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return layout_;
}

} // namespace cri
