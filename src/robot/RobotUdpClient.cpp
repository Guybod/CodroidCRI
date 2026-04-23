#include "robot/RobotUdpClient.h"

#include <asio.hpp>

#include <array>
#include <cstring>

namespace cri {

struct RobotUdpClient::Impl {
    asio::io_context io;
    asio::ip::udp::socket recvSocket {io};
    asio::ip::udp::socket sendSocket {io};
};

namespace {

// 以小端字节序将任意标量追加到输出缓冲区。
template <typename T>
void appendBytesLE(std::vector<uint8_t>* out, const T& value) {
    const auto* bytes = reinterpret_cast<const uint8_t*>(&value);
    out->insert(out->end(), bytes, bytes + sizeof(T));
}

// 将字符串IP与端口组装为 asio UDP 端点。
asio::ip::udp::endpoint makeUdpEndpoint(const UdpEndpoint& endpoint, asio::error_code* ec) {
    asio::ip::address address = asio::ip::make_address(endpoint.ip, *ec);
    if (*ec) {
        return {};
    }
    return asio::ip::udp::endpoint(address, endpoint.port);
}

} // namespace

// 构造UDP客户端：一个端口用于接收推流，一个目标用于发送控制指令。
RobotUdpClient::RobotUdpClient(UdpEndpoint localReceive, UdpEndpoint robotControlTarget)
    : localReceive_(std::move(localReceive))
    , controlTarget_(std::move(robotControlTarget))
    , impl_(new Impl()) {}

// 析构时停止接收线程并关闭收发socket。
RobotUdpClient::~RobotUdpClient() {
    stopReceive();
    std::lock_guard<std::mutex> lock(socketMutex_);
    asio::error_code ec;
    impl_->recvSocket.close(ec);
    impl_->sendSocket.close(ec);
    delete impl_;
    impl_ = nullptr;
}

// 启动UDP接收线程并初始化收发socket。
bool RobotUdpClient::startReceive() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return true;
    }

    asio::error_code ec;
    auto bindEndpoint = makeUdpEndpoint(localReceive_, &ec);
    if (ec) {
        running_ = false;
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(socketMutex_);
        if (!impl_->recvSocket.is_open()) {
            impl_->recvSocket.open(bindEndpoint.protocol(), ec);
            if (ec) {
                running_ = false;
                return false;
            }
            impl_->recvSocket.bind(bindEndpoint, ec);
            if (ec) {
                running_ = false;
                return false;
            }
        }

        if (!impl_->sendSocket.is_open()) {
            impl_->sendSocket.open(asio::ip::udp::v4(), ec);
            if (ec) {
                running_ = false;
                return false;
            }
        }
    }

    receiveThread_ = std::thread(&RobotUdpClient::receiveLoop, this);
    return true;
}

// 停止UDP接收线程并关闭接收socket。
void RobotUdpClient::stopReceive() {
    bool expected = true;
    if (!running_.compare_exchange_strong(expected, false)) {
        return;
    }
    asio::error_code ec;
    impl_->recvSocket.cancel(ec);
    impl_->recvSocket.close(ec);
    if (receiveThread_.joinable()) {
        receiveThread_.join();
    }
}

// 返回当前接收线程是否处于运行状态。
bool RobotUdpClient::isReceiving() const {
    return running_.load();
}

// 将 CommandData 序列化后发送到实时控制目标端点。
bool RobotUdpClient::sendCommand(const CommandData& cmd) {
    const auto payload = serializeCommand(cmd);
    return sendRaw(payload.data(), payload.size(), controlTarget_);
}

// 发送任意UDP二进制数据到指定目标端点。
bool RobotUdpClient::sendRaw(const uint8_t* data, size_t size, const UdpEndpoint& target) {
    if (!data || size == 0) {
        return false;
    }

    asio::error_code ec;
    auto endpoint = makeUdpEndpoint(target, &ec);
    if (ec) {
        return false;
    }

    std::lock_guard<std::mutex> lock(socketMutex_);
    if (!impl_->sendSocket.is_open()) {
        impl_->sendSocket.open(asio::ip::udp::v4(), ec);
        if (ec) {
            return false;
        }
    }

    impl_->sendSocket.send_to(asio::buffer(data, size), endpoint, 0, ec);
    return !ec;
}

// 绑定实时帧解析器，供接收循环自动解析载荷。
void RobotUdpClient::bindParser(std::shared_ptr<CriRealtimeParser> parser) {
    std::lock_guard<std::mutex> lock(callbackMutex_);
    parser_ = std::move(parser);
}

// 设置解析成功后的帧回调。
void RobotUdpClient::setOnFrame(FrameCallback callback) {
    std::lock_guard<std::mutex> lock(callbackMutex_);
    onFrame_ = std::move(callback);
}

// 设置原始UDP包回调（可用于抓包/调试）。
void RobotUdpClient::setOnRawPacket(RawPacketCallback callback) {
    std::lock_guard<std::mutex> lock(callbackMutex_);
    onRawPacket_ = std::move(callback);
}

// UDP接收循环：读取数据 -> 回调原始包 -> 调用解析器 -> 回调解析帧。
void RobotUdpClient::receiveLoop() {
    std::array<uint8_t, 4096> buffer {};
    asio::ip::udp::endpoint sender;

    while (running_.load()) {
        asio::error_code ec;
        if (!impl_->recvSocket.is_open()) {
            break;
        }
        std::size_t received = impl_->recvSocket.receive_from(asio::buffer(buffer), sender, 0, ec);

        if (ec) {
            if (!running_.load()) {
                break;
            }
            continue;
        }

        std::shared_ptr<CriRealtimeParser> parser;
        FrameCallback frameCb;
        RawPacketCallback rawCb;
        {
            std::lock_guard<std::mutex> lock(callbackMutex_);
            parser = parser_;
            frameCb = onFrame_;
            rawCb = onRawPacket_;
        }

        if (rawCb) {
            rawCb(buffer.data(), received);
        }

        if (!parser) {
            continue;
        }
        if (parser->parse(buffer.data(), received) == CriRealtimeParser::ParseResult::Ok) {
            auto latest = parser->latestFrame();
            if (latest.has_value() && frameCb) {
                frameCb(latest.value());
            }
        }
    }
}

// 按协议定义序列化 CommandData，生成可直接发送的UDP负载。
std::vector<uint8_t> RobotUdpClient::serializeCommand(const CommandData& cmd) {
    std::vector<uint8_t> bytes;
    bytes.reserve(sizeof(int64_t) + sizeof(double) * 6 + sizeof(uint8_t) + 7);

    appendBytesLE(&bytes, cmd.timestamp);
    for (double v : cmd.position) {
        appendBytesLE(&bytes, v);
    }
    appendBytesLE(&bytes, cmd.type);
    for (uint8_t v : cmd.reserved) {
        appendBytesLE(&bytes, v);
    }
    return bytes;
}

} // namespace cri
