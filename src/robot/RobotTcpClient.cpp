#include "robot/RobotTcpClient.h"

#include <asio.hpp>
#include <nlohmann/json.hpp>

namespace cri {

struct RobotTcpClient::Impl {
    asio::io_context io;
    asio::ip::tcp::socket socket {io};
};

// 构造TCP客户端并初始化默认轴信息（6关节、0外部轴）。
RobotTcpClient::RobotTcpClient(TcpEndpoint endpoint)
    : endpoint_(std::move(endpoint))
    , impl_(new Impl()) {
    configStore_.setAxisInfo(6, 0);
}

// 析构时确保连接被正确关闭并释放底层实现对象。
RobotTcpClient::~RobotTcpClient() {
    disconnect();
    delete impl_;
    impl_ = nullptr;
}

// 建立到机器人指令端口的TCP连接，并切换为非阻塞模式供超时收发使用。
bool RobotTcpClient::connect(std::chrono::milliseconds timeout) {
    (void)timeout;
    std::lock_guard<std::mutex> lock(socketMutex_);
    if (impl_->socket.is_open()) {
        return true;
    }

    asio::error_code ec;
    auto address = asio::ip::make_address(endpoint_.ip, ec);
    if (ec) {
        return false;
    }

    asio::ip::tcp::endpoint endpoint(address, endpoint_.port);
    impl_->socket.open(endpoint.protocol(), ec);
    if (ec) {
        return false;
    }

    impl_->socket.connect(endpoint, ec);
    if (ec) {
        impl_->socket.close();
        return false;
    }
    impl_->socket.non_blocking(true, ec);
    if (ec) {
        impl_->socket.close();
        return false;
    }
    return true;
}

// 主动关闭TCP连接，忽略shutdown过程中的非关键错误码。
void RobotTcpClient::disconnect() {
    std::lock_guard<std::mutex> lock(socketMutex_);
    if (!impl_->socket.is_open()) {
        return;
    }
    asio::error_code ec;
    impl_->socket.shutdown(asio::ip::tcp::socket::shutdown_both, ec);
    impl_->socket.close(ec);
}

// 返回当前socket是否处于打开状态。
bool RobotTcpClient::isConnected() const {
    std::lock_guard<std::mutex> lock(socketMutex_);
    return impl_->socket.is_open();
}

// 发送一行JSON请求并在超时时间内读取一行JSON响应。
// 采用非阻塞 read/write 循环实现收发超时控制。
bool RobotTcpClient::request(
    std::string_view requestJson,
    std::string* responseJson,
    std::chrono::milliseconds timeout) {
    std::lock_guard<std::mutex> lock(socketMutex_);
    if (!impl_->socket.is_open()) {
        return false;
    }

    asio::error_code ec;
    std::string wire(requestJson);
    if (wire.empty() || wire.back() != '\n') {
        wire.push_back('\n');
    }

    std::size_t written = 0;
    auto start = std::chrono::steady_clock::now();
    while (written < wire.size() && std::chrono::steady_clock::now() - start < timeout) {
        const std::size_t n = impl_->socket.write_some(asio::buffer(wire.data() + written, wire.size() - written), ec);
        if (!ec) {
            written += n;
            continue;
        }
        if (ec == asio::error::would_block || ec == asio::error::try_again) {
            continue;
        }
        return false;
    }
    if (written != wire.size()) {
        return false;
    }

    asio::streambuf streamBuf;
    start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout) {
        std::size_t bytes = impl_->socket.read_some(streamBuf.prepare(1024), ec);
        if (!ec) {
            streamBuf.commit(bytes);
            std::istream is(&streamBuf);
            std::string line;
            if (std::getline(is, line)) {
                if (responseJson) {
                    *responseJson = line;
                }
                return true;
            }
            continue;
        }

        if (ec == asio::error::would_block || ec == asio::error::try_again) {
            continue;
        }
        return false;
    }

    return false;
}

// 下发 Robot/toAuto，请求机器人进入自动模式。
bool RobotTcpClient::toAuto(std::string* responseJson) {
    nlohmann::json body = {
        {"id", nextId_.fetch_add(1)},
        {"ty", "Robot/toAuto"},
        {"db", ""}
    };
    return sendAndValidateType(body.dump(), "Robot/toAuto", responseJson);
}

// 下发 Robot/toRemote，请求机器人进入远程模式。
// 注意：协议说明不能直接从手动模式跳到远程，建议先执行 toAuto。
bool RobotTcpClient::toRemote(std::string* responseJson) {
    nlohmann::json body = {
        {"id", nextId_.fetch_add(1)},
        {"ty", "Robot/toRemote"},
        {"db", ""}
    };
    return sendAndValidateType(body.dump(), "Robot/toRemote", responseJson);
}

// 下发 Robot/switchOn，请求机器人上使能。
bool RobotTcpClient::switchOn(std::string* responseJson) {
    nlohmann::json body = {
        {"id", nextId_.fetch_add(1)},
        {"ty", "Robot/switchOn"},
        {"db", ""}
    };
    return sendAndValidateType(body.dump(), "Robot/switchOn", responseJson);
}

// 下发 StartDataPush；成功后更新本地配置并触发布局生效回调。
bool RobotTcpClient::startDataPush(const StartDataPushRequest& req, std::string* responseJson) {
    nlohmann::json body = {
        {"id", nextId_.fetch_add(1)},
        {"ty", "CRI/StartDataPush"},
        {"db",
            {
                {"ip", req.localPushTarget.ip},
                {"port", req.localPushTarget.port},
                {"duration", req.durationMs},
                {"highPercision", req.highPrecision},
                {"mask", req.mask}
            }}
    };

    if (!sendAndValidateType(body.dump(), "CRI/StartDataPush", responseJson)) {
        return false;
    }

    configStore_.updateFromStartDataPush(req);
    if (onLayoutApplied_) {
        onLayoutApplied_(configStore_.currentLayout());
    }
    return true;
}

// 下发 StopDataPush；可选指定要停止的UDP目标端点。
bool RobotTcpClient::stopDataPush(const std::optional<UdpEndpoint>& target, std::string* responseJson) {
    nlohmann::json body = {
        {"id", nextId_.fetch_add(1)},
        {"ty", "CRI/StopDataPush"},
    };

    if (target.has_value()) {
        body["db"] = {{"ip", target->ip}, {"port", target->port}};
    }

    return sendAndValidateType(body.dump(), "CRI/StopDataPush", responseJson);
}

// 下发 StartControl，开启实时控制模式及参数。
bool RobotTcpClient::startControl(const StartControlRequest& req, std::string* responseJson) {
    nlohmann::json body = {
        {"id", nextId_.fetch_add(1)},
        {"ty", "CRI/StartControl"},
        {"db",
            {
                {"filterType", req.filterType},
                {"duration", req.durationMs},
                {"startBuffer", req.startBuffer}
            }}
    };

    return sendAndValidateType(body.dump(), "CRI/StartControl", responseJson);
}

// 下发 StopControl，关闭实时控制模式。
bool RobotTcpClient::stopControl(std::string* responseJson) {
    nlohmann::json body = {
        {"id", nextId_.fetch_add(1)},
        {"ty", "CRI/StopControl"},
    };

    return sendAndValidateType(body.dump(), "CRI/StopControl", responseJson);
}

// 设置 StartDataPush 成功后的布局生效通知回调。
void RobotTcpClient::setOnLayoutApplied(std::function<void(const FrameLayoutConfig&)> callback) {
    onLayoutApplied_ = std::move(callback);
}

// 返回可写配置存储对象，供上层做定制配置管理。
DataPushConfigStore& RobotTcpClient::configStore() {
    return configStore_;
}

// 返回只读配置存储对象。
const DataPushConfigStore& RobotTcpClient::configStore() const {
    return configStore_;
}

// 发送请求并校验响应中的 ty 字段是否与预期一致。
bool RobotTcpClient::sendAndValidateType(
    const std::string& requestJson,
    const char* expectedType,
    std::string* responseJson) {
    std::string response;
    if (!request(requestJson, &response)) {
        return false;
    }

    nlohmann::json parsed;
    try {
        parsed = nlohmann::json::parse(response);
    } catch (...) {
        return false;
    }

    if (!parsed.contains("ty") || !parsed["ty"].is_string()) {
        return false;
    }
    if (parsed["ty"].get<std::string>() != expectedType) {
        return false;
    }

    if (responseJson) {
        *responseJson = response;
    }
    return true;
}

} // namespace cri
