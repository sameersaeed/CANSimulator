#pragma once

#include <linux/can.h>
#include <optional>
#include <string_view>

namespace CAN {

// wraps around raw CAN socket bound to a CAN interface (for example vcan0)
class Socket {
public:
    explicit Socket(const std::string_view interface);
    ~Socket();

    // non-copyable
    Socket(const Socket&)            = delete;
    Socket& operator=(const Socket&) = delete;

    bool sendFrame(const can_frame& frame);
    bool setFilter(canid_t id, canid_t mask);

    std::optional<can_frame> receiveFrame(double timeoutS = 0.2);

    int  fd()     const { return m_fd; }
    bool isOpen() const { return m_fd >= 0; }

private:
    int m_fd{-1};
    std::string_view m_interface;
};

} // namespace CAN