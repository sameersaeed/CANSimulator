#pragma once

#include <linux/can.h>
#include <optional>
#include <string>

// wraps around raw CAN socket bound to a CAN interface (for example vcan0)
class CANSocket {
public:
    explicit CANSocket(const std::string& interface);
    ~CANSocket();

    // non-copyable
    CANSocket(const CANSocket&)            = delete;
    CANSocket& operator=(const CANSocket&) = delete;

    bool sendFrame(const can_frame& frame);
    bool setFilter(canid_t id, canid_t mask);

    std::optional<can_frame> receiveFrame(double timeoutS = 0.2);

    int  fd()     const { return m_fd; }
    bool isOpen() const { return m_fd >= 0; }

private:
    int m_fd{-1};
    std::string m_interface;
};
