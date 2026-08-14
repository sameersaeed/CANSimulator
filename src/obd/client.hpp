#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "../can/socket.hpp"
#include "../obd/pid_handler.hpp"

namespace OBD {

struct QueryResult {
    std::string name;
    double      value{0.0};
    std::string unit;
    int64_t     latencyUs{0};
};

// sends Mode 01 request frames (CAN ID 0x7DF) and waits for ECU response (0x7E8)
class Client {
public:
    explicit Client(const std::string& iface = "vcan0");

    std::optional<QueryResult> query(PID::Type pid, double timeoutS = 0.5); // timeout in seconds
    void benchmark(PID::Type pid, int count);

private:
    CAN::Socket m_socket;
};

} // namespace OBD