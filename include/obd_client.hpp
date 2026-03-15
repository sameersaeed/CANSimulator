#pragma once

#include <optional>
#include <string>

#include "can_socket.hpp"
#include "pid_handler.hpp"

struct QueryResult {
    std::string name;
    double      value{0.0};
    std::string unit;
    int64_t     latency_us{0};
};

// sends Mode 01 request frames (CAN ID 0x7DF) and waits for ECU response (0x7E8)
class OBDClient {
public:
    explicit OBDClient(const std::string& iface = "vcan0");

    std::optional<QueryResult> query(PID pid, int timeout_ms = 500);
    void benchmark(PID pid, int count);

private:
    CANSocket m_socket;
};
