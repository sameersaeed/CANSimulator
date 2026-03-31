#pragma once

#include <optional>
#include <string>

#include "CANSocket.hpp"
#include "PIDHandler.hpp"

struct QueryResult {
    std::string name;
    double      value{0.0};
    std::string unit;
    int64_t     latencyUs{0};
};

// sends Mode 01 request frames (CAN ID 0x7DF) and waits for ECU response (0x7E8)
class OBDClient {
public:
    explicit OBDClient(const std::string& iface = "vcan0");

    std::optional<QueryResult> query(PID pid, double timeoutS = 0.5); // timeout in seconds
    void benchmark(PID pid, int count);

private:
    CANSocket m_socket;
};
