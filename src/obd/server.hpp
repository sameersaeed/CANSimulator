#pragma once

#include <atomic>
#include <string>

#include "../can/socket.hpp"
#include "../telemetry/metrics.hpp"
#include "../vehicle/vehicle_state.hpp"

#include "pid_handler.hpp"

namespace OBD {
 
class Server {
public:
    Server(VehicleState& state, Metrics& metrics, const std::string& iface = "vcan0");

    void run();
    void stop() { m_running = false; }

private:
    VehicleState&       m_state;
    Metrics&            m_metrics;
    CAN::Socket         m_socket;
    std::atomic<bool>   m_running{true};

    void   handleRequest(const can_frame& frame);
    double getPidValue(PID::Type pid) const noexcept;
};

} // namespace OBD