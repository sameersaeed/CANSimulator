#pragma once

#include <atomic>
#include <string>

#include "can_socket.hpp"
#include "metrics.hpp"
#include "pid_handler.hpp"
#include "vehicle_state.hpp"

class OBDServer {
public:
    OBDServer(VehicleState& state, Metrics& metrics, const std::string& iface = "vcan0");

    void run();
    void stop() { m_running = false; }

private:
    VehicleState&      m_state;
    Metrics&           m_metrics;
    CANSocket          m_socket;
    std::atomic<bool>  m_running{true};

    void   handle_request(const can_frame& frame);
    double get_pid_value(PID pid) const noexcept;
};
