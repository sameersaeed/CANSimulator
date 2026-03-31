#pragma once

#include <atomic>
#include <string>

#include "CANSocket.hpp"
#include "Metrics.hpp"
#include "PIDHandler.hpp"
#include "VehicleState.hpp"

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

    void   handleRequest(const can_frame& frame);
    double getPidValue(PID pid) const noexcept;
};
