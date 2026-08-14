#include <linux/can.h>
#include <chrono>

#include "../telemetry/logger.hpp"
#include "server.hpp"

namespace OBD {

Server::Server(VehicleState& state, Metrics& metrics, const std::string& iface) : m_state(state), m_metrics(metrics), m_socket(iface) {
    m_socket.setFilter(0x7DF, 0x7FF); // filter to broadcast request id
}

double Server::getPidValue(PID::Type pid) const noexcept {
    switch (pid) {
        case PID::Type::RPM:          return m_state.rpm.load();
        case PID::Type::SPEED:        return m_state.speed.load();
        case PID::Type::COOLANT_TEMP: return m_state.coolantTemp.load();
        case PID::Type::ENGINE_LOAD:  return m_state.engineLoad.load();
        case PID::Type::THROTTLE:     return m_state.throttle.load();
        case PID::Type::INTAKE_TEMP:  return m_state.intakeTemp.load();
        case PID::Type::MAF:          return m_state.maf.load();
        case PID::Type::FUEL_LEVEL:   return m_state.fuelLevel.load();
        case PID::Type::RUNTIME:      return m_state.runtime.load();
        case PID::Type::DIST_DTC:     return m_state.distDTC.load();
        default:                      return 0.0;
    }
}

void Server::handleRequest(const can_frame& frame) {
    if (frame.can_id != 0x7DF && frame.can_id != 0x7E0) return;
    if (frame.can_dlc < 3)     return;
    if (frame.data[1] != 0x01) return; // Mode 01 only

    uint8_t pidByte = frame.data[2];

    PID::Type pid;
    switch (pidByte) {
        case 0x04: pid = PID::Type::ENGINE_LOAD;  break;
        case 0x05: pid = PID::Type::COOLANT_TEMP; break;
        case 0x0C: pid = PID::Type::RPM;          break;
        case 0x0D: pid = PID::Type::SPEED;        break;
        case 0x0F: pid = PID::Type::INTAKE_TEMP;  break;
        case 0x10: pid = PID::Type::MAF;          break;
        case 0x11: pid = PID::Type::THROTTLE;     break;
        case 0x1F: pid = PID::Type::RUNTIME;      break;
        case 0x2F: pid = PID::Type::FUEL_LEVEL;   break;
        case 0x31: pid = PID::Type::DIST_DTC;     break;
        default:   return; // unsupported pid
    }

    // start timing after validation, before encode
    auto startTime = std::chrono::steady_clock::now();

    double value = Server::getPidValue(pid);
    PID::Encoded enc = PID::encode(pid, value);

    can_frame response{};
    response.can_id  = 0x7E8; // ECU 1 response id
    response.can_dlc = 8;
    response.data[0] = static_cast<uint8_t>(enc.numBytes + 2); // mode + pid + data bytes
    response.data[1] = 0x41; // 0x40 | mode (positive response)
    response.data[2] = pidByte;
    response.data[3] = enc.A;
    response.data[4] = enc.B;
    response.data[5] = response.data[6] = response.data[7] = 0x00;

    m_socket.sendFrame(response);

    auto latencyUs = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - startTime).count();

    m_metrics.recordRequest(latencyUs);
}

void Server::run() {
    Logger::info("OBD server listening on vcan0 (CAN ID 0x7DF)");
    
    while (m_running.load() && m_state.running.load()) {
        auto frame = m_socket.receiveFrame(0.1); // 0.1 sec timeout
        if (frame) Server::handleRequest(*frame);
    }
    
    Logger::info("OBD server stopped");
}

} // namespace OBD