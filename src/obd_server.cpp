#include <linux/can.h>
#include <chrono>

#include "obd_server.hpp"
#include "logger.hpp"

OBDServer::OBDServer(VehicleState& state, Metrics& metrics, const std::string& iface) 
                    : m_state(state), m_metrics(metrics), m_socket(iface) {
    m_socket.set_filter(0x7DF, 0x7FF); // filter to broadcast request id
}

double OBDServer::get_pid_value(PID pid) const noexcept {
    switch (pid) {
        case PID::RPM:          return m_state.rpm.load();
        case PID::SPEED:        return m_state.speed.load();
        case PID::COOLANT_TEMP: return m_state.coolant_temp.load();
        case PID::ENGINE_LOAD:  return m_state.engine_load.load();
        case PID::THROTTLE:     return m_state.throttle.load();
        case PID::INTAKE_TEMP:  return m_state.intake_temp.load();
        case PID::MAF:          return m_state.maf.load();
        case PID::FUEL_LEVEL:   return m_state.fuel_level.load();
        case PID::RUNTIME:      return m_state.runtime.load();
        case PID::DIST_DTC:     return m_state.dist_dtc.load();
        default:                return 0.0;
    }
}

// hot path
void OBDServer::handle_request(const can_frame& frame) {
    if (frame.can_id != 0x7DF && frame.can_id != 0x7E0) return;
    if (frame.can_dlc < 3)     return;
    if (frame.data[1] != 0x01) return; // Mode 01 only

    uint8_t pid_byte = frame.data[2];

    PID pid;
    switch (pid_byte) {
        case 0x04: pid = PID::ENGINE_LOAD;  break;
        case 0x05: pid = PID::COOLANT_TEMP; break;
        case 0x0C: pid = PID::RPM;          break;
        case 0x0D: pid = PID::SPEED;        break;
        case 0x0F: pid = PID::INTAKE_TEMP;  break;
        case 0x10: pid = PID::MAF;          break;
        case 0x11: pid = PID::THROTTLE;     break;
        case 0x1F: pid = PID::RUNTIME;      break;
        case 0x2F: pid = PID::FUEL_LEVEL;   break;
        case 0x31: pid = PID::DIST_DTC;     break;
        default:   return; // unsupported pid
    }

    // start timing after validation, before encode
    auto t_start = std::chrono::steady_clock::now();

    double     value = get_pid_value(pid);
    EncodedPID enc   = encode_pid(pid, value);

    can_frame response{};
    response.can_id  = 0x7E8; // ECU 1 response id
    response.can_dlc = 8;
    response.data[0] = static_cast<uint8_t>(enc.num_bytes + 2); // mode + pid + data bytes
    response.data[1] = 0x41; // 0x40 | mode (positive response)
    response.data[2] = pid_byte;
    response.data[3] = enc.A;
    response.data[4] = enc.B;
    response.data[5] = response.data[6] = response.data[7] = 0x00;

    m_socket.send_frame(response);

    auto latency_us = std::chrono::duration_cast<std::chrono::microseconds>(
                          std::chrono::steady_clock::now() - t_start).count();
    m_metrics.record_request(latency_us);
}

void OBDServer::run() {
    Logger::info("OBDServer listening on vcan0 (CAN ID 0x7DF)");
    while (m_running.load() && m_state.running.load()) {
        auto frame = m_socket.receive_frame(100);
        if (frame) handle_request(*frame);
    }
    Logger::info("OBDServer stopped");
}
