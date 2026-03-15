#pragma once

#include <cstdint>
#include <optional>
#include <string>

// SAE J1979 Mode 01 PIDs 
enum class PID : uint8_t {
    ENGINE_LOAD  = 0x04,
    COOLANT_TEMP = 0x05,
    RPM          = 0x0C,
    SPEED        = 0x0D,
    INTAKE_TEMP  = 0x0F,
    MAF          = 0x10,
    THROTTLE     = 0x11,
    RUNTIME      = 0x1F,
    FUEL_LEVEL   = 0x2F,
    DIST_DTC     = 0x31,
};

// encoded OBD-II data bytes (A and optionally B)
struct EncodedPID {
    uint8_t num_bytes{1};
    uint8_t A{0};
    uint8_t B{0};
};

// decoded OBD-II response frame result
struct PIDResult {
    std::string name;
    double      value{0.0};
    std::string unit;
};

// encode physical OBD-II raw byte val (SAE J1979 table A-6)
EncodedPID encode_pid(PID pid, double value);

// decode raw bytes (A and B) from Mode 01 response
std::optional<PIDResult> decode_response(uint8_t pid_byte, uint8_t A, uint8_t B);

std::optional<PID> pid_from_name(const std::string& name);
std::string        pid_to_name(PID pid);
