#include <algorithm>
#include <map>

#include "pid_handler.hpp"

std::optional<PID> pid_from_name(const std::string& name) {
    static const std::map<std::string, PID> table = {
        {"RPM",          PID::RPM},
        {"SPEED",        PID::SPEED},
        {"COOLANT_TEMP", PID::COOLANT_TEMP},
        {"ENGINE_LOAD",  PID::ENGINE_LOAD},
        {"THROTTLE",     PID::THROTTLE},
        {"INTAKE_TEMP",  PID::INTAKE_TEMP},
        {"MAF",          PID::MAF},
        {"FUEL_LEVEL",   PID::FUEL_LEVEL},
        {"RUNTIME",      PID::RUNTIME},
        {"DIST_DTC",     PID::DIST_DTC},
    };
    auto it = table.find(name);
    return (it != table.end()) ? std::optional<PID>{it->second} : std::nullopt;
}

std::string pid_to_name(PID pid) {
    switch (pid) {
        case PID::RPM:          return "RPM";
        case PID::SPEED:        return "SPEED";
        case PID::COOLANT_TEMP: return "COOLANT_TEMP";
        case PID::ENGINE_LOAD:  return "ENGINE_LOAD";
        case PID::THROTTLE:     return "THROTTLE";
        case PID::INTAKE_TEMP:  return "INTAKE_TEMP";
        case PID::MAF:          return "MAF";
        case PID::FUEL_LEVEL:   return "FUEL_LEVEL";
        case PID::RUNTIME:      return "RUNTIME";
        case PID::DIST_DTC:     return "DIST_DTC";
        default:                return "UNKNOWN";
    }
}

// encode physical value: OBD-II raw bytes (SAE J1979 table A-6)
EncodedPID encode_pid(PID pid, double value) {
    EncodedPID enc{};
    switch (pid) {

        case PID::RPM: { // raw = value * 4
            uint16_t raw = static_cast<uint16_t>(
                std::clamp(value, 0.0, 16383.75) * 4.0);
            enc.num_bytes = 2;
            enc.A = static_cast<uint8_t>((raw >> 8) & 0xFF);
            enc.B = static_cast<uint8_t>(raw & 0xFF);
            break;
        }

        case PID::SPEED: {
            enc.num_bytes = 1;
            enc.A = static_cast<uint8_t>(std::clamp(value, 0.0, 255.0));
            break;
        }

        case PID::COOLANT_TEMP:
        case PID::INTAKE_TEMP: { // A = value + 40
            enc.num_bytes = 1;
            enc.A = static_cast<uint8_t>(std::clamp(value + 40.0, 0.0, 255.0));
            break;
        }

        case PID::ENGINE_LOAD:
        case PID::THROTTLE:
        case PID::FUEL_LEVEL: {  // A = value * 2.55
            enc.num_bytes = 1;
            enc.A = static_cast<uint8_t>(std::clamp(value * 2.55, 0.0, 255.0));
            break;
        }

        case PID::MAF: { // raw = value * 100
            uint16_t raw = static_cast<uint16_t>(
                std::clamp(value * 100.0, 0.0, 65535.0));
            enc.num_bytes = 2;
            enc.A = static_cast<uint8_t>((raw >> 8) & 0xFF);
            enc.B = static_cast<uint8_t>(raw & 0xFF);
            break;
        }

        case PID::RUNTIME:
        case PID::DIST_DTC: { // 256*A + B (seconds / km)
            uint16_t raw = static_cast<uint16_t>(
                std::clamp(value, 0.0, 65535.0));
            enc.num_bytes = 2;
            enc.A = static_cast<uint8_t>((raw >> 8) & 0xFF);
            enc.B = static_cast<uint8_t>(raw & 0xFF);
            break;
        }

        default:
            enc.num_bytes = 1;
            enc.A = 0;
    }
    return enc;
}

// decode OBD-II raw bytes → physical value (SAE J1979 table A-6)
std::optional<PIDResult> decode_response(uint8_t pid_byte, uint8_t A, uint8_t B) {
    switch (pid_byte) {
        case 0x04: return PIDResult{"Engine Load",              A / 2.55,                             "%"    };
        case 0x05: return PIDResult{"Coolant Temp",             static_cast<double>(A) - 40.0,        "°C"   };
        case 0x0C: return PIDResult{"Engine RPM",               (256.0 * A + B) / 4.0,                "rpm"  };
        case 0x0D: return PIDResult{"Vehicle Speed",            static_cast<double>(A),               "km/h" };
        case 0x0F: return PIDResult{"Intake Air Temp",          static_cast<double>(A) - 40.0,        "°C"   };
        case 0x10: return PIDResult{"MAF Air Flow",             (256.0 * A + B) / 100.0,              "g/s"  };
        case 0x11: return PIDResult{"Throttle Position",        A / 2.55,                             "%"    };
        case 0x1F: return PIDResult{"Runtime Since Start",      static_cast<double>(256 * A + B),     "s"    };
        case 0x2F: return PIDResult{"Fuel Level",               A / 2.55,                             "%"    };
        case 0x31: return PIDResult{"Distance Since DTC Clear", static_cast<double>(256 * A + B),     "km"   };
        default:   return std::nullopt;
    }
}
