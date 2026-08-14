#include <algorithm>
#include <map>

#include "pid_handler.hpp"

namespace OBD::PID {

std::optional<Type> fromName(const std::string& name) {
    static const std::map<std::string, Type> table = {
        {"RPM",          Type::RPM},
        {"SPEED",        Type::SPEED},
        {"COOLANT_TEMP", Type::COOLANT_TEMP},
        {"ENGINE_LOAD",  Type::ENGINE_LOAD},
        {"THROTTLE",     Type::THROTTLE},
        {"INTAKE_TEMP",  Type::INTAKE_TEMP},
        {"MAF",          Type::MAF},
        {"FUEL_LEVEL",   Type::FUEL_LEVEL},
        {"RUNTIME",      Type::RUNTIME},
        {"DIST_DTC",     Type::DIST_DTC},
    };
    auto it = table.find(name);

    return (it != table.end()) ? std::optional<Type>{it->second} : std::nullopt;
}

std::string toName(Type pid) {
    switch (pid) {
        case Type::RPM:          return "RPM";
        case Type::SPEED:        return "SPEED";
        case Type::COOLANT_TEMP: return "COOLANT_TEMP";
        case Type::ENGINE_LOAD:  return "ENGINE_LOAD";
        case Type::THROTTLE:     return "THROTTLE";
        case Type::INTAKE_TEMP:  return "INTAKE_TEMP";
        case Type::MAF:          return "MAF";
        case Type::FUEL_LEVEL:   return "FUEL_LEVEL";
        case Type::RUNTIME:      return "RUNTIME";
        case Type::DIST_DTC:     return "DIST_DTC";
        default:                return "UNKNOWN";
    }
}

// encode physical value: OBD-II raw bytes (SAE J1979 table A-6)
Encoded encode(Type pid, double value) {
    Encoded enc{};
    switch (pid) {

        case Type::RPM: { // raw = value * 4
            uint16_t raw = static_cast<uint16_t>(std::clamp(value, 0.0, 16383.75) * 4.0);
            enc.numBytes = 2;
            enc.A = static_cast<uint8_t>((raw >> 8) & 0xFF);
            enc.B = static_cast<uint8_t>(raw & 0xFF);

            break;
        }

        case Type::SPEED: {
            enc.numBytes = 1;
            enc.A = static_cast<uint8_t>(std::clamp(value, 0.0, 255.0));

            break;
        }

        case Type::COOLANT_TEMP:
        case Type::INTAKE_TEMP: { // A = value + 40
            enc.numBytes = 1;
            enc.A = static_cast<uint8_t>(std::clamp(value + 40.0, 0.0, 255.0));

            break;
        }

        case Type::ENGINE_LOAD:
        case Type::THROTTLE:
        case Type::FUEL_LEVEL: {  // A = value * 2.55
            enc.numBytes = 1;
            enc.A = static_cast<uint8_t>(std::clamp(value * 2.55, 0.0, 255.0));

            break;
        }

        case Type::MAF: { // raw = value * 100
            uint16_t raw = static_cast<uint16_t>(std::clamp(value * 100.0, 0.0, 65535.0));
            enc.numBytes = 2;
            enc.A = static_cast<uint8_t>((raw >> 8) & 0xFF);
            enc.B = static_cast<uint8_t>(raw & 0xFF);

            break;
        }

        case Type::RUNTIME:
        case Type::DIST_DTC: { // 256*A + B (seconds / km)
            uint16_t raw = static_cast<uint16_t>(std::clamp(value, 0.0, 65535.0));
            enc.numBytes = 2;
            enc.A = static_cast<uint8_t>((raw >> 8) & 0xFF);
            enc.B = static_cast<uint8_t>(raw & 0xFF);

            break;
        }

        default:
            enc.numBytes = 1;
            enc.A = 0;
    }
    return enc;
}

// decode OBD-II raw bytes to physical value (SAE J1979 table A-6)
std::optional<Decoded> decode(uint8_t pidByte, uint8_t A, uint8_t B) {
    switch (pidByte) {
        case 0x04: return Decoded{"Engine Load",              A / 2.55,                             "%"    };
        case 0x05: return Decoded{"Coolant Temp",             static_cast<double>(A) - 40.0,        "°C"   };
        case 0x0C: return Decoded{"Engine RPM",               (256.0 * A + B) / 4.0,                "rpm"  };
        case 0x0D: return Decoded{"Vehicle Speed",            static_cast<double>(A),               "km/h" };
        case 0x0F: return Decoded{"Intake Air Temp",          static_cast<double>(A) - 40.0,        "°C"   };
        case 0x10: return Decoded{"MAF Air Flow",             (256.0 * A + B) / 100.0,              "g/s"  };
        case 0x11: return Decoded{"Throttle Position",        A / 2.55,                             "%"    };
        case 0x1F: return Decoded{"Runtime Since Start",      static_cast<double>(256 * A + B),     "s"    };
        case 0x2F: return Decoded{"Fuel Level",               A / 2.55,                             "%"    };
        case 0x31: return Decoded{"Distance Since DTC Clear", static_cast<double>(256 * A + B),     "km"   };
        default:   return std::nullopt;
    }
}

} // namespace OBD::PID