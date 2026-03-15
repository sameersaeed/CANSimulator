#pragma once

#include <atomic>

// state gets shared between simulator (writes) and OBD server (reads)
struct VehicleState {
    std::atomic<double> rpm         {800.0};
    std::atomic<double> speed       {0.0};
    std::atomic<double> coolant_temp{90.0};
    std::atomic<double> engine_load {15.0};
    std::atomic<double> throttle    {5.0};
    std::atomic<double> intake_temp {25.0};
    std::atomic<double> maf         {3.0};
    std::atomic<double> fuel_level  {75.0};
    std::atomic<double> runtime     {0.0};
    std::atomic<double> dist_dtc    {0.0};

    std::atomic<bool>   running{true};        // shutdown signal polled by both server / client threads

    // dashboard inputs -- only used when manual_control is true
    std::atomic<double> user_throttle{0.0};   // 0-100 set by dashboard
    std::atomic<bool>   user_brake{false};
    std::atomic<bool>   manual_control{false};
};
