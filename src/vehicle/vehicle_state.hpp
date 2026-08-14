#pragma once

#include <atomic>

// state gets shared between simulator (writes) and OBD server (reads)
struct VehicleState {
    std::atomic<double> rpm         {800.0};
    std::atomic<double> speed       {0.0};
    std::atomic<double> coolantTemp {90.0};
    std::atomic<double> engineLoad  {15.0};
    std::atomic<double> throttle    {5.0};
    std::atomic<double> intakeTemp  {25.0};
    std::atomic<double> maf         {3.0};
    std::atomic<double> fuelLevel   {75.0};
    std::atomic<double> runtime     {0.0};
    std::atomic<double> distDTC     {0.0};

    std::atomic<bool>   running{true};        // shutdown signal polled by both server / client threads

    // dashboard inputs - only used when manual control is enabled
    std::atomic<double> userThrottle{0.0};   // 0-100 set by dashboard
    std::atomic<bool>   userBrake{false};
    std::atomic<bool>   manualControl{false};
};
