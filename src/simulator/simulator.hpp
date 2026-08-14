#pragma once

#include "../vehicle/vehicle_state.hpp"

// updates vehicle's state at ~20 Hz, simulating engine behaviour
class Simulator {
public:
    explicit Simulator(VehicleState& state) : m_state(state) {}

    void run();
    void stop() { m_state.running = false; }

private:
    VehicleState& m_state;
};
