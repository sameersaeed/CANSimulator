#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <thread>

#include "simulator.hpp"

// simulation engine cycles through idle -> accel -> cruise -> decel via |sin(t)|

void Simulator::run() {
    using namespace std::chrono_literals;

    double t = 0.0;
    const double dt = 0.05; // 50 ms tick = 20 Hz

    // persistent state for manual mode -- not used in autonomous mode
    double local_rpm     = 800.0;
    double local_speed   = 0.0;
    double local_coolant = 75.0; // starts cold, warms under load
    double local_fuel    = m_state.fuel_level.load(); // carry over initial value

    while (m_state.running.load()) {
        double rpm, speed, throttle, load, maf;

        if (m_state.manual_control.load()) {
            double user_t  = m_state.user_throttle.load();
            bool   braking = m_state.user_brake.load();

            // kill throttle if out of fuel
            if (local_fuel <= 0.0) user_t = 0.0;

            // ramp rpm toward target at ~14 RPM/tick (0->full in ~20s at full throttle)
            double target_rpm = braking ? 800.0 : (800.0 + user_t * 54.0);
            double ramp       = braking ? 80.0 : 14.0;
            if (local_rpm < target_rpm) local_rpm = std::min(local_rpm + ramp, target_rpm);
            else                        local_rpm = std::max(local_rpm - ramp, target_rpm);

            rpm      = local_rpm;
            throttle = user_t;
            load     = std::clamp(throttle * 0.85 + 8.0, 10.0, 100.0);
            maf      = std::clamp(rpm / 500.0 * (load / 100.0) + 1.5, 1.5, 28.0);

            // speed follows rpm with lag -- divisor 15 gives ~2300 RPM at 100 km/h
            double target_speed = std::clamp((rpm - 800.0) / 15.0, 0.0, 180.0);
            if (braking)                         local_speed = std::max(0.0, local_speed - 3.0);
            else if (local_speed < target_speed) local_speed = std::min(local_speed + 0.5, target_speed);
            else                                 local_speed = std::max(local_speed - 1.0, target_speed);
            speed = local_speed;
        } else {
            // autonomous sin-wave mode
            rpm      = 800.0 + 5400.0 * std::abs(std::sin(t * 0.25));

            // speed, load, and maf all derived from rpm
            speed    = std::clamp((rpm - 800.0) / 15.0, 0.0, 180.0);
            throttle = std::clamp((rpm - 800.0) / 54.0, 5.0, 100.0);
            load     = std::clamp(throttle * 0.85 + 8.0, 10.0, 100.0);
            maf      = std::clamp(rpm / 500.0 * (load / 100.0) + 1.5, 1.5, 28.0);
        }

        // coolant target rises with load; ramps slowly (0.05 deg/tick)
        double coolant_target = 75.0 + load * 0.35 + 2.5 * std::sin(t * 0.04);
        local_coolant += std::clamp(coolant_target - local_coolant, -0.05, 0.05);

        double intake = 25.0 + 4.0 * std::sin(t * 0.09 + 1.2);

        // fuel drain proportional to load (~5% per min at full load, near-zero at idle)
        local_fuel = std::max(0.0, local_fuel - (load / 100.0) * 0.004);

        double new_runtime  = m_state.runtime.load()  + dt;
        double new_dist_dtc = m_state.dist_dtc.load() + speed * dt / 3600.0;

        m_state.rpm          = rpm;
        m_state.speed        = speed;
        m_state.throttle     = throttle;
        m_state.engine_load  = load;
        m_state.maf          = maf;
        m_state.coolant_temp = local_coolant;
        m_state.intake_temp  = intake;
        m_state.fuel_level   = local_fuel;
        m_state.runtime      = new_runtime;
        m_state.dist_dtc     = new_dist_dtc;

        t += dt;
        std::this_thread::sleep_for(50ms);
    }
}
