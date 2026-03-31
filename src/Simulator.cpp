#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <thread>

#include "Simulator.hpp"

// simulation engine cycles through idle -> accel -> cruise -> decel via |sin(t)|

void Simulator::run() {
    using namespace std::chrono_literals;

    double t = 0.0;
    const double dt = 0.05; // 50 ms tick = 20 Hz

    // persistent state for manual mode - not used in autonomous mode
    double localRPM     = 800.0;
    double localSpeed   = 0.0;
    double localCoolant = 75.0; // starts cold, warms under load
    double localFuel    = m_state.fuelLevel.load(); // carry over initial value

    while (m_state.running.load()) {
        double rpm, speed, throttle, load, maf;

        if (m_state.manualControl.load()) {
            double userThrottleInput  = m_state.userThrottle.load();
            bool   braking = m_state.userBrake.load();

            // kill throttle if out of fuel
            if (localFuel <= 0.0) userThrottleInput = 0.0;

            // ramp rpm toward target at ~14 RPM/tick (0->full in ~20s at full throttle)
            double targetRPM = braking ? 800.0 : (800.0 + userThrottleInput * 54.0);
            double ramp      = braking ? 80.0 : 14.0;
            if (localRPM < targetRPM)  localRPM = std::min(localRPM + ramp, targetRPM);
            else                       localRPM = std::max(localRPM - ramp, targetRPM);

            rpm      = localRPM;
            throttle = userThrottleInput;
            load     = std::clamp(throttle * 0.85 + 8.0, 10.0, 100.0);
            maf      = std::clamp(rpm / 500.0 * (load / 100.0) + 1.5, 1.5, 28.0);

            // speed follows rpm with lag -- divisor 15 gives ~2300 RPM at 100 km/h
            double targetSpeed = std::clamp((rpm - 800.0) / 15.0, 0.0, 180.0);
            if (braking)                        localSpeed = std::max(0.0, localSpeed - 3.0);
            else if (localSpeed < targetSpeed)  localSpeed = std::min(localSpeed + 0.5, targetSpeed);
            else                                localSpeed = std::max(localSpeed - 1.0, targetSpeed);
            
            speed = localSpeed;
        } 
        
        else {
            rpm      = 800.0 + 5400.0 * std::abs(std::sin(t * 0.25)); // goes in a sin-wave in auto mode

            // speed, load, and maf all derived from rpm
            speed    = std::clamp((rpm - 800.0) / 15.0, 0.0, 180.0);
            throttle = std::clamp((rpm - 800.0) / 54.0, 5.0, 100.0);
            load     = std::clamp(throttle * 0.85 + 8.0, 10.0, 100.0);
            maf      = std::clamp(rpm / 500.0 * (load / 100.0) + 1.5, 1.5, 28.0);
        }

        // coolant target rises with load; ramps slowly (0.05 deg/tick)
        double coolantTarget = 75.0 + load * 0.35 + 2.5 * std::sin(t * 0.04);
        localCoolant += std::clamp(coolantTarget - localCoolant, -0.05, 0.05);

        double intake = 25.0 + 4.0 * std::sin(t * 0.09 + 1.2);

        // fuel drain proportional to load (~5% per min at full load, near-zero at idle)
        localFuel = std::max(0.0, localFuel - (load / 100.0) * 0.004);

        double newRuntime  = m_state.runtime.load()  + dt;
        double newDistDTC = m_state.distDTC.load() + speed * dt / 3600.0;

        m_state.rpm         = rpm;
        m_state.speed       = speed;
        m_state.throttle    = throttle;
        m_state.engineLoad  = load;
        m_state.maf         = maf;
        m_state.coolantTemp = localCoolant;
        m_state.intakeTemp  = intake;
        m_state.fuelLevel   = localFuel;
        m_state.runtime     = newRuntime;
        m_state.distDTC     = newDistDTC;

        t += dt;
        std::this_thread::sleep_for(50ms);
    }
}
