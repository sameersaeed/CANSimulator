#pragma once

#include <atomic>
#include <string>
#include <thread>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include "Metrics.hpp"
#include "VehicleState.hpp"

namespace Labels {
    inline constexpr const char* SPEED       = "SPEED";
    inline constexpr const char* ENGINE_RPM  = "ENGINE RPM";
    inline constexpr const char* COOLANT     = "COOLANT";
    inline constexpr const char* ENGINE_LOAD = "ENGINE LOAD";
    inline constexpr const char* MAF         = "MAF";
    inline constexpr const char* FUEL        = "FUEL";
    inline constexpr const char* RUNTIME     = "RUNTIME";
}

class Dashboard {
public:
    Dashboard(VehicleState& state, Metrics& metrics);
    ~Dashboard();

    void run();

private:
    VehicleState& m_state;
    Metrics&      m_metrics;

    SDL_Window*   m_window{nullptr};
    SDL_Renderer* m_renderer{nullptr};
    TTF_Font*     m_fontLg{nullptr};
    TTF_Font*     m_fontMd{nullptr};
    TTF_Font*     m_fontSm{nullptr};

    bool   m_accel{false};
    bool   m_brake{false};
    double m_throttle{0.0};

    // background thread, sends periodic OBD queries so the CAN req counter increments
    std::thread          m_poller;
    std::atomic<bool>    m_pollerRunning{false};
    void pollerLoop();

    void render();
    
    void blitText(const std::string& s, int x, int y, TTF_Font* font, SDL_Color col, bool centerX = true);
    void drawGauge(int cx, int cy, int r, double value, double maxVal, const char* label, const char* unit);
    void drawStat(int x, int y, int w, int h, const char* label, const std::string& value, SDL_Color vcol);
    void drawKeyHint(int x, int y, const char* keys, const char* action, bool active);
};
