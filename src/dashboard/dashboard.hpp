#pragma once

#include <string>
#include <thread>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include "../telemetry/metrics.hpp"
#include "../vehicle/vehicle_state.hpp"

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
    struct SDLWindowDeleter { void operator()(SDL_Window* w) const { if (w) SDL_DestroyWindow(w); } };
    struct SDLRendererDeleter { void operator()(SDL_Renderer* r) const { SDL_DestroyRenderer(r); } };
    struct TTFFontDeleter     { void operator()(TTF_Font* f) const { TTF_CloseFont(f); } };

    Dashboard(VehicleState& state, Metrics& metrics);

    using WindowPtr = std::unique_ptr<SDL_Window, SDLWindowDeleter>;
    using RendererPtr = std::unique_ptr<SDL_Renderer, SDLRendererDeleter>;
    using FontPtr = std::unique_ptr<TTF_Font, TTFFontDeleter>;

    void run();

private:
    VehicleState& m_state;
    Metrics&      m_metrics;

    WindowPtr   m_window{nullptr};
    RendererPtr m_renderer{nullptr};
    FontPtr     m_fontLg{nullptr};
    FontPtr     m_fontMd{nullptr};
    FontPtr     m_fontSm{nullptr};

    bool   m_accel{false};
    bool   m_brake{false};
    double m_throttle{0.0};

    // background thread, sends periodic OBD queries so the CAN req counter increments
    std::jthread         m_poller;
    void pollerLoop(std::stop_token tok);

    void render();
    
    void blitText(const std::string& s, int x, int y, TTF_Font* font, SDL_Color col, bool centerX = true);
    void drawGauge(int cx, int cy, int r, double value, double maxVal, const char* label, const char* unit);
    void drawStat(int x, int y, int w, int h, const char* label, const std::string& value, SDL_Color vcol);
    void drawKeyHint(int x, int y, const char* keys, const char* action, bool active);
};
