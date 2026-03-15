#pragma once

#include <atomic>
#include <string>
#include <thread>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include "metrics.hpp"
#include "vehicle_state.hpp"

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
    TTF_Font*     m_font_lg{nullptr};
    TTF_Font*     m_font_md{nullptr};
    TTF_Font*     m_font_sm{nullptr};

    bool   m_accel{false};
    bool   m_brake{false};
    double m_throttle{0.0};

    // background thread, sends periodic OBD queries so the CAN req counter increments
    std::thread          m_poller;
    std::atomic<bool>    m_poller_running{false};
    void poller_loop();

    void render();
    
    void blit_text(const std::string& s, int x, int y, TTF_Font* font, SDL_Color col, bool center_x = true);
    void draw_gauge(int cx, int cy, int r, double value, double max_val, const char* label, const char* unit);
    void draw_stat(int x, int y, int w, int h, const char* label, const std::string& value, SDL_Color vcol);
    void draw_key_hint(int x, int y, const char* keys, const char* action, bool active);
};
