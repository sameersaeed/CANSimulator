#include "dashboard.hpp"
#include "obd_client.hpp"

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <thread>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// window size
static const int WIN_W = 900;
static const int WIN_H = 560;

// gauge layout
static const int SPD_CX = 220, SPD_CY = 265, SPD_R = 155;
static const int RPM_CX = 680, RPM_CY = 265, RPM_R = 155;

// gauge arc geometry
static const double ARC_START = 120.0;
static const double ARC_SWEEP = 300.0;

// colour palette
static const SDL_Color BG        = {13,  17,  23,  255};
static const SDL_Color PANEL     = {22,  30,  42,  255};
static const SDL_Color BORDER    = {48,  60,  78,  255};
static const SDL_Color TEXT      = {220, 225, 235, 255};
static const SDL_Color SUBTEXT   = {110, 122, 140, 255};
static const SDL_Color DARK_ARC  = {32,  42,  58,  255};
static const SDL_Color COL_GREEN  = {34,  197, 94,  255};
static const SDL_Color COL_YELLOW = {234, 179, 8,   255};
static const SDL_Color COL_RED    = {239, 68,  68,  255};
static const SDL_Color COL_BLUE   = {59,  130, 246, 255};
static const SDL_Color COL_WHITE  = {255, 255, 255, 255};

static inline void set_col(SDL_Renderer* r, SDL_Color c) {
    SDL_SetRenderDrawColor(r, c.r, c.g, c.b, c.a);
}

// guage arc created by stacking concentric arcs
static void draw_arc(SDL_Renderer* r, int cx, int cy, int radius,
                     double start_deg, double end_deg, SDL_Color col, int thick = 7) {
    set_col(r, col);
    const double step = 0.4;
    for (int dr = -(thick / 2); dr <= thick / 2; ++dr) {
        double rr = radius + dr;
        double px = cx + rr * std::cos(start_deg * M_PI / 180.0);
        double py = cy + rr * std::sin(start_deg * M_PI / 180.0);
        for (double a = start_deg + step; a <= end_deg + step * 0.5; a += step) {
            double rad = a * M_PI / 180.0;
            double nx  = cx + rr * std::cos(rad);
            double ny  = cy + rr * std::sin(rad);
            SDL_RenderDrawLine(r, (int)px, (int)py, (int)nx, (int)ny);
            px = nx;
            py = ny;
        }
    }
}

static void draw_filled_circle(SDL_Renderer* r, int cx, int cy, int radius, SDL_Color col) {
    set_col(r, col);
    for (int dy = -radius; dy <= radius; ++dy) {
        int dx = (int)std::sqrt((double)(radius * radius - dy * dy));
        SDL_RenderDrawLine(r, cx - dx, cy + dy, cx + dx, cy + dy);
    }
}

// needle from center toward arc edge
static void draw_needle(SDL_Renderer* r, int cx, int cy, int length,
                        double angle_deg, SDL_Color col) {
    double rad  = angle_deg * M_PI / 180.0;
    double perp = rad + M_PI / 2.0;
    int    ex   = cx + (int)(length * std::cos(rad));
    int    ey   = cy + (int)(length * std::sin(rad));
    set_col(r, col);
    for (int i = -2; i <= 2; ++i) {
        int ox = (int)(i * std::cos(perp));
        int oy = (int)(i * std::sin(perp));
        SDL_RenderDrawLine(r, cx + ox, cy + oy, ex + ox, ey + oy);
    }
}

// try common Linux system font paths for the text
static TTF_Font* try_load_font(int pt) {
    static const char* paths[] = {
        // arch
        "/usr/share/fonts/gnu-free/FreeSans.otf",
        "/usr/share/fonts/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/noto/NotoSans-Regular.ttf",
        "/usr/share/fonts/TTF/DejaVuSans.ttf",
        // ubuntu / debian
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
        "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf",
        // fedora / rhel
        "/usr/share/fonts/liberation-sans/LiberationSans-Regular.ttf",
        "/usr/share/fonts/google-noto/NotoSans-Regular.ttf",
        nullptr
    };

    for (int i = 0; paths[i]; ++i) {
        TTF_Font* f = TTF_OpenFont(paths[i], pt);
        if (f) return f;
    }

    return nullptr; // couldnt find any fonts
}

Dashboard::Dashboard(VehicleState& state, Metrics& metrics) 
                    : m_state(state), m_metrics(metrics) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
        throw std::runtime_error(std::string("SDL_Init: ") + SDL_GetError());

    if (TTF_Init() < 0)
        throw std::runtime_error(std::string("TTF_Init: ") + TTF_GetError());

    m_window = SDL_CreateWindow("CANSimulator - OBD-II Dashboard",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WIN_W, WIN_H, SDL_WINDOW_SHOWN);
 
    if (!m_window)
        throw std::runtime_error(std::string("CreateWindow: ") + SDL_GetError());

    m_renderer = SDL_CreateRenderer(m_window, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    if (!m_renderer)
        throw std::runtime_error(std::string("CreateRenderer: ") + SDL_GetError());

    SDL_SetRenderDrawBlendMode(m_renderer, SDL_BLENDMODE_BLEND);

    m_font_lg = try_load_font(40);
    m_font_md = try_load_font(17);
    m_font_sm = try_load_font(13);

    if (!m_font_lg || !m_font_md || !m_font_sm)
        throw std::runtime_error(
            "no system font found -- run: sudo apt install fonts-dejavu-core");

    m_poller_running = true;
    m_poller = std::thread([this]{ poller_loop(); });
}

Dashboard::~Dashboard() {
    m_poller_running = false;
    if (m_poller.joinable()) m_poller.join();

    if (m_font_sm) TTF_CloseFont(m_font_sm);
    if (m_font_md) TTF_CloseFont(m_font_md);
    if (m_font_lg) TTF_CloseFont(m_font_lg);
    if (m_renderer) SDL_DestroyRenderer(m_renderer);
    if (m_window)   SDL_DestroyWindow(m_window);

    TTF_Quit();
    SDL_Quit();
}

// queries RPM via OBDClient every ~100ms so the CAN req counter increments
void Dashboard::poller_loop() {
    using namespace std::chrono_literals;
    OBDClient client;
    while (m_poller_running.load() && m_state.running.load()) {
        client.query(PID::RPM, 200);
        std::this_thread::sleep_for(100ms);
    }
}

void Dashboard::blit_text(const std::string& s, int x, int y, TTF_Font* font,
                           SDL_Color col, bool center_x) {
    if (!font || s.empty()) return;

    SDL_Surface* surf = TTF_RenderText_Blended(font, s.c_str(), col);
    if (!surf) return;

    SDL_Texture* tex = SDL_CreateTextureFromSurface(m_renderer, surf);
    SDL_Rect dst = {center_x ? x - surf->w / 2 : x, y, surf->w, surf->h};

    SDL_RenderCopy(m_renderer, tex, nullptr, &dst);
    SDL_FreeSurface(surf);
    SDL_DestroyTexture(tex);
}

void Dashboard::draw_gauge(int cx, int cy, int r, double value, double max_val,
                            const char* label, const char* unit) {
    // full background track
    draw_arc(m_renderer, cx, cy, r, ARC_START, ARC_START + ARC_SWEEP, DARK_ARC, 9);

    // color value arc, green -> yellow -> red
    double frac = std::clamp(value / max_val, 0.0, 1.0);
    if (frac > 0.001) {
        SDL_Color arc_col = (frac < 0.70) ? COL_GREEN : (frac < 0.88) ? COL_YELLOW : COL_RED;
        draw_arc(m_renderer, cx, cy, r, ARC_START, ARC_START + frac * ARC_SWEEP, arc_col, 9);
    }

    // tick marks
    for (int i = 0; i <= 20; ++i) {
        bool   major = (i % 5 == 0); // on interval of 5
        double arad  = (ARC_START + (i / 20.0) * ARC_SWEEP) * M_PI / 180.0;
        int    r_out = r + 5;
        int    r_in  = major ? r - 18 : r - 8;

        set_col(m_renderer, major ? TEXT : BORDER);
        SDL_RenderDrawLine(m_renderer,
            cx + (int)(r_in  * std::cos(arad)), cy + (int)(r_in  * std::sin(arad)),
            cx + (int)(r_out * std::cos(arad)), cy + (int)(r_out * std::sin(arad)));

        if (major) {
            SDL_RenderDrawLine(m_renderer,
                cx + (int)((r_in + 1) * std::cos(arad)), cy + (int)((r_in + 1) * std::sin(arad)),
                cx + (int)((r_out + 1) * std::cos(arad)), cy + (int)((r_out + 1) * std::sin(arad)));
        }
    }

    // labels at 0%, 50%, and 100% of range
    const double label_fracs[] = {0.0, 0.5, 1.0};
    for (double lf : label_fracs) {
        double arad = (ARC_START + lf * ARC_SWEEP) * M_PI / 180.0;
        int    lx   = cx + (int)((r - 32) * std::cos(arad));
        int    ly   = cy + (int)((r - 32) * std::sin(arad));
        char   lbuf[12];
        double lval = lf * max_val;

        if (lval >= 1000) snprintf(lbuf, sizeof(lbuf), "%.0fk", lval / 1000.0);
        else              snprintf(lbuf, sizeof(lbuf), "%.0f",  lval);

        blit_text(lbuf, lx, ly - 7, m_font_sm, SUBTEXT);
    }

    // needle
    draw_needle(m_renderer, cx, cy, r - 16, ARC_START + frac * ARC_SWEEP, COL_WHITE);

    // center cap
    draw_filled_circle(m_renderer, cx, cy, 14, PANEL);
    draw_filled_circle(m_renderer, cx, cy,  7, COL_WHITE);

    // large val in center
    char vbuf[12];
    snprintf(vbuf, sizeof(vbuf), "%.0f", value);
    blit_text(vbuf, cx, cy + 48, m_font_lg, TEXT);

    blit_text(unit,  cx, cy + 95,  m_font_sm, SUBTEXT);
    blit_text(label, cx, cy - 88, m_font_sm, SUBTEXT);
}

void Dashboard::draw_stat(int x, int y, int w, int h,
                           const char* label, const std::string& value, SDL_Color vcol) {
    SDL_Rect bg = {x, y, w, h};
    
    set_col(m_renderer, PANEL);
    SDL_RenderFillRect(m_renderer, &bg);
    
    set_col(m_renderer, BORDER);
    SDL_RenderDrawRect(m_renderer, &bg);

    blit_text(label, x + w / 2, y + 8,        m_font_sm, SUBTEXT);
    blit_text(value, x + w / 2, y + h / 2 + 5, m_font_md, vcol);
}

void Dashboard::draw_key_hint(int x, int y, const char* keys, const char* action, bool active) {
    const int w = 190, h = 30;
    SDL_Rect bg = {x, y, w, h};
    
    set_col(m_renderer, active ? COL_BLUE : PANEL);
    SDL_RenderFillRect(m_renderer, &bg);
    
    set_col(m_renderer, active ? COL_BLUE : BORDER);
    SDL_RenderDrawRect(m_renderer, &bg);

    char buf[48];
    snprintf(buf, sizeof(buf), "%s  %s", keys, action);
    blit_text(buf, x + w / 2, y + h / 2 - 6, m_font_sm, active ? COL_WHITE : SUBTEXT);
}

void Dashboard::render() {
    double speed   = m_state.speed.load();
    double rpm     = m_state.rpm.load();
    double coolant = m_state.coolant_temp.load();
    double fuel    = m_state.fuel_level.load();
    double load    = m_state.engine_load.load();
    double maf     = m_state.maf.load();
    double runtime = m_state.runtime.load();
    uint64_t reqs  = m_metrics.request_count();

    // clear
    set_col(m_renderer, BG);
    SDL_RenderClear(m_renderer);

    // header bar
    SDL_Rect hdr = {0, 0, WIN_W, 42};
    set_col(m_renderer, PANEL);
    SDL_RenderFillRect(m_renderer, &hdr);
    set_col(m_renderer, BORDER);
    SDL_RenderDrawLine(m_renderer, 0, 42, WIN_W, 42);

    blit_text("CANSimulator",   WIN_W / 2, 5,  m_font_md, TEXT);
    blit_text("OBD-II | vcan0", WIN_W / 2, 24, m_font_sm, SUBTEXT);

    // live CAN request counter 
    char rbuf[32];
    snprintf(rbuf, sizeof(rbuf), "CAN req: %llu", (unsigned long long)reqs);
    int rw = 0;
    TTF_SizeText(m_font_sm, rbuf, &rw, nullptr);
    blit_text(rbuf, WIN_W - rw - 18, 14, m_font_sm, SUBTEXT, false);

    // center divider
    set_col(m_renderer, BORDER);
    SDL_RenderDrawLine(m_renderer, 450, 50, 450, 448);

    // gauges
    draw_gauge(SPD_CX, SPD_CY, SPD_R, speed, 200.0,  "SPEED",       "km/h");
    draw_gauge(RPM_CX, RPM_CY, RPM_R, rpm,   7000.0, "ENGINE RPM",  "rpm");

    // throttle % 
    char tbuf[20];
    snprintf(tbuf, sizeof(tbuf), "throttle  %.0f%%", m_throttle);
    blit_text(tbuf, SPD_CX, SPD_CY + 118, m_font_sm, SUBTEXT);

    // stat boxes
    const int sx = 30, sy = 458, sw = 164, sh = 60, gap = 10;
    char buf[24];

    SDL_Color ccol = (coolant > 105) ? COL_RED : (coolant > 95) ? COL_YELLOW : COL_GREEN;
    SDL_Color fcol = (fuel < 15)     ? COL_RED : (fuel < 25)    ? COL_YELLOW : COL_GREEN;

    snprintf(buf, sizeof(buf), "%.0f C", coolant);
    draw_stat(sx + 0 * (sw + gap), sy, sw, sh, "COOLANT", buf, ccol);

    snprintf(buf, sizeof(buf), "%.0f%%", load);
    draw_stat(sx + 1 * (sw + gap), sy, sw, sh, "ENGINE LOAD", buf, TEXT);

    snprintf(buf, sizeof(buf), "%.1f g/s", maf);
    draw_stat(sx + 2 * (sw + gap), sy, sw, sh, "MAF", buf, TEXT);

    snprintf(buf, sizeof(buf), "%.0f%%", fuel);
    draw_stat(sx + 3 * (sw + gap), sy, sw, sh, "FUEL", buf, fcol);

    int secs = (int)runtime;
    snprintf(buf, sizeof(buf), "%d:%02d", secs / 60, secs % 60);
    draw_stat(sx + 4 * (sw + gap), sy, sw, sh, "RUNTIME", buf, TEXT);

    // key hint info
    const int ky = 528;
    draw_key_hint(30,  ky, "W / UP",   "ACCELERATE", m_accel);
    draw_key_hint(230, ky, "S / DOWN", "BRAKE",       m_brake);
    blit_text("Q / ESC  quit", 520, ky + 8, m_font_sm, SUBTEXT, false);

    SDL_RenderPresent(m_renderer);
}

void Dashboard::run() {
    SDL_Event ev;
    bool quit = false;

    while (!quit && m_state.running.load()) {
        Uint32 t0 = SDL_GetTicks();

        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT) { quit = true; break; }
            if (ev.type == SDL_KEYDOWN) {
                switch (ev.key.keysym.sym) {
                    case SDLK_ESCAPE: case SDLK_q:    quit    = true;  break;
                    case SDLK_w:      case SDLK_UP:   m_accel = true;  break;
                    case SDLK_s:      case SDLK_DOWN: m_brake = true;  break;
                    default: break;
                }
            }
            if (ev.type == SDL_KEYUP) {
                switch (ev.key.keysym.sym) {
                    case SDLK_w: case SDLK_UP:   m_accel = false; break;
                    case SDLK_s: case SDLK_DOWN: m_brake = false; break;
                    default: break;
                }
            }
        }

        // adjust throttle based on key state
        if      (m_accel && !m_brake) m_throttle = std::min(m_throttle + 0.5,  100.0);
        else if (m_brake)             m_throttle = std::max(m_throttle - 3.0,    0.0);
        else                          m_throttle = std::max(m_throttle - 0.8,    0.0);

        // no throttle if out of fuel
        if (m_state.fuel_level.load() <= 0.0) m_throttle = 0.0;

        m_state.user_throttle = m_throttle;
        m_state.user_brake    = m_brake;

        render();

        int elapsed = (int)(SDL_GetTicks() - t0);
        if (elapsed < 16) SDL_Delay(16 - elapsed); // cap at ~60 fps
    }

    m_state.running = false;
}
