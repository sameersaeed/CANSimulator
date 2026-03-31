#include <algorithm>
#include <cmath>
#include <iomanip>
#include <ios>
#include <stdexcept>
#include <string>
#include <thread>

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include "Dashboard.hpp"
#include "OBDClient.hpp"

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

static inline void setCol(SDL_Renderer* r, SDL_Color c) {
    SDL_SetRenderDrawColor(r, c.r, c.g, c.b, c.a);
}

auto formatLabel = [](double val, bool useK) -> std::string {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(0);

    // toggle K suffix for vals >= 1000, if enabled
    if (useK && val >= 1000.0) oss << (val / 1000.0) << "k";
    else               oss << val;

    return oss.str();
};

// gauge arc created by stacking concentric arcs
static void drawArc(SDL_Renderer* r, int cx, int cy, int radius, double startDeg, double endDeg, SDL_Color col, int thick = 7) {
    setCol(r, col);
    const double step = 0.4;

    for (int dr = -(thick / 2); dr <= thick / 2; ++dr) {
        double rr = radius + dr;
        double px = cx + rr * std::cos(startDeg * M_PI / 180.0);
        double py = cy + rr * std::sin(startDeg * M_PI / 180.0);
        
        for (double a = startDeg + step; a <= endDeg + step * 0.5; a += step) {
            double rad = a * M_PI / 180.0;
            double nx  = cx + rr * std::cos(rad);
            double ny  = cy + rr * std::sin(rad);

            SDL_RenderDrawLine(r, static_cast<int>(px), static_cast<int>(py), static_cast<int>(nx), static_cast<int>(ny));
            px = nx;
            py = ny;
        }
    }
}

static void drawFilledCircle(SDL_Renderer* r, int cx, int cy, int radius, SDL_Color col) {
    setCol(r, col);
    for (int dy = -radius; dy <= radius; ++dy) {
        int dx = static_cast<int>(std::sqrt(static_cast<double>(radius * radius - dy * dy)));
        SDL_RenderDrawLine(r, cx - dx, cy + dy, cx + dx, cy + dy);
    }
}

// needle from center toward arc edge
static void drawNeedle(SDL_Renderer* r, int cx, int cy, int length, double angleDeg, SDL_Color col) {
    double rad  = angleDeg * M_PI / 180.0;
    double perp = rad + M_PI / 2.0;

    int ex = cx + static_cast<int>(length * std::cos(rad));
    int ey = cy + static_cast<int>(length * std::sin(rad));
    
    setCol(r, col);
    for (int i = -2; i <= 2; ++i) {
        int ox = static_cast<int>(i * std::cos(perp));
        int oy = static_cast<int>(i * std::sin(perp));
        SDL_RenderDrawLine(r, cx + ox, cy + oy, ex + ox, ey + oy);
    }
}

// try common Linux system font paths for the text
static TTF_Font* tryLoadFont(int pt) {
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

Dashboard::Dashboard(VehicleState& state, Metrics& metrics) : m_state(state), m_metrics(metrics) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) throw std::runtime_error(std::string("SDL_Init: ") + SDL_GetError());
    if (TTF_Init() < 0) throw std::runtime_error(std::string("TTF_Init: ") + TTF_GetError());

    m_window = SDL_CreateWindow("CANSimulator - OBD-II Dashboard", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WIN_W, WIN_H, SDL_WINDOW_SHOWN);
    if (!m_window) throw std::runtime_error(std::string("CreateWindow: ") + SDL_GetError());

    m_renderer = SDL_CreateRenderer(m_window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!m_renderer) throw std::runtime_error(std::string("CreateRenderer: ") + SDL_GetError());

    SDL_SetRenderDrawBlendMode(m_renderer, SDL_BLENDMODE_BLEND);

    m_fontLg = tryLoadFont(40);
    m_fontMd = tryLoadFont(17);
    m_fontSm = tryLoadFont(13);

    if (!m_fontLg || !m_fontMd || !m_fontSm) throw std::runtime_error("no system font found - run: sudo apt install fonts-dejavu-core");

    m_pollerRunning = true;
    m_poller = std::thread([this]{ pollerLoop(); });
}

Dashboard::~Dashboard() {
    m_pollerRunning = false;
    if (m_poller.joinable()) m_poller.join();

    if (m_fontSm) TTF_CloseFont(m_fontSm);
    if (m_fontMd) TTF_CloseFont(m_fontMd);
    if (m_fontLg) TTF_CloseFont(m_fontLg);
    if (m_renderer) SDL_DestroyRenderer(m_renderer);
    if (m_window)   SDL_DestroyWindow(m_window);

    TTF_Quit();
    SDL_Quit();
}

// queries RPM via OBDClient every ~100ms so the CAN req counter increments
void Dashboard::pollerLoop() {
    using namespace std::chrono_literals;
    OBDClient client;

    while (m_pollerRunning.load() && m_state.running.load()) {
        client.query(PID::RPM, 0.2); // 0.2 sec timeout
        std::this_thread::sleep_for(100ms);
    }
}

void Dashboard::blitText(const std::string& s, int x, int y, TTF_Font* font, SDL_Color col, bool centerX) {
    if (!font || s.empty()) return;

    SDL_Surface* surf = TTF_RenderText_Blended(font, s.c_str(), col);
    if (!surf) return;

    SDL_Texture* tex = SDL_CreateTextureFromSurface(m_renderer, surf);
    SDL_Rect dst = {centerX ? x - surf->w / 2 : x, y, surf->w, surf->h};

    SDL_RenderCopy(m_renderer, tex, nullptr, &dst);
    SDL_FreeSurface(surf);
    SDL_DestroyTexture(tex);
}

void Dashboard::drawGauge(int cx, int cy, int r, double value, double maxVal, const char* label, const char* unit) {
    // full background track
    drawArc(m_renderer, cx, cy, r, ARC_START, ARC_START + ARC_SWEEP, DARK_ARC, 9);

    // color value arc, green -> yellow -> red
    double frac = std::clamp(value / maxVal, 0.0, 1.0);
    if (frac > 0.001) {
        SDL_Color arcCol = (frac < 0.70) ? COL_GREEN : (frac < 0.88) ? COL_YELLOW : COL_RED;
        drawArc(m_renderer, cx, cy, r, ARC_START, ARC_START + frac * ARC_SWEEP, arcCol, 9);
    }

    // tick marks
    for (int i = 0; i <= 20; ++i) {
        bool   major = (i % 5 == 0); // on interval of 5
        double arad  = (ARC_START + (i / 20.0) * ARC_SWEEP) * M_PI / 180.0;
        
        int    rOut = r + 5;
        int    rIn  = major ? r - 18 : r - 8;

        setCol(m_renderer, major ? TEXT : BORDER);
        SDL_RenderDrawLine(m_renderer,
            cx + static_cast<int>(rIn  * std::cos(arad)), cy + static_cast<int>(rIn  * std::sin(arad)),
            cx + static_cast<int>(rOut * std::cos(arad)), cy + static_cast<int>(rOut * std::sin(arad)));

        if (major) {
            SDL_RenderDrawLine(m_renderer,
                cx + static_cast<int>((rIn + 1) * std::cos(arad)), cy + static_cast<int>((rIn + 1) * std::sin(arad)),
                cx + static_cast<int>((rOut + 1) * std::cos(arad)), cy + static_cast<int>((rOut + 1) * std::sin(arad)));
        }
    }

    // labels at 0%, 50%, and 100% of range
    const double labelFracs[] = {0.0, 0.5, 1.0};
    for (double lf : labelFracs) {
        double arad = (ARC_START + lf * ARC_SWEEP) * M_PI / 180.0;
        double lval = lf * maxVal;

        int lx = cx + static_cast<int>((r - 32) * std::cos(arad));
        int ly = cy + static_cast<int>((r - 32) * std::sin(arad));

        blitText(formatLabel(lval, true), lx, ly - 7, m_fontSm, SUBTEXT);
    }

    // needle
    drawNeedle(m_renderer, cx, cy, r - 16, ARC_START + frac * ARC_SWEEP, COL_WHITE);

    // center cap
    drawFilledCircle(m_renderer, cx, cy, 14, PANEL);
    drawFilledCircle(m_renderer, cx, cy,  7, COL_WHITE);

    // large val in center
    bool isRPM = (strcmp(label, Labels::ENGINE_RPM) == 0); // show full value for RPM
    blitText(formatLabel(value, !isRPM), cx, cy + 48, m_fontLg, TEXT);
    blitText(unit,  cx, cy + 95,  m_fontSm, SUBTEXT);
    blitText(label, cx, cy - 88, m_fontSm, SUBTEXT);
}

void Dashboard::drawStat(int x, int y, int w, int h, const char* label, const std::string& value, SDL_Color vcol) {
    SDL_Rect bg = {x, y, w, h};
    
    setCol(m_renderer, PANEL);
    SDL_RenderFillRect(m_renderer, &bg);
    
    setCol(m_renderer, BORDER);
    SDL_RenderDrawRect(m_renderer, &bg);

    blitText(label, x + w / 2, y + 8,        m_fontSm, SUBTEXT);
    blitText(value, x + w / 2, y + h / 2 + 5, m_fontMd, vcol);
}

void Dashboard::drawKeyHint(int x, int y, const char* keys, const char* action, bool active) {
    const int w = 190, h = 30;
    SDL_Rect bg = {x, y, w, h};
    
    setCol(m_renderer, active ? COL_BLUE : PANEL);
    SDL_RenderFillRect(m_renderer, &bg);
    
    setCol(m_renderer, active ? COL_BLUE : BORDER);
    SDL_RenderDrawRect(m_renderer, &bg);

    blitText(std::string(keys) + "  " + action, x + w / 2, y + h / 2 - 6, m_fontSm, active ? COL_WHITE : SUBTEXT);
}

void Dashboard::render() {
    double speed   = m_state.speed.load();
    double rpm     = m_state.rpm.load();
    double coolant = m_state.coolantTemp.load();
    double fuel    = m_state.fuelLevel.load();
    double load    = m_state.engineLoad.load();
    double maf     = m_state.maf.load();
    double runtime = m_state.runtime.load();
    uint64_t reqs  = m_metrics.requestCount();

    // clear
    setCol(m_renderer, BG);
    SDL_RenderClear(m_renderer);

    // header bar
    SDL_Rect hdr = {0, 0, WIN_W, 42};
    setCol(m_renderer, PANEL);
    SDL_RenderFillRect(m_renderer, &hdr);
    setCol(m_renderer, BORDER);
    SDL_RenderDrawLine(m_renderer, 0, 42, WIN_W, 42);

    blitText("CANSimulator",   WIN_W / 2, 5,  m_fontMd, TEXT);
    blitText("OBD-II | vcan0", WIN_W / 2, 24, m_fontSm, SUBTEXT);

    // live CAN request counter 
    std::string CANReqCtr = "CAN req: " + std::to_string(reqs);

    int rw = 0;
    TTF_SizeText(m_fontSm, CANReqCtr.c_str(), &rw, nullptr);
    blitText(CANReqCtr, WIN_W - rw - 18, 14, m_fontSm, SUBTEXT, false);

    // center divider
    setCol(m_renderer, BORDER);
    SDL_RenderDrawLine(m_renderer, 450, 50, 450, 448);

    // gauges
    drawGauge(SPD_CX, SPD_CY, SPD_R, speed, 200.0,  Labels::SPEED,       "km/h");
    drawGauge(RPM_CX, RPM_CY, RPM_R, rpm,   7000.0, Labels::ENGINE_RPM,  "rpm");

    // throttle % 
    blitText("throttle  " + formatLabel(m_throttle, false) + "%", SPD_CX, SPD_CY + 118, m_fontSm, SUBTEXT);

    // stat boxes
    const int sx = 30, sy = 458, sw = 164, sh = 60, gap = 10;

    SDL_Color ccol = (coolant > 105) ? COL_RED : (coolant > 95) ? COL_YELLOW : COL_GREEN;
    SDL_Color fcol = (fuel < 15)     ? COL_RED : (fuel < 25)    ? COL_YELLOW : COL_GREEN;

    drawStat(sx + 0 * (sw + gap), sy, sw, sh, Labels::COOLANT, formatLabel(coolant, false) + " C", ccol);

    drawStat(sx + 1 * (sw + gap), sy, sw, sh, Labels::ENGINE_LOAD, formatLabel(load, false) + "%", TEXT);

    std::ostringstream sMaf;
    sMaf << std::fixed << std::setprecision(1) << maf << " g/s";
    drawStat(sx + 2 * (sw + gap), sy, sw, sh, Labels::MAF, sMaf.str(), TEXT);

    drawStat(sx + 3 * (sw + gap), sy, sw, sh, Labels::FUEL, formatLabel(fuel, false) + "%", fcol);

    int secs = static_cast<int>(runtime);
    std::ostringstream sRun;
    sRun << (secs / 60) << ":" << std::setfill('0') << std::setw(2) << (secs % 60);
    drawStat(sx + 4 * (sw + gap), sy, sw, sh, Labels::RUNTIME, sRun.str(), TEXT);

    // key hint info
    const int ky = 528;
    drawKeyHint(30,  ky, "W / UP",   "ACCELERATE", m_accel);
    drawKeyHint(230, ky, "S / DOWN", "BRAKE",       m_brake);
    blitText("Q / ESC  quit", 520, ky + 8, m_fontSm, SUBTEXT, false);

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
        if (m_state.fuelLevel.load() <= 0.0) m_throttle = 0.0;

        m_state.userThrottle = m_throttle;
        m_state.userBrake    = m_brake;

        render();

        int elapsed = static_cast<int>(SDL_GetTicks() - t0);
        if (elapsed < 16) SDL_Delay(16 - elapsed); // cap at ~60 fps
    }

    m_state.running = false;
}
