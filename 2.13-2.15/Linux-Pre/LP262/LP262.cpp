#include <iostream>
#include <vector>
#include <thread>
#include <barrier>
#include <cmath>
#include <atomic>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <deque>
#include <chrono>
#include <iomanip>
#include <mutex>
#include <string>
#include <algorithm>
#include <sstream>
#include <random>
#include <unordered_map>

#undef main

using namespace std;

// --- Constants ---
const double G = 6.67430e-11;
const double AU = 1.496e11;
const double C = 299792458.0;
const double C2 = C * C;
const double dt = 100.0;
const int MAX_TRAIL_POINTS = 8000;
const int HISTORY_LIMIT = 1000;

const double w1 = 1.35120719195966;
const double w0 = -1.70241438391932;
const double c_y[4] = { w1 / 2.0, (w0 + w1) / 2.0, (w0 + w1) / 2.0, w1 / 2.0 };
const double d_y[3] = { w1, w0, w1 };

struct Body {
    string name;
    double mass;
    double x, y, vx, vy, ax, ay;
    SDL_Color color;
};

struct HistoryState {
    int step;
    vector<Body> bodies;
};

struct TrailBuffer {
    vector<pair<double, double>> points;
    int head = 0;
    bool full = false;
    TrailBuffer() { points.resize(MAX_TRAIL_POINTS); }
    void add(double x, double y) {
        points[head] = {x, y};
        head = (head + 1) % MAX_TRAIL_POINTS;
        if (head == 0) full = true;
    }
};

// --- Globals ---
vector<Body> b;
vector<double> shared_x, shared_y;
atomic<bool> quit{ false }, paused{ false }, replay_mode{ false };
atomic<bool> show_details{ true }, show_tags{ true }, show_grid{ true };
atomic<int> current_step{ 0 }, replay_speed{ 1 };
deque<HistoryState> history;
mutex history_mutex;
int replay_index = -1;

// --- Physics ---
void compute_acceleration(int id, int n, double& ax, double& ay) {
    ax = 0.0; ay = 0.0;
    const double soft = 1e7;
    for (int j = 0; j < n; ++j) {
        if (j == id) continue;
        double dx = shared_x[j] - shared_x[id];
        double dy = shared_y[j] - shared_y[id];
        double r2 = dx * dx + dy * dy + soft;
        double r = sqrt(r2);
        double f_newton = G * b[j].mass / (r2 * r);
        if (b[j].mass > 1e29) { 
            double v2 = b[id].vx * b[id].vx + b[id].vy * b[id].vy;
            double r_v = (dx * b[id].vx + dy * b[id].vy);
            double prec = (4.0 * G * b[j].mass / r - v2) / C2;
            ax += f_newton * (dx * (1.0 + prec) + (4.0 * r_v * b[id].vx) / C2);
            ay += f_newton * (dy * (1.0 + prec) + (4.0 * r_v * b[id].vy) / C2);
        } else {
            ax += f_newton * dx; ay += f_newton * dy;
        }
    }
}

void physics_worker_pooled(int thread_id, int num_threads, int n, barrier<>& sync) {
    int start = (n / num_threads) * thread_id;
    int end = (thread_id == num_threads - 1) ? n : (n / num_threads) * (thread_id + 1);
    while (!quit) {
        while ((paused || replay_mode) && !quit) this_thread::sleep_for(chrono::milliseconds(10));
        for (int sub = 0; sub < 20; ++sub) {
            for (int s = 0; s < 3; ++s) {
                for (int i = start; i < end; ++i) {
                    b[i].x += b[i].vx * c_y[s] * dt;
                    b[i].y += b[i].vy * c_y[s] * dt;
                    shared_x[i] = b[i].x; shared_y[i] = b[i].y;
                }
                sync.arrive_and_wait();
                for (int i = start; i < end; ++i) {
                    compute_acceleration(i, n, b[i].ax, b[i].ay);
                    b[i].vx += b[i].ax * d_y[s] * dt;
                    b[i].vy += b[i].ay * d_y[s] * dt;
                }
                sync.arrive_and_wait();
            }
            for (int i = start; i < end; ++i) {
                b[i].x += b[i].vx * c_y[3] * dt;
                b[i].y += b[i].vy * c_y[3] * dt;
                shared_x[i] = b[i].x; shared_y[i] = b[i].y;
            }
            sync.arrive_and_wait();
        }
        if (thread_id == 0) {
            current_step += 20;
            if (current_step % 400 == 0) {
                lock_guard<mutex> lock(history_mutex);
                history.push_back({ (int)current_step, b });
                if (history.size() > HISTORY_LIMIT) history.pop_front();
            }
        }
    }
}

// --- Utils ---
string format_time(double sec) {
    int d = (int)(sec / 86400), h = (int)((sec - d * 86400) / 3600);
    return to_string(d) + "d " + to_string(h) + "h";
}

string format_unit(double v) {
    double a = abs(v);
    if (a >= AU * 0.1) return to_string(v / AU).substr(0, 4) + " AU";
    return a >= 1000.0 ? to_string(v / 1000.0).substr(0, 5) + " km" : to_string(v).substr(0, 5) + " m";
}

string to_sci(double val) {
    stringstream ss; ss << scientific << setprecision(2) << val;
    return ss.str();
}

// --- Render ---
void render_loop() {
    SDL_Init(SDL_INIT_VIDEO); TTF_Init();
    int W = 1400, H = 800;
    auto win = SDL_CreateWindow("Celestial Lab v4.1 - NO CHINESE MODE", 100, 100, W, H, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
    auto ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    TTF_Font* font = TTF_OpenFont("/usr/share/fonts/dejavu-sans-fonts/DejaVuSans.ttf", 12);
    
    unordered_map<string, SDL_Texture*> text_cache;
    double scale = 1e-9, offX = W / 2.0, offY = H / 2.0;
    int follow_id = -1;
    bool dragging = false; int lx = 0, ly = 0;
    vector<TrailBuffer> trails(b.size());

    auto draw_text = [&](string txt, int x, int y, SDL_Color col, bool right = false) {
        if (!font || txt.empty()) return;
        // Logic to detect if the string is dynamic (contains changing numbers or symbols)
        bool is_dynamic = false;
        for(char c : txt) if (isdigit(c) || c == ':' || c == ',' || c == '.') { is_dynamic = true; break; }

        string key = txt + to_string(col.r) + to_string(col.g) + to_string(col.b);
        SDL_Texture* tex = (!is_dynamic && text_cache.count(key)) ? text_cache[key] : nullptr;
        if (!tex) {
            SDL_Surface* s = TTF_RenderText_Blended(font, txt.c_str(), col);
            if (!s) return;
            tex = SDL_CreateTextureFromSurface(ren, s);
            SDL_FreeSurface(s);
            if (!is_dynamic) text_cache[key] = tex;
        }
        int tw, th; SDL_QueryTexture(tex, NULL, NULL, &tw, &th);
        SDL_Rect r = { right ? x - tw : x, y, tw, th };
        SDL_RenderCopy(ren, tex, NULL, &r);
        if (is_dynamic) SDL_DestroyTexture(tex);
    };

    while (!quit) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) quit = true;
            else if (e.type == SDL_WINDOWEVENT && e.window.event == SDL_WINDOWEVENT_RESIZED) { W = e.window.data1; H = e.window.data2; }
            else if (e.type == SDL_MOUSEWHEEL) {
                double old = scale; scale *= (e.wheel.y > 0 ? 1.2 : 0.83);
                int mx, my; SDL_GetMouseState(&mx, &my);
                if (follow_id == -1) { offX = mx - (mx - offX) * (scale / old); offY = my - (my - offY) * (scale / old); }
            }
            else if (e.type == SDL_KEYDOWN) {
                switch (e.key.keysym.sym) {
                    case SDLK_SPACE: paused = !paused; break;
                    case SDLK_g: show_grid = !show_grid; break;
                    case SDLK_t: show_tags = !show_tags; break;
                    case SDLK_d: show_details = !show_details; break;
                    case SDLK_c: for (auto& tr : trails) { tr.head = 0; tr.full = false; } break;
                    case SDLK_EQUALS: replay_speed = min((int)replay_speed + 1, 50); break;
                    case SDLK_MINUS:  replay_speed = max((int)replay_speed - 1, 1); break;
                    case SDLK_TAB: follow_id = (follow_id + 1 >= (int)b.size()) ? -1 : follow_id + 1; break;
                    case SDLK_ESCAPE: follow_id = -1; break;
                    case SDLK_r: 
                        replay_mode = !replay_mode;
                        if (replay_mode) { paused = true; lock_guard<mutex> l(history_mutex); if (!history.empty()) { replay_index = history.size() - 1; b = history[replay_index].bodies; } }
                        else paused = false;
                        break;
                    case SDLK_LEFT:
                        if (replay_mode) { lock_guard<mutex> l(history_mutex); replay_index = max(0, replay_index - (int)replay_speed); b = history[replay_index].bodies; current_step = history[replay_index].step; }
                        break;
                    case SDLK_RIGHT:
                        if (replay_mode) { lock_guard<mutex> l(history_mutex); replay_index = min((int)history.size() - 1, replay_index + (int)replay_speed); b = history[replay_index].bodies; current_step = history[replay_index].step; }
                        break;
                }
            }
            if (e.type == SDL_MOUSEBUTTONDOWN) { dragging = true; SDL_GetMouseState(&lx, &ly); }
            if (e.type == SDL_MOUSEBUTTONUP) dragging = false;
            if (e.type == SDL_MOUSEMOTION && dragging && follow_id == -1) { offX += e.motion.x - lx; offY += e.motion.y - ly; lx = e.motion.x; ly = e.motion.y; }
        }

        if (follow_id != -1) { offX = W / 2.0 - b[follow_id].x * scale; offY = H / 2.0 + b[follow_id].y * scale; }

        SDL_SetRenderDrawColor(ren, 10, 10, 25, 255); SDL_RenderClear(ren);

        if (show_grid) {
            double sw = 120.0 / scale; double mag = pow(10, floor(log10(sw)));
            double step = (sw / mag > 5.0) ? 10.0 * mag : (sw / mag > 2.0 ? 5.0 * mag : 2.0 * mag);
            SDL_SetRenderDrawColor(ren, 40, 40, 60, 255);
            for (double x = ceil((-offX / scale) / step) * step; x * scale + offX <= W; x += step) {
                int sx = (int)(x * scale + offX); SDL_RenderDrawLine(ren, sx, 0, sx, H);
                draw_text(format_unit(x), sx + 5, H - 20, { 100, 100, 130, 255 });
            }
            for (double y = ceil((- (H - offY)) / scale / step) * step; y * scale + (H - offY) <= H + H; y += step) {
                int sy = (int)(-y * scale + offY); if (sy >= 0 && sy <= H) {
                    SDL_RenderDrawLine(ren, 0, sy, W, sy); draw_text(format_unit(y), 5, sy + 2, { 100, 100, 130, 255 });
                }
            }
        }

        for (size_t i = 0; i < b.size(); ++i) {
            if (!paused && !replay_mode) trails[i].add(b[i].x, b[i].y);
            SDL_SetRenderDrawColor(ren, b[i].color.r, b[i].color.g, b[i].color.b, 50);
            int count = trails[i].full ? MAX_TRAIL_POINTS : trails[i].head;
            for (int k = 1; k < count; ++k) {
                if (trails[i].full && k == trails[i].head) continue;
                auto& p1 = trails[i].points[k-1], &p2 = trails[i].points[k];
                if (abs(p2.first - p1.first) < AU)
                    SDL_RenderDrawLine(ren, (int)(p1.first * scale + offX), (int)(-p1.second * scale + offY), (int)(p2.first * scale + offX), (int)(-p2.second * scale + offY));
            }
            int rx = (int)(b[i].x * scale + offX), ry = (int)(-b[i].y * scale + offY);
            SDL_SetRenderDrawColor(ren, b[i].color.r, b[i].color.g, b[i].color.b, 255);
            SDL_Rect rb = { rx - 4, ry - 4, 8, 8 }; SDL_RenderFillRect(ren, &rb);
            if (show_tags) draw_text(b[i].name, rx + 10, ry - 6, { 255, 255, 255, 255 });
        }

        // --- TOP LEFT: TIME & STATUS ---
        draw_text("TIME: " + format_time((double)current_step * dt), 20, 20, { 0, 255, 150, 255 });
        draw_text(replay_mode ? "MODE: REPLAY" : (paused ? "MODE: PAUSED" : "MODE: RUNNING"), 20, 38, { 255, 255, 0, 255 });

        // --- LEFT LEGEND: BODIES ---
        int ly_legend = 70; draw_text("[ SYSTEM BODIES ]", 20, ly_legend, { 150, 150, 150, 255 });
        for (size_t i = 0; i < min((int)b.size(), 18); ++i) {
            ly_legend += 18; draw_text("* " + b[i].name, 20, ly_legend, b[i].color);
        }

        // --- RIGHT PANEL: DETAILED TELEMETRY ---
        if (show_details) {
            int py = 20, col = 0;
            for (const auto& p : b) {
                int cur_x = W - 20 - (col * 210);
                if (py + 90 > H - 120) { py = 20; col++; cur_x = W - 20 - (col * 210); }
                draw_text("[" + p.name + "]", cur_x, py, p.color, true);
                draw_text("M: " + to_sci(p.mass), cur_x, py + 15, {180, 180, 180, 255}, true);
                draw_text("V: " + to_sci(sqrt(p.vx*p.vx + p.vy*p.vy)), cur_x, py + 30, {180, 180, 180, 255}, true);
                draw_text("A: " + to_sci(sqrt(p.ax*p.ax + p.ay*p.ay)), cur_x, py + 45, {140, 140, 140, 255}, true);
                draw_text("P: " + to_sci(p.x) + "," + to_sci(p.y), cur_x, py + 60, {120, 120, 120, 255}, true);
                py += 85; if (col > 3) break;
            }
        }

        // --- BOTTOM LEFT: REPLAY BAR ---
        if (replay_mode && !history.empty()) {
            draw_text("REPLAY STEP: x" + to_string(replay_speed), 20, H - 85, { 255, 100, 255, 255 });
            SDL_Rect bar_bg = { 20, H - 65, 300, 6 }, bar_fg = { 20, H - 65, (int)(300 * (float)replay_index / max(1, (int)history.size()-1)), 6 };
            SDL_SetRenderDrawColor(ren, 60, 60, 80, 255); SDL_RenderFillRect(ren, &bar_bg);
            SDL_SetRenderDrawColor(ren, 255, 100, 255, 255); SDL_RenderFillRect(ren, &bar_fg);
        }

        // --- BOTTOM CONTROLS ---
        string ctrl = "[G] Grid | [C] Clear | [T] Tags | [D] Details | [TAB] Follow | [R] Replay | [SPACE] Pause | [+/-] Speed | [Esc] Free";
        draw_text(ctrl, 20, H - 35, { 150, 150, 150, 255 });
        if (follow_id != -1) draw_text("FOCUS: " + b[follow_id].name, W - 20, H - 35, { 255, 255, 0, 255 }, true);

        SDL_RenderPresent(ren); SDL_Delay(5);
    }
    TTF_CloseFont(font); TTF_Quit(); SDL_Quit();
}

int main() {
    const double v_e = 29780.0, s60 = 0.866, c60 = 0.5;
    b = {
        {"Sun", 1.989e30, 0, 0, 0, 0, 0, 0, {255, 220, 0, 255}},
        {"Mercury", 3.301e23, 0.387 * AU, 0, 0, 47360, 0, 0, {160, 160, 160, 255}},
        {"Venus", 4.867e24, 0.723 * AU, 0, 0, 35020, 0, 0, {255, 230, 150, 255}},
        {"Earth", 5.972e24, -1.0 * AU, 0, 0, -v_e, 0, 0, {50, 150, 255, 255}},
        {"Moon", 7.348e22, -1.00257 * AU, 0, 0, -v_e - 1022, 0, 0, {180, 180, 180, 255}},
        {"Mars", 6.417e23, 1.524 * AU, 0, 0, 24077, 0, 0, {255, 80, 50, 255}},
        {"Jupiter", 1.898e27, 5.203 * AU, 0, 0, 13070, 0, 0, {240, 160, 110, 255}},
        {"Saturn", 5.683e26, 9.537 * AU, 0, 0, 9680, 0, 0, {210, 180, 140, 255}},
        {"Uranus", 8.681e25, 19.19 * AU, 0, 0, 6800, 0, 0, {150, 255, 255, 255}},
        {"Neptune", 1.024e26, 30.07 * AU, 0, 0, 5430, 0, 0, {80, 120, 255, 255}},
        {"Pluto", 1.303e22, 39.48 * AU, 0, 0, 4740, 0, 0, {200, 170, 140, 255}},
        {"Halley", 2.2e14, 35.08 * AU, 0, 0, -910, 0, 0, {255, 255, 255, 255}}
    };
    b.push_back({ "Earth_L4", 10.0, -AU * c60, -AU * s60, v_e * s60, -v_e * c60, 0, 0, {0, 255, 120, 255} });
    b.push_back({ "Earth_L5", 10.0, -AU * c60, AU * s60, -v_e * s60, -v_e * c60, 0, 0, {255, 120, 255, 255} });

    double mvx = 0, mvy = 0;
    for (size_t i = 1; i < b.size(); ++i) { mvx += b[i].mass * b[i].vx; mvy += b[i].mass * b[i].vy; }
    b[0].vx = -mvx / b[0].mass; b[0].vy = -mvy / b[0].mass;

    int n = b.size(), workers_cnt = max(1u, thread::hardware_concurrency());
    shared_x.resize(n); shared_y.resize(n);
    barrier<> sync(workers_cnt);
    vector<thread> workers;
    for (int i = 0; i < workers_cnt; ++i) workers.emplace_back(physics_worker_pooled, i, workers_cnt, n, ref(sync));

    render_loop();
    for (auto& t : workers) t.join();
    return 0;
}