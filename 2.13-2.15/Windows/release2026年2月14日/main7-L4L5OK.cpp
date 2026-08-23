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

#undef main 

using namespace std;

// --- 物理常数 ---
const double G = 6.67430e-11;
const double AU = 1.496e11;
const double dt = 100.0;
const int MAX_TRAIL_POINTS = 8000;
const int HISTORY_LIMIT = 1000;

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

// --- 全局变量 ---
vector<Body> b;
vector<double> shared_x, shared_y;
atomic<bool> quit{ false }, paused{ false }, replay_mode{ false }, ready{ false };
atomic<bool> show_details{ true }, show_tags{ true }, show_grid{ true };
atomic<int> current_step{ 0 }, replay_speed{ 1 };
deque<HistoryState> history;
mutex history_mutex;
int replay_index = -1;

// --- 工具函数 (预先定义) ---

string format_time(double sec) {
    int days = (int)(sec / 86400);
    int hours = (int)((sec - days * 86400) / 3600);
    return to_string(days) + "d " + to_string(hours) + "h";
}

string format_unit(double value) {
    double abs_v = abs(value);
    if (abs_v >= AU * 0.1) return to_string(value / AU).substr(0, 4) + " AU";
    if (abs_v >= 1000.0) return to_string(value / 1000.0).substr(0, 5) + " km";
    return to_string(value).substr(0, 5) + " m";
}

string to_sci(double val) {
    stringstream ss;
    ss << scientific << setprecision(2) << val;
    return ss.str();
}

// --- 物理引擎 ---
void compute_acceleration(int id, int n, double& ax, double& ay) {
    ax = 0.0; ay = 0.0;
    for (int j = 0; j < n; ++j) {
        if (j == id) continue;
        double dx = shared_x[j] - shared_x[id];
        double dy = shared_y[j] - shared_y[id];
        double r2 = dx * dx + dy * dy + 1e7;
        double f = G * b[j].mass / (r2 * sqrt(r2));
        ax += f * dx; ay += f * dy;
    }
}
// 四阶辛积分系数 (Yoshida Coefficients)
const double w1 = 1.35120719195966;
const double w0 = -1.70241438391932;
const double c[4] = { w1 / 2.0, (w0 + w1) / 2.0, (w0 + w1) / 2.0, w1 / 2.0 };
const double d[3] = { w1, w0, w1 };

void physics_worker(int id, int n, barrier<>& sync) {
    for (int step = 0; !quit; ++step) {
        while ((paused || replay_mode) && !quit) this_thread::sleep_for(chrono::milliseconds(10));

        // 四阶辛积分需要 3 次加速度计算 (3 Stages)
        for (int stage = 0; stage < 3; ++stage) {
            // 1. 更新位置 (基于当前阶段系数)
            b[id].x += b[id].vx * c[stage] * dt;
            b[id].y += b[id].vy * c[stage] * dt;
            shared_x[id] = b[id].x;
            shared_y[id] = b[id].y;

            // 2. 同步：确保所有天体位置更新完成
            sync.arrive_and_wait();

            // 3. 计算该子步的加速度
            double cur_ax, cur_ay;
            compute_acceleration(id, n, cur_ax, cur_ay);
            b[id].ax = cur_ax;
            b[id].ay = cur_ay;

            // 4. 更新速度
            b[id].vx += b[id].ax * d[stage] * dt;
            b[id].vy += b[id].ay * d[stage] * dt;

            // 再次同步，准备下一子步
            sync.arrive_and_wait();
        }

        // 最终位置补齐 (最后一个 c4 系数)
        b[id].x += b[id].vx * c[3] * dt;
        b[id].y += b[id].vy * c[3] * dt;
        shared_x[id] = b[id].x;
        shared_y[id] = b[id].y;
        sync.arrive_and_wait();

        // 记录历史
        if (id == 0) {
            current_step = step;
            if (step % 50 == 0) { // 高阶积分可以适当降低记录频率，因为步长 dt 可以更稳
                lock_guard<mutex> lock(history_mutex);
                history.push_back({ (int)step, b });
                if (history.size() > HISTORY_LIMIT) history.pop_front();
            }
        }
    }
}

// --- 渲染循环 (v3.8 完整功能版) ---
void render_loop() {
    SDL_Init(SDL_INIT_VIDEO); TTF_Init();
    int W = 1400, H = 800;
    auto win = SDL_CreateWindow("Celestial Lab v3.8 - Lagrange Study", 100, 100, W, H, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
    auto ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    // 请确保路径下有此字体，或改为你系统中的字体路径
    TTF_Font* font = TTF_OpenFont("C:/Windows/Fonts/arial.ttf", 13);

    double scale = 1e-9, offX = W / 2.0, offY = H / 2.0;
    vector<deque<pair<double, double>>> trails(b.size());
    bool dragging = false; int lx = 0, ly = 0;

    auto draw_text = [&](string text, int x, int y, SDL_Color col, bool right_align = false) {
        if (!font || text.empty()) return;
        SDL_Surface* surf = TTF_RenderText_Blended(font, text.c_str(), col);
        if (!surf) return;
        SDL_Texture* tex = SDL_CreateTextureFromSurface(ren, surf);
        int tx = right_align ? x - surf->w : x;
        SDL_Rect r = { tx, y, surf->w, surf->h };
        SDL_RenderCopy(ren, tex, NULL, &r);
        SDL_FreeSurface(surf); SDL_DestroyTexture(tex);
        };

    while (!quit) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) quit = true;
            if (e.type == SDL_WINDOWEVENT && e.window.event == SDL_WINDOWEVENT_RESIZED) {
                W = e.window.data1; H = e.window.data2;
            }
            if (e.type == SDL_MOUSEWHEEL) {
                double old = scale; scale *= (e.wheel.y > 0 ? 1.2 : 0.83);
                int mx, my; SDL_GetMouseState(&mx, &my);
                offX = mx - (mx - offX) * (scale / old); offY = my - (my - offY) * (scale / old);
            }
            if (e.type == SDL_KEYDOWN) {
                switch (e.key.keysym.sym) {
                case SDLK_SPACE: paused = !paused; break;
                case SDLK_g: show_grid = !show_grid; break;
                case SDLK_t: show_tags = !show_tags; break;
                case SDLK_d: show_details = !show_details; break;
                case SDLK_c: for (auto& tr : trails) tr.clear(); break;
                case SDLK_EQUALS: replay_speed = min((int)replay_speed + 1, 50); break;
                case SDLK_MINUS:  replay_speed = max((int)replay_speed - 1, 1); break;
                case SDLK_r:
                    replay_mode = !replay_mode;
                    if (replay_mode) {
                        paused = true; // 进入回放自动暂停模拟
                        lock_guard<mutex> lock(history_mutex);
                        if (!history.empty()) {
                            replay_index = (int)history.size() - 1;
                            b = history[replay_index].bodies;
                        }
                    }
                    else {
                        paused = false;
                    }
                    break;
                case SDLK_LEFT:
                    if (replay_mode) {
                        lock_guard<mutex> lock(history_mutex);
                        if (!history.empty()) {
                            replay_index = max(0, replay_index - (int)replay_speed);
                            b = history[replay_index].bodies;
                            current_step = history[replay_index].step;
                        }
                    }
                    break;
                case SDLK_RIGHT:
                    if (replay_mode) {
                        lock_guard<mutex> lock(history_mutex);
                        if (!history.empty()) {
                            replay_index = min((int)history.size() - 1, replay_index + (int)replay_speed);
                            b = history[replay_index].bodies;
                            current_step = history[replay_index].step;
                        }
                    }
                    break;
                }
            }
            if (e.type == SDL_MOUSEBUTTONDOWN) { dragging = true; SDL_GetMouseState(&lx, &ly); }
            if (e.type == SDL_MOUSEBUTTONUP) dragging = false;
            if (e.type == SDL_MOUSEMOTION && dragging) { offX += e.motion.x - lx; offY += e.motion.y - ly; lx = e.motion.x; ly = e.motion.y; }
        }

        // --- 开始绘制背景 ---
        SDL_SetRenderDrawColor(ren, 10, 10, 25, 255);
        SDL_RenderClear(ren);

        // --- 1. 动态网格与坐标轴标注 ---
        if (show_grid) {
            double raw_step = 120.0 / scale;
            double mag = pow(10, floor(log10(raw_step)));
            double step_world = (raw_step / mag > 5.0) ? 10.0 * mag : (raw_step / mag > 2.0 ? 5.0 * mag : 2.0 * mag);

            SDL_SetRenderDrawColor(ren, 50, 50, 70, 255);
            // 垂直线
            for (double x = ceil(((0 - offX) / scale) / step_world) * step_world; x * scale + offX <= W; x += step_world) {
                int sx = (int)(x * scale + offX);
                SDL_RenderDrawLine(ren, sx, 0, sx, H);
                draw_text(format_unit(x), sx + 5, H - 20, { 120, 120, 150, 255 });
            }
            // 水平线
            for (double y = ceil(((0 - offY) / scale) / step_world) * step_world; y * scale + offY <= H; y += step_world) {
                int sy = (int)(y * scale + offY);
                SDL_RenderDrawLine(ren, 0, sy, W, sy);
                draw_text(format_unit(y), 5, sy + 2, { 120, 120, 150, 255 });
            }
        }

        // --- 2. 轨迹与天体渲染 ---
        // --- 2. 轨迹与天体渲染 (修复直线拖迹版) ---
        for (size_t i = 0; i < b.size(); ++i) {
            // 关键修复：只有在【非暂停】且【非回放】模式下才记录轨迹点
            if (!paused && !replay_mode) {
                trails[i].push_back({ b[i].x, b[i].y });
                if (trails[i].size() > MAX_TRAIL_POINTS) trails[i].pop_front();
            }

            // 绘制轨迹：在回放模式下，我们通常只想看天体的实时位置，
            // 如果你想在回放时也看到轨迹，可以保持这段代码；
            // 如果回放时轨迹太乱，可以将下面这段放在 if (!replay_mode) 里。
            SDL_SetRenderDrawColor(ren, b[i].color.r, b[i].color.g, b[i].color.b, 80);
            for (size_t k = 1; k < trails[i].size(); ++k) {
                // 检查两个点之间的距离，如果距离过大（如回放跳变），则不连线
                double dist_sq = pow(trails[i][k].first - trails[i][k - 1].first, 2) +
                    pow(trails[i][k].second - trails[i][k - 1].second, 2);
                if (dist_sq < pow(AU * 0.5, 2)) { // 距离超过 0.5 AU 则判定为跳变，不画线
                    SDL_RenderDrawLine(ren,
                        (int)(trails[i][k - 1].first * scale + offX), (int)(trails[i][k - 1].second * scale + offY),
                        (int)(trails[i][k].first * scale + offX), (int)(trails[i][k].second * scale + offY));
                }
            }

            // 天体实体绘制
            int rx = (int)(b[i].x * scale + offX);
            int ry = (int)(b[i].y * scale + offY);
            SDL_SetRenderDrawColor(ren, b[i].color.r, b[i].color.g, b[i].color.b, 255);
            SDL_Rect rb = { rx - 5, ry - 5, 10, 10 };
            SDL_RenderFillRect(ren, &rb);

            if (show_tags) {
                draw_text(b[i].name, rx + 12, ry - 8, { 255, 255, 255, 255 });
            }
        }

        // --- 3. UI 叠加层 ---
        // A. 左上角：时间、状态与图例
        draw_text("TIME: " + format_time((double)current_step * dt), 20, 20, { 0, 255, 150, 255 });
        draw_text(replay_mode ? "STATUS: REPLAYING" : (paused ? "STATUS: PAUSED" : "STATUS: RUNNING"), 20, 40, { 255, 255, 0, 255 });

        int legend_y = 75;
        draw_text("[ LIST OF BODIES ]", 20, legend_y, { 150, 150, 150, 255 });
        for (auto& body : b) {
            legend_y += 20;
            draw_text("-> " + body.name, 20, legend_y, body.color);
        }

        // B. 右上角：物理详细参数 (由 D 键控制)
        if (show_details) {
            int py = 20;
            for (auto& p : b) {
                draw_text("[" + p.name + "]", W - 20, py, p.color, true);
                draw_text("P: " + to_sci(p.x) + "," + to_sci(p.y), W - 20, py + 18, { 200,200,200,255 }, true);
                draw_text("V: " + to_sci(sqrt(p.vx * p.vx + p.vy * p.vy)) + " m/s", W - 20, py + 34, { 200,200,200,255 }, true);
                draw_text("A: " + to_sci(sqrt(p.ax * p.ax + p.ay * p.ay)) + " m/s2", W - 20, py + 50, { 180,180,180,255 }, true);
                py += 80;
            }
        }

        // C. 左下角：操作指南
        draw_text("[G] Grid | [C] Clear Trails | [T] Tags | [D] Details | [R] Replay | [SPACE] Pause", 20, H - 40, { 150, 150, 150, 255 });

        // D. 回放进度控制条
        if (replay_mode && !history.empty()) {
            draw_text("REPLAY SPEED: x" + to_string(replay_speed) + " | [+ / -] Adjust", 20, H - 90, { 255, 100, 255, 255 });
            SDL_Rect bar_bg = { 20, H - 65, 350, 8 };
            SDL_SetRenderDrawColor(ren, 60, 60, 80, 255);
            SDL_RenderFillRect(ren, &bar_bg);

            float progress = (float)replay_index / (float)max(1, (int)history.size() - 1);
            SDL_Rect bar_fg = { 20, H - 65, (int)(350 * progress), 8 };
            SDL_SetRenderDrawColor(ren, 255, 100, 255, 255);
            SDL_RenderFillRect(ren, &bar_fg);

            draw_text("Seek: [LEFT / RIGHT] Arrow Keys", 385, H - 70, { 220, 220, 220, 255 });
        }

        SDL_RenderPresent(ren);
        SDL_Delay(5);
    }

    if (font) TTF_CloseFont(font);
    TTF_Quit(); SDL_Quit();
}

int main() {
    double v_e = 29780.0;
    double dist_e = AU;
    double dist_m = 3.844e8;
    double v_m_rel = 1022.0;
    double sin60 = 0.8660254;
    double cos60 = 0.5;

    b = {
        {"Sun",   1.989e30, 0, 0, 0, 0, 0, 0, {255, 200, 0, 255}},
        {"Earth", 5.972e24, -dist_e, 0, 0, -v_e, 0, 0, {100, 180, 255, 255}},
        {"Moon",  7.348e22, -dist_e - dist_m, 0, 0, -v_e + v_m_rel, 0, 0, {180, 180, 180, 255}},
        // L4 
        {"L4", 10.0, -dist_e * cos60, -dist_e * sin60, v_e * sin60, -v_e * cos60, 0, 0, {0, 255, 100, 255}},
        // L5
        {"L5", 10.0, -dist_e * cos60,  dist_e * sin60, -v_e * sin60, -v_e * cos60, 0, 0, {255, 100, 255, 255}}
    };

    // 再次强调：必须执行质心修正，否则加入月球后系统漂移会更快
    double m_total = 0, mvx = 0, mvy = 0;
    for (auto& p : b) { m_total += p.mass; mvx += p.mass * p.vx; mvy += p.mass * p.vy; }
    b[0].vx -= mvx / b[0].mass;
    b[0].vy -= mvy / b[0].mass;

    // --- 质心修正逻辑 (防止系统飘走) ---
    double total_mvx = 0, total_mvy = 0, total_mass = 0;
    for (auto& body : b) {
        total_mvx += body.mass * body.vx;
        total_mvy += body.mass * body.vy;
        total_mass += body.mass;
    }
    // 让太阳承担反向动量，使总动量为 0
    b[0].vx -= total_mvx / b[0].mass;
    b[0].vy -= total_mvy / b[0].mass;
    int n = (int)b.size();
    shared_x.resize(n); shared_y.resize(n);
    barrier<> sync(n);
    vector<thread> workers;
    for (int i = 0; i < n; ++i) workers.emplace_back(physics_worker, i, n, ref(sync));
    thread renderer(render_loop);
    for (auto& t : workers) t.join(); renderer.join();
    return 0;
}