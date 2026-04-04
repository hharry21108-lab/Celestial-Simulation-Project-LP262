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

// --- 物理常数 ---
const double G = 6.67430e-11;
const double AU = 1.496e11;
const double dt = 10.0;
const int MAX_TRAIL_POINTS = 8000;
const int HISTORY_LIMIT = 1000;

enum ControlMode { STABLE, THRUSTING, MANEUVERING };
enum ThrustType { PROGRADE, RETROGRADE, RADIAL_IN, RADIAL_OUT, CUSTOM_VECTOR };

struct Maneuver {
    double startTime;    // 触发时间
    double duration;     // 持续时间
    double force;        // 推力 (N)
    ThrustType type;     // 变轨类型
    double custom_dx, custom_dy; 

    // 显式构造函数：修复参数推导错误
    Maneuver(double s, double d, double f, ThrustType t, double dx = 0, double dy = 0)
        : startTime(s), duration(d), force(f), type(t), custom_dx(dx), custom_dy(dy) {}
};
struct Body {
    string name;
    double mass;
    double x, y, vx, vy, ax, ay;
    SDL_Color color;
    
    // --- 新增人造天体属性 ---
    bool is_spacecraft = false;
    double fuel_mass = 0.0;      // 燃料质量
    double dry_mass = 0.0;       // 净重
    double isp = 3000.0;         // 比冲 (s), 离子推进器约为 3000, 化学能约为 300
    double thrust_force = 0.0;   // 推力 (N)
    ControlMode mode = STABLE;
    double target_x, target_y;   // 变轨目标
    deque<Maneuver> plan;
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
// 确保 MAX_TRAIL_POINTS 已定义，例如：const int MAX_TRAIL_POINTS = 8000;

struct TrailBuffer {
    vector<pair<double, double>> points;
    int head = 0;
    bool full = false;
    TrailBuffer() { points.resize(MAX_TRAIL_POINTS); }

    void add(double x, double y) {
        if (points.size() != MAX_TRAIL_POINTS) points.resize(MAX_TRAIL_POINTS);
        points[head] = {x, y};
        head = (head + 1) % MAX_TRAIL_POINTS;
        if (head == 0) full = true;
    }
};
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



// --- 物理引擎函数实现 ---

// --- 核心物理函数声明 ---
void compute_acceleration(int id, int n, double& ax, double& ay);
void apply_maneuvers(Body& ship, double currentTime, double delta_t);
void advanced_mission_control(Body& ship, double delta_t);
void apply_spacecraft_physics(Body& ship, double delta_t);
// --- 1. 基础引力计算：多体扰动模型 ---
void compute_acceleration(int id, int n, double& ax, double& ay) {
    ax = 0.0; ay = 0.0;
    const double soft = 1e7; // 软化因子，防止近距离数值爆炸
    for (int j = 0; j < n; ++j) {
        if (j == id) continue;
        double dx = shared_x[j] - shared_x[id];
        double dy = shared_y[j] - shared_y[id];
        double r2 = dx * dx + dy * dy + soft;
        double invR3 = 1.0 / (r2 * sqrt(r2));
        double f = G * b[j].mass * invR3; 
        ax += f * dx; 
        ay += f * dy;
    }
}

// --- 2. 局部参考系寻址：实现自主捕获的关键 ---
// 航天器需要知道当前谁对它影响最大，才能针对该天体进行变轨
int find_primary_body(const Body& ship) {
    int primary = 0; // 默认为太阳
    double max_influence = 0;
    for (int j = 0; j < (int)b.size(); ++j) {
        if (b[j].name == ship.name || b[j].mass < 1e20) continue; 
        double r2 = pow(b[j].x - ship.x, 2) + pow(b[j].y - ship.y, 2);
        // 计算引力势能的影响力 U = M/r
        double influence = b[j].mass / sqrt(r2);
        if (influence > max_influence) {
            max_influence = influence;
            primary = j;
        }
    }
    return primary;
}

// --- 3. 自动导航大脑：PID 轨道动力学控制器 ---
void advanced_mission_control(Body& ship, double delta_t) {
    if (ship.mode != MANEUVERING) return;

    // A. 动态确定参考系 (比如靠近木星时，自动切换到木星参考系)
    int target_body_idx = find_primary_body(ship);
    const Body& center = b[target_body_idx];

    // B. 计算相对于中心天体的状态向量
    double rel_x = ship.x - center.x;
    double rel_y = ship.y - center.y;
    double rel_vx = ship.vx - center.vx;
    double rel_vy = ship.vy - center.vy;
    
    double r = sqrt(rel_x * rel_x + rel_y * rel_y);
    double v_current = sqrt(rel_vx * rel_vx + rel_vy * rel_vy);

    // C. 轨道捕获与圆化逻辑：计算目标圆轨道速度 v = sqrt(GM/r)
    double v_target = sqrt(G * center.mass / r);
    
    // 计算离心率 e (决定轨道形状，e=0为圆，e<1为椭圆，e>=1为逃逸)
    // 简化版 e 判定：基于当前动能与势能比
    double v_error = v_target - v_current;

    // D. PID 推力决策
    if (abs(v_error) > 0.05) { // 5cm/s 精度门限
        // 推力大小与速度偏差成正比 (P分量)
        ship.thrust_force = min(abs(v_error) * 1500.0, 2000.0); 
    } else {
        ship.thrust_force = 0.0;
        // 如果已经达到了圆轨道速度且处于闭合轨道，则稳定
        if (abs(v_error) < 0.01) ship.mode = STABLE;
    }
}

// --- 4. 物理执行层：全向推力注入与 VNC 坐标系 ---
void apply_spacecraft_physics(Body& ship, double delta_t) {
    if (!ship.is_spacecraft || ship.fuel_mass <= 0 || ship.thrust_force <= 0) return;

    // 计算推力产生的加速度矢量
    // (此处简化为使用已在 apply_maneuvers 中确定的 ux, uy)
    // 注意：推力加速度应累加到当前的 ax, ay 上
    double a_mag = ship.thrust_force / ship.mass;
    
    // 消耗燃料
    const double g0 = 9.80665;
    double dm = (ship.thrust_force / (ship.isp * g0)) * abs(delta_t);
    ship.fuel_mass -= dm;
    ship.mass -= dm;

    // 状态切换 UI 反馈
    ship.mode = THRUSTING; 
}
// --- 5. 指令队列处理器：执行预设变轨指令 ---
void apply_maneuvers(Body& ship, double currentTime, double delta_t) {
    if (ship.plan.empty()) return;
    Maneuver& m = ship.plan.front();
    
    if (currentTime < m.startTime) return;
    if (currentTime > m.startTime + m.duration) {
        ship.plan.pop_front();
        return;
    }

    // 建立 VNC 局部坐标系
    double v_mag = sqrt(ship.vx * ship.vx + ship.vy * ship.vy);
    double r_mag = sqrt(ship.x * ship.x + ship.y * ship.y);
    if (v_mag < 1e-6) return;

    double ux = 0, uy = 0;
    if (m.type == PROGRADE) { ux = ship.vx / v_mag; uy = ship.vy / v_mag; }
    else if (m.type == RETROGRADE) { ux = -ship.vx / v_mag; uy = -ship.vy / v_mag; }
    else if (m.type == RADIAL_OUT) { ux = ship.x / r_mag; uy = ship.y / r_mag; }
    else if (m.type == RADIAL_IN) { ux = -ship.x / r_mag; uy = -ship.y / r_mag; }

    double dm = (m.force / (ship.isp * 9.80665)) * abs(delta_t);
    if (ship.fuel_mass > dm) {
        ship.fuel_mass -= dm;
        ship.mass -= dm;
        ship.ax += (ux * m.force) / ship.mass;
        ship.ay += (uy * m.force) / ship.mass;
    }
}
// --- 四阶辛积分 (Yoshida) 核心系数 ---
const double w1 = 1.35120719195966;
const double w0 = -1.70241438391932;
// 位置更新权重
const double c_y[4] = { w1 / 2.0, (w0 + w1) / 2.0, (w0 + w1) / 2.0, w1 / 2.0 };
// 速度/加速度更新权重
const double d_y[3] = { w1, w0, w1 };
// --- 6. 多线程物理工作循环 (吉田四阶辛算法) ---
// --- 物理引擎核心循环 ---// --- 2. 核心物理工作循环 (修复 i 和 sub_dt 未定义) ---
void physics_worker_pooled(int thread_id, int num_threads, int n, barrier<>& sync) {
    int start_idx = (n / num_threads) * thread_id;
    int end_idx = (thread_id == num_threads - 1) ? n : (n / num_threads) * (thread_id + 1);

    for (int step = 0; !quit; ++step) {
        while ((paused || replay_mode) && !quit) {
            this_thread::sleep_for(chrono::milliseconds(10));
        }

        for (int sub = 0; sub < 200; ++sub) {
            // 在子步循环开始时获取当前模拟总时间
            double currentTime = current_step * dt; 

            for (int stage = 0; stage < 3; ++stage) {
                // 定义当前辛积分阶段的时间步长权重 (修复 sub_dt 未定义)
                double sub_dt = d_y[stage] * dt;

                // Stage A: 更新位置 (Drift)
                for (int i = start_idx; i < end_idx; ++i) {
                    b[i].x += b[i].vx * c_y[stage] * dt;
                    b[i].y += b[i].vy * c_y[stage] * dt;
                    shared_x[i] = b[i].x;
                    shared_y[i] = b[i].y;
                }
                sync.arrive_and_wait();

                // Stage B: 计算引力加速度并注入推力 (Kick)
                for (int i = start_idx; i < end_idx; ++i) {
                    double cur_ax, cur_ay;
                    compute_acceleration(i, n, cur_ax, cur_ay);
                    
                    // 将计算出的引力加速度赋值给天体
                    b[i].ax = cur_ax; 
                    b[i].ay = cur_ay;

                    if (b[i].is_spacecraft) {
                        // 【核心修复】：在计算速度更新前，执行变轨和推进逻辑
                        // apply_maneuvers 会根据任务类型直接修改 b[i].ax 和 b[i].ay
                        apply_maneuvers(b[i], currentTime, sub_dt);
                        advanced_mission_control(b[i], sub_dt); 
                        apply_spacecraft_physics(b[i], sub_dt);
                    }

                    // 更新速度：包含引力 + 推进系统的总加速度
                    b[i].vx += b[i].ax * sub_dt;
                    b[i].vy += b[i].ay * sub_dt;
                }
                sync.arrive_and_wait();
            }
            
            // Stage C: 最终位置校正
            for (int i = start_idx; i < end_idx; ++i) {
                b[i].x += b[i].vx * c_y[3] * dt;
                b[i].y += b[i].vy * c_y[3] * dt;
                shared_x[i] = b[i].x;
                shared_y[i] = b[i].y;
            }
            sync.arrive_and_wait();
        }

        if (thread_id == 0) {
            current_step += 200; 
            lock_guard<mutex> lock(history_mutex);
            history.push_back({ (int)current_step, b });
            if (history.size() > HISTORY_LIMIT) history.pop_front();
        }
    }
}// 1. 在 render_loop 开头定义缓存
unordered_map<string, SDL_Texture*> text_cache;
// --- 渲染循环 (v4.0 坐标镜像与 UI 自动优化版) ---
void render_loop() {
    SDL_Init(SDL_INIT_VIDEO); TTF_Init();
    int W = 1400, H = 800;
    auto win = SDL_CreateWindow("Celestial Lab v4.0 - Lagrange Study", 100, 100, W, H, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
    auto ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    // Fedora 中 DejaVuSans 的典型路径
    TTF_Font* font = TTF_OpenFont("/usr/share/fonts/dejavu-sans-fonts/DejaVuSans.ttf", 13);

    double scale = 1e-9, offX = W / 2.0, offY = H / 2.0;
    int follow_id = -1; // -1: 自由视角, 0+: 锁定天体索引
    bool dragging = false; int lx = 0, ly = 0;



    auto draw_text = [&](string text, int x, int y, SDL_Color col, bool right_align = false) {
        if (!font || text.empty()) return;
        
        // 只有行星名字等静态文本才缓存，动态数值（如坐标）不缓存，防止内存溢出
        bool is_dynamic = (text.find('.') != string::npos || text.find(':') != string::npos);
        string key = text + to_string(col.r) + to_string(col.g) + to_string(col.b);

        SDL_Texture* tex = nullptr;
        if (!is_dynamic && text_cache.count(key)) {
            tex = text_cache[key];
        } else {
            SDL_Surface* surf = TTF_RenderText_Blended(font, text.c_str(), col);
            if (!surf) return;
            tex = SDL_CreateTextureFromSurface(ren, surf);
            SDL_FreeSurface(surf);
            if (!is_dynamic) text_cache[key] = tex;
        }

        int tw, th;
        SDL_QueryTexture(tex, NULL, NULL, &tw, &th);
        int tx = right_align ? x - tw : x;
        SDL_Rect r = { tx, y, tw, th };
        SDL_RenderCopy(ren, tex, NULL, &r);
        
        // 动态文本用完即毁，静态文本留着
        if (is_dynamic) SDL_DestroyTexture(tex);
    };
    vector<TrailBuffer> trails(b.size());
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
                if (follow_id == -1) {
                    offX = mx - (mx - offX) * (scale / old);
                    offY = my - (my - offY) * (scale / old);
                }
            }
            if (e.type == SDL_KEYDOWN) {
                switch (e.key.keysym.sym) {
                case SDLK_SPACE: paused = !paused; break;
                case SDLK_g: show_grid = !show_grid; break;
                case SDLK_t: show_tags = !show_tags; break;
                case SDLK_d: show_details = !show_details; break;
                case SDLK_c: 
                    for (auto& tr : trails) { 
                        tr.head = 0; 
                        tr.full = false; 
                    } 
                    break;
                case SDLK_EQUALS: replay_speed = min((int)replay_speed + 1, 50); break;
                case SDLK_MINUS:  replay_speed = max((int)replay_speed - 1, 1); break;
                case SDLK_TAB:
                    follow_id++;
                    if (follow_id >= (int)b.size()) follow_id = -1;
                    break;
                case SDLK_l:
                    if (follow_id == 3) follow_id = -1; else follow_id = 3;
                    break;
                case SDLK_ESCAPE: follow_id = -1; break;
                case SDLK_r:
                    replay_mode = !replay_mode;
                    if (replay_mode) {
                        paused = true;
                        lock_guard<mutex> lock(history_mutex);
                        if (!history.empty()) {
                            replay_index = (int)history.size() - 1;
                            b = history[replay_index].bodies;
                        }
                    }
                    else paused = false;
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
            if (e.type == SDL_MOUSEMOTION && dragging) {
                if (follow_id == -1) {
                    offX += e.motion.x - lx; offY += e.motion.y - ly;
                }
                lx = e.motion.x; ly = e.motion.y;
            }
        }

        // --- 核心：坐标系镜像映射下的视角锁定 ---
        if (follow_id != -1 && follow_id < (int)b.size()) {
            offX = (W / 2.0) - b[follow_id].x * scale;
            offY = (H / 2.0) - (-b[follow_id].y) * scale; // 锁定 y 的镜像值
        }

        SDL_SetRenderDrawColor(ren, 10, 10, 25, 255);
        SDL_RenderClear(ren);

        // --- 网格绘制（y 镜像） ---
        if (show_grid) {
            double raw_step = 120.0 / scale;
            double mag = pow(10, floor(log10(raw_step)));
            double step_world = (raw_step / mag > 5.0) ? 10.0 * mag : (raw_step / mag > 2.0 ? 5.0 * mag : 2.0 * mag);
            SDL_SetRenderDrawColor(ren, 50, 50, 70, 255);
            for (double x = ceil(((0 - offX) / scale) / step_world) * step_world; x * scale + offX <= W; x += step_world) {
                int sx = (int)(x * scale + offX);
                SDL_RenderDrawLine(ren, sx, 0, sx, H);
                draw_text(format_unit(x), sx + 5, H - 20, { 120, 120, 150, 255 });
            }
            for (double y = ceil(((0 - (H - offY)) / scale) / step_world) * step_world; y * scale + (H - offY) <= H + H; y += step_world) {
                int sy = (int)(-y * scale + offY); // 网格位置镜像
                if (sy >= 0 && sy <= H) {
                    SDL_RenderDrawLine(ren, 0, sy, W, sy);
                    draw_text(format_unit(y), 5, sy + 2, { 120, 120, 150, 255 });
                }
            }
        }

        // --- 天体与轨迹绘制（y 镜像） ---
        // --- 天体与轨迹绘制（y 镜像） ---
        for (size_t i = 0; i < b.size(); ++i) {
            // 更新轨迹（使用新的 TrailBuffer）
            if (!paused && !replay_mode) {
                trails[i].add(b[i].x, b[i].y);
            }

            // 绘制轨迹线
            SDL_SetRenderDrawColor(ren, b[i].color.r, b[i].color.g, b[i].color.b, 60);
            int count = trails[i].full ? MAX_TRAIL_POINTS : trails[i].head;
            
            // 环形缓冲区遍历逻辑
            for (int k = 1; k < count; ++k) {
                // 避免将最新的点连接到最旧的点（衔接处断开）
                if (trails[i].full && k == trails[i].head) continue;

                int p1 = k - 1;
                int p2 = k;

                double dist_sq = pow(trails[i].points[p2].first - trails[i].points[p1].first, 2) +
                                 pow(trails[i].points[p2].second - trails[i].points[p1].second, 2);

                // 只有距离合理才画线（防止坐标突变产生连线）
                if (dist_sq < pow(AU * 2.0, 2)) {
                    SDL_RenderDrawLine(ren,
                        (int)(trails[i].points[p1].first * scale + offX),
                        (int)(-trails[i].points[p1].second * scale + offY),
                        (int)(trails[i].points[p2].first * scale + offX),
                        (int)(-trails[i].points[p2].second * scale + offY));
                }
            }

            // 绘制天体实体
            int rx = (int)(b[i].x * scale + offX);
            int ry = (int)(-b[i].y * scale + offY);
            SDL_SetRenderDrawColor(ren, b[i].color.r, b[i].color.g, b[i].color.b, 255);
            SDL_Rect rb = { rx - 5, ry - 5, 10, 10 };
            SDL_RenderFillRect(ren, &rb);

            if (show_tags) {
                draw_text(b[i].name, rx + 12, ry - 8, { 255, 255, 255, 255 });
            }
        }
        // --- UI 叠加层 ---
        draw_text("TIME: " + format_time((double)current_step * dt), 20, 20, { 0, 255, 150, 255 });
        draw_text(replay_mode ? "STATUS: REPLAYING" : (paused ? "STATUS: PAUSED" : "STATUS: RUNNING"), 20, 40, { 255, 255, 0, 255 });

        if (follow_id != -1) {
            draw_text("FOLLOWING: " + b[follow_id].name + " [TAB/ESC]", W - 20, H - 40, { 255, 255, 0, 255 }, true);
        }

        // A. 左侧列表
        int legend_y = 75;
        draw_text("[ BODIES ]", 20, legend_y, { 150, 150, 150, 255 });
        for (size_t i = 0; i < (b.size() > 15 ? 15 : b.size()); ++i) {
            legend_y += 20;
            draw_text("-> " + b[i].name, 20, legend_y, b[i].color);
        }
        if (b.size() > 15) draw_text("... (" + to_string(b.size() - 15) + " more)", 20, legend_y + 20, { 100,100,100,255 });

        // B. 右上角：物理详细参数（自动换列）
        if (show_details) {
            int py = 20, col = 0, col_w = 220;
            for (auto& p : b) {
                int cur_x = W - 20 - (col * col_w);
                if (py + 100 > H - 100) { py = 20; col++; cur_x = W - 20 - (col * col_w); }

                draw_text("[" + p.name + "]", cur_x, py, p.color, true);
                draw_text("P: " + to_sci(p.x) + "," + to_sci(p.y), cur_x, py + 18, { 200,200,200,255 }, true);
                draw_text("V: " + to_sci(sqrt(p.vx * p.vx + p.vy * p.vy)) + " m/s", cur_x, py + 34, { 200,200,200,255 }, true);
                
                // --- 新增：航天器专属 UI ---
                if (p.is_spacecraft) {
                    SDL_Color fuel_col = (p.fuel_mass > 100) ? SDL_Color{0, 255, 255, 255} : SDL_Color{255, 50, 50, 255};
                    draw_text("Fuel: " + to_string((int)p.fuel_mass) + " kg", cur_x, py + 50, fuel_col, true);
                    
                    string mode_str = (p.mode == MANEUVERING ? "MANEUVER" : (p.thrust_force > 0 ? "THRUST" : "STABLE"));
                    draw_text("Mode: " + mode_str, cur_x, py + 66, {255, 255, 0, 255}, true);
                    py += 96; // 航天器占用更多空间
                } else {
                    draw_text("A: " + to_sci(sqrt(p.ax * p.ax + p.ay * p.ay)) + " m/s2", cur_x, py + 50, { 180,180,180,255 }, true);
                    py += 80;
                }

                if (col > 3) break; 
            }
        }

        draw_text("[G] Grid | [C] Clear | [T] Tags | [D] Details | [TAB] Follow | [R] Replay | [SPACE] Pause | [+/-] Replay spead | [Left/Right] Replay schedule", 20, H - 40, { 150, 150, 150, 255 });

        if (replay_mode && !history.empty()) {
            draw_text("REPLAY SPEED: x" + to_string(replay_speed), 20, H - 90, { 255, 100, 255, 255 });
            SDL_Rect bar_bg = { 20, H - 65, 350, 8 };
            SDL_SetRenderDrawColor(ren, 60, 60, 80, 255);
            SDL_RenderFillRect(ren, &bar_bg);
            float progress = (float)replay_index / (float)max(1, (int)history.size() - 1);
            SDL_Rect bar_fg = { 20, H - 65, (int)(350 * progress), 8 };
            SDL_SetRenderDrawColor(ren, 255, 100, 255, 255);
            SDL_RenderFillRect(ren, &bar_fg);
        }

        SDL_RenderPresent(ren);
        SDL_Delay(5);
    }
    if (font) TTF_CloseFont(font);
    TTF_Quit(); SDL_Quit();
}
// 在 main 函数启动线程之前执行：
void configure_voyager_mission(Body& ship) {
    // 节点 1：变速 (Prograde) - 增加轨道能量
    // 在第 500 秒开始，喷火 2 小时，推力 10000 牛顿
    ship.plan.emplace_back(500.0, 7200.0, 10000.0, PROGRADE);

    // 节点 2：转向 (Radial Out) - 改变轨道偏心率
    // 在第 1 天后执行一次横向修正
    ship.plan.emplace_back(86400.0, 600.0, 5000.0, RADIAL_OUT);

    // 节点 3：制动停靠 (Retrograde) - 为进入行星引力圈做准备
    // 在第 50 天执行大幅减速
    ship.plan.emplace_back(50.0 * 86400.0, 3600.0, 8000.0, RETROGRADE);
}

int main() {
    // 1. 定义基础天文常数 (修复报错的关键)
    const double v_e = 29780.0;        // 地球平均公转速度 (m/s)
    const double dist_e = AU;          // 地日平均距离 (1 AU)
    const double dist_m = 3.844e8;     // 地月距离 (m)
    const double v_m_rel = 1022.0;     // 月球相对于地球的速度
    const double sin60 = 0.86602540378;
    const double cos60 = 0.5;
    // 1. 核心天体初始化 (所有 vy 符号已校对为逆时针公转)
    b = {
        // 名称      质量(kg)    x(m)         y(m)   vx(m/s)  vy(m/s)  ax ay  颜色
        {"Sun",     1.989e30,  0,           0,     0,       0,       0, 0, {255, 220, 0, 255}},
        {"Mercury", 3.301e23,  0.387 * AU,  0,     0,       47360,   0, 0, {160, 160, 160, 255}},
        {"Venus",   4.867e24,  0.723 * AU,  0,     0,       35020,   0, 0, {255, 230, 150, 255}},
        {"Earth",   5.972e24, -1.0 * AU,    0,     0,      -v_e,     0, 0, {50, 150, 255, 255}},
        {"Moon",    7.348e22, -1.00257 * AU,0,     0,      -v_e - 1022, 0, 0, {180, 180, 180, 255}},
        {"Mars",    6.417e23,  1.524 * AU,  0,     0,       24077,   0, 0, {255, 80, 50, 255}},
        {"Jupiter", 1.898e27,  5.203 * AU,  0,     0,       13070,   0, 0, {240, 160, 110, 255}},
        {"Saturn",  5.683e26,  9.537 * AU,  0,     0,       9680,    0, 0, {210, 180, 140, 255}},
        {"Uranus",  8.681e25,  19.19 * AU,  0,     0,       6800,    0, 0, {150, 255, 255, 255}},
        {"Neptune", 1.024e26,  30.07 * AU,  0,     0,       5430,    0, 0, {80, 120, 255, 255}},
        // 冥王星数据 (Pluto)
        {"Pluto", 1.303e22, 39.48 * AU, 0, 0, 4740, 0, 0, {200, 170, 140, 255}},
        // 哈雷彗星 (逆行轨道：速度方向与行星相反)
        {"Halley",  2.2e14,    35.08 * AU,  0,     0,      -910,     0, 0, {255, 255, 255, 255}}
    };

    // 2. 月球捕获指令 (Probe B)
    // 这里利用我们写的 PID 模式。我们只需在特定时间开启 MANEUVERING 模式
    // 物理引擎会自动计算相对于月球的速度并进行“刹车”停靠。
    /*// 方案 A: 高比冲（离子发动机），低推力，模拟长期维持
    Body probeA;
    probeA.name = "Ion_Engine_Plan";
    probeA.is_spacecraft = true;
    probeA.mass = 2000; probeA.fuel_mass = 1000;
    probeA.isp = 3000; probeA.thrust_force = 0.5; // 推力小但持久
    probeA.x = -AU; probeA.y = 0.05 * AU; // 在地球轨道附近
    probeA.vx = 0; probeA.vy = -29780;
    probeA.mode = MANEUVERING;
    probeA.target_x = -1.5 * AU; // 目标是外层轨道
    probeA.color = {0, 255, 255, 255};
    b.push_back(probeA);

    // 方案 B: 低比冲（化学动力），大推力，模拟快速变轨
    Body probeB = probeA;
    probeB.name = "Chemical_Rocket_Plan";
    probeB.isp = 450; probeB.thrust_force = 100.0; // 暴力推力
    probeB.color = {255, 100, 0, 255};
    b.push_back(probeB);*/

    // 1. 创建航天器
    Body voyager;
    voyager.name = "Voyager_Alpha";
    voyager.is_spacecraft = true;
    voyager.mass = 5000.0;      // 总质量 (kg)
    voyager.fuel_mass = 2000.0; // 燃料质量 (kg)
    voyager.isp = 450.0;        // 化学火箭比冲 (s)
    voyager.x = -AU; voyager.y = 1e8; // 地球附近
    voyager.vx = 0; voyager.vy = -29780.0;
    voyager.color = {255, 255, 0, 255};
    voyager.mode = STABLE;
    // 在 main 函数启动线程之前：
    voyager.vx = 0; 
    voyager.vy = -29780.0; // 匹配地球速度，使其先进入稳定圆轨道
    voyager.fuel_mass = 5000.0; // 给足够的燃料

    // 设置一个立即生效的 90 度转向指令 (Radial Out)
    // 参数：10秒后开始，喷火1小时，力量 100,000 N
    voyager.plan.emplace_back(10.0, 3600.0, 100000.0, RADIAL_OUT);
    // 2. 为航天器规划任务
    configure_voyager_mission(voyager); // 100秒后开始任务
    // 在 b.push_back(voyager) 之前执行

    // 3. 将航天器加入物理系统
    b.push_back(voyager);
    // 2. 地日 L4/L5 稳定平衡点
    // L4 (地球前方 60°): 地球在 (-1, 0) 逆时针走，前方在第三象限 (y 为负)
    b.push_back({ "Earth_L4", 10.0, -AU * cos60, -AU * sin60,  v_e * sin60, -v_e * cos60, 0, 0, {0, 255, 120, 255} });
    // L5 (地球后方 60°): 后方在第二象限 (y 为正)
    b.push_back({ "Earth_L5", 10.0, -AU * cos60,  AU * sin60, -v_e * sin60, -v_e * cos60, 0, 0, {255, 120, 255, 255} });
    /*
    // 3. 批量生成特洛伊粒子群 (各 100 个)
    std::mt19937 gen(42);
    std::normal_distribution<double> pos_d(0.0, AU * 0.04);
    std::normal_distribution<double> vel_d(0.0, 180.0);

    // T4 群 (围绕 L4)
    for (int i = 0; i < 100; ++i) {
        double px = -AU * cos60 + pos_d(gen);
        double py = -AU * sin60 + pos_d(gen);
        double vx = v_e * sin60 + vel_d(gen);
        double vy = -v_e * cos60 + vel_d(gen);
        b.push_back({ "T4_" + std::to_string(i), 1e10, px, py, vx, vy, 0, 0, {100, 255, 150, 180} });
    }

    // T5 群 (围绕 L5)
    for (int i = 0; i < 100; ++i) {
        double px = -AU * cos60 + pos_d(gen);
        double py = AU * sin60 + pos_d(gen);
        double vx = -v_e * sin60 + vel_d(gen);
        double vy = -v_e * cos60 + vel_d(gen);
        b.push_back({ "T5_" + std::to_string(i), 1e10, px, py, vx, vy, 0, 0, {255, 150, 255, 180} });
    }
*/
    // 4. 全局质心修正 (确保太阳抵消总动量)
    double mvx = 0, mvy = 0;
    for (size_t i = 1; i < b.size(); ++i) {
        mvx += b[i].mass * b[i].vx;
        mvy += b[i].mass * b[i].vy;
    }
    b[0].vx = -mvx / b[0].mass;
    b[0].vy = -mvy / b[0].mass;
    int n = (int)b.size();
    shared_x.resize(n); shared_y.resize(n);

    // 获取 CPU 物理核心数，通常建议 4-8 个
    int num_workers = thread::hardware_concurrency();
    if (num_workers <= 0) num_workers = 4;
    
    // 同步点现在只针对工作线程，不再针对天体数
    barrier<> sync(num_workers); 
    vector<thread> workers;

    for (int i = 0; i < num_workers; ++i) {
        workers.emplace_back(physics_worker_pooled, i, num_workers, n, ref(sync));
    }

    thread renderer(render_loop);
    for (auto& t : workers) t.join(); 
    renderer.join();
}