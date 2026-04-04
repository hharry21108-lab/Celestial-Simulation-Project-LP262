#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <barrier>
#include <cmath>
#include <atomic>
#include <string>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL.h>
#include <deque>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <array>
#include <cstring>
#include <cstdlib>
#include <algorithm>
#undef main
using namespace std;
using namespace std::chrono;

const double G = 6.67430e-11;
const double AU = 1.496e11;
const double dt = 10;
const int n = 3;
const int steps = 2e8;

// 优化参数
const int MAX_TRAIL_POINTS = 5000;
const int HISTORY_STORE_INTERVAL = 200;
const int TARGET_FPS = 30;
const int FRAME_INTERVAL_MS = 1000 / TARGET_FPS;

struct Body {
    double mass = 0.0;
    double x = 0.0, y = 0.0;
    double vx = 0.0, vy = 0.0;
    double ax = 0.0, ay = 0.0;

    // 构造函数，确保所有成员被初始化
    Body() = default;
    Body(double m, double px, double py, double vx_val, double vy_val)
        : mass(m), x(px), y(py), vx(vx_val), vy(vy_val), ax(0.0), ay(0.0) {
    }
};

struct HistoryState {
    int step = 0;
    array<Body, n> bodies;
    double real_time = 0.0;

    // 构造函数：确保bodies数组被初始化
    HistoryState() {
        // array会自动调用Body的默认构造函数，所以bodies数组会被初始化
        // 但为了显式初始化，我们可以使用fill
        fill(bodies.begin(), bodies.end(), Body());
    }

    // 拷贝构造函数（编译器会自动生成，但显式声明更清晰）
    HistoryState(const HistoryState&) = default;
    HistoryState& operator=(const HistoryState&) = default;
};

vector<Body> b(n);
atomic<bool> ready(false);
atomic<bool> quit(false);
atomic<bool> paused(false);
atomic<bool> replay_mode(false);
atomic<int> current_step(0);
deque<HistoryState> history;
const int MAX_HISTORY = 100;
mutex history_mutex;

atomic<int> decimal_places(2);
atomic<bool> show_grid(true);
atomic<bool> force_render(false);
atomic<bool> trails_cleared(false);
atomic<bool> show_details(true);

void verlet_integration_step(int id, int step, barrier<>& sync_point, vector<double>& old_x, vector<double>& old_y, ofstream& fout);

string format_number(double value) {
    static char buffer[32];
    int places = decimal_places.load();

    if (abs(value) < 1e-3 && abs(value) > 0) {
        snprintf(buffer, sizeof(buffer), "%.*e", places, value);
    }
    else {
        snprintf(buffer, sizeof(buffer), "%.*f", places, value);
    }
    return string(buffer);
}

string format_time(double seconds) {
    static char buffer[64];
    if (seconds < 60) {
        snprintf(buffer, sizeof(buffer), "%.2fs", seconds);
    }
    else if (seconds < 3600) {
        snprintf(buffer, sizeof(buffer), "%.2fmin", seconds / 60);
    }
    else if (seconds < 86400) {
        snprintf(buffer, sizeof(buffer), "%.2fh", seconds / 3600);
    }
    else if (seconds < 31536000) {
        snprintf(buffer, sizeof(buffer), "%.2fd", seconds / 86400);
    }
    else {
        snprintf(buffer, sizeof(buffer), "%.2fy", seconds / 31536000);
    }
    return string(buffer);
}

vector<SDL_Color> generate_colors() {
    return {
        {255, 100, 100, 255},
        {100, 255, 100, 255},
        {255, 255, 100, 255}
    };
}

void render_thread(vector<Body>& b) {
    // 1. 初始化SDL和TTF
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        cerr << "SDL_Init failed: " << SDL_GetError() << endl;
        return;
    }

    if (TTF_Init() != 0) {
        cerr << "TTF_Init failed: " << TTF_GetError() << endl;
        SDL_Quit();
        return;
    }

    // 2. 创建窗口
    int WIN_W = 1500;
    int WIN_H = 780;
    SDL_Window* win = SDL_CreateWindow(
        "N-body Simulation",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        WIN_W,
        WIN_H,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
    );

    if (!win) {
        cerr << "SDL_CreateWindow failed: " << SDL_GetError() << endl;
        TTF_Quit();
        SDL_Quit();
        return;
    }

    // 3. 创建渲染器
    SDL_Renderer* ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);
    if (!ren) {
        cerr << "SDL_CreateRenderer failed: " << SDL_GetError() << endl;
        SDL_DestroyWindow(win);
        TTF_Quit();
        SDL_Quit();
        return;
    }

    // 4. 加载字体
    TTF_Font* font = nullptr;
    const char* font_paths[] = {
        "arial.ttf",
        "C:/Windows/Fonts/arial.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        nullptr
    };

    for (int i = 0; font_paths[i] != nullptr; i++) {
        font = TTF_OpenFont(font_paths[i], 16);
        if (font) {
            cout << "Loaded font: " << font_paths[i] << endl;
            break;
        }
    }

    if (!font) {
        font = TTF_OpenFont(nullptr, 16);
        if (!font) {
            cerr << "Failed to load any font: " << TTF_GetError() << endl;
        }
    }

    TTF_Font* details_font = nullptr;
    if (font) {
        details_font = TTF_OpenFont("arial.ttf", 12);
        if (!details_font) details_font = TTF_OpenFont(nullptr, 12);
        if (!details_font) details_font = font;
    }

    // 5. 准备轨迹数据
    vector<array<pair<double, double>, MAX_TRAIL_POINTS>> trails(b.size());
    vector<int> trail_sizes(b.size(), 0);
    vector<SDL_Color> bodyColor = generate_colors();

    // 6. 坐标转换函数
    double scale = 1e-9;
    double offsetX = WIN_W / 2;
    double offsetY = WIN_H / 2;
    bool dragging = false;
    int lastX = 0, lastY = 0;

    auto wx = [&](double x) { return static_cast<int>(x * scale + offsetX); };
    auto wy = [&](double y) { return static_cast<int>(y * scale + offsetY); };
    auto sx_to_world = [&](int sx) { return (sx - offsetX) / scale; };
    auto sy_to_world = [&](int sy) { return (sy - offsetY) / scale; };

    // 7. 辅助函数
    auto format_with_unit = [&](double meters) {
        double absx = fabs(meters);
        static char buf[64];
        if (absx < 1e3)
            snprintf(buf, sizeof(buf), "%s m", format_number(meters).c_str());
        else if (absx < 1e9)
            snprintf(buf, sizeof(buf), "%s km", format_number(meters / 1000.0).c_str());
        else
            snprintf(buf, sizeof(buf), "%s AU", format_number(meters / AU).c_str());
        return string(buf);
        };

    auto draw_text = [&](string text, int x, int y, SDL_Color color = { 200, 200, 200 }, TTF_Font* f = nullptr) {
        if (!font) return;
        TTF_Font* use_font = f ? f : font;
        SDL_Surface* surf = TTF_RenderText_Solid(use_font, text.c_str(), color);
        if (!surf) return;
        SDL_Texture* tex = SDL_CreateTextureFromSurface(ren, surf);
        if (!tex) {
            SDL_FreeSurface(surf);
            return;
        }
        SDL_Rect r = { x, y, surf->w, surf->h };
        SDL_RenderCopy(ren, tex, NULL, &r);
        SDL_FreeSurface(surf);
        SDL_DestroyTexture(tex);
        };

    // 优化：获取文本宽度，用于动态布局
    auto get_text_width = [&](string text, TTF_Font* f = nullptr) -> int {
        if (!font) return 0;
        TTF_Font* use_font = f ? f : font;
        int w, h;
        TTF_SizeText(use_font, text.c_str(), &w, &h);
        return w;
        };

    auto choose_grid_step = [&]() {
        double steps[] = { 1e6, 1e7, 1e8, 1e9, 5e9, 1e10, 5e10, 1e11 };
        for (double s : steps)
            if (s * scale > 60) return s;
        return 1e12;
        };

    auto draw_grid = [&]() {
        if (!show_grid.load()) return;
        double grid_step = choose_grid_step();
        double startX = sx_to_world(0);
        double endX = sx_to_world(WIN_W);
        double startY = sy_to_world(0);
        double endY = sy_to_world(WIN_H);
        SDL_SetRenderDrawColor(ren, 50, 50, 50, 255);

        for (double gx = floor(startX / grid_step) * grid_step; gx < endX; gx += grid_step) {
            int sx = wx(gx);
            SDL_RenderDrawLine(ren, sx, 0, sx, WIN_H);
            if (font) draw_text(format_with_unit(gx), sx + 3, offsetY + 8);
        }

        for (double gy = floor(startY / grid_step) * grid_step; gy < endY; gy += grid_step) {
            int sy = wy(gy);
            SDL_RenderDrawLine(ren, 0, sy, WIN_W, sy);
            if (font) draw_text(format_with_unit(gy), offsetX + 8, sy + 3);
        }

        int x0 = wx(0);
        int y0 = wy(0);
        SDL_SetRenderDrawColor(ren, 120, 120, 120, 255);
        SDL_RenderDrawLine(ren, 0, y0, WIN_W, y0);
        SDL_RenderDrawLine(ren, x0, 0, x0, WIN_H);
        };

    auto clear_trails = [&]() {
        for (int i = 0; i < b.size(); i++) {
            trail_sizes[i] = 0;
        }
        };

    // 优化后的图例和详细信息绘制 - 添加性能优化
    auto draw_legend = [&]() {
        int x = 20, y = 20;

        // 天体图例（始终显示，因为显示天体是核心功能）
        for (int i = 0; i < static_cast<int>(b.size()); i++) {
            SDL_SetRenderDrawColor(ren, bodyColor[i].r, bodyColor[i].g, bodyColor[i].b, 255);
            SDL_Rect rect = { x, y + i * 25, 15, 15 };
            SDL_RenderFillRect(ren, &rect);

            static const char* names[] = { "Earth", "Moon", "Sun" };
            if (font) draw_text(names[i], x + 25, y + i * 25);
        }

        int status_y = y + static_cast<int>(b.size()) * 25 + 10;

        // 优化：只在需要时绘制状态信息
        string status = replay_mode.load() ? "Replay Mode" : (paused.load() ? "Paused" : "Running");
        if (font) draw_text("Status: " + status, x, status_y, { 255, 255, 0 });

        if (font) draw_text("Step: " + to_string(current_step.load()), x, status_y + 20);

        // 优化：时间显示可以简化或省略，但为保持完整性仍保留
        double real_time = current_step.load() * dt;
        if (font) draw_text("Time: " + format_time(real_time), x, status_y + 40, { 0, 255, 0 });

        if (font) draw_text("Decimals: " + to_string(decimal_places.load()), x, status_y + 60);

        // 优化：历史记录数只有在需要时才计算和显示
        if (replay_mode.load() || show_details.load()) {
            lock_guard<mutex> lock(history_mutex);
            if (font) draw_text("History: " + to_string(history.size()), x, status_y + 80);
        }

        // 优化：详细信息面板 - 只在显示时才进行计算和绘制
        if (show_details.load() && font) {
            // 计算最长的文本行，以确定面板宽度
            int max_width = 0;
            for (int i = 0; i < static_cast<int>(b.size()); i++) {
                static const char* names[] = { "Earth", "Moon", "Sun" };
                string mass_str = "Mass: " + format_number(b[i].mass) + " kg";
                string pos_str = "Pos: (" + format_number(b[i].x) + ", " + format_number(b[i].y) + ")";
                string vel_str = "Vel: (" + format_number(b[i].vx) + ", " + format_number(b[i].vy) + ")";

                max_width = max(max_width, get_text_width(names[i], details_font));
                max_width = max(max_width, get_text_width(mass_str, details_font));
                max_width = max(max_width, get_text_width(pos_str, details_font));
                max_width = max(max_width, get_text_width(vel_str, details_font));
            }

            max_width += 40; // 添加边距

            // 确保面板不会超出窗口边界
            int details_x = WIN_W - max_width - 20;
            if (details_x < 0) {
                details_x = 10;
                max_width = WIN_W - 20;
            }

            int details_y = 20;

            // 绘制详细信息标题
            draw_text("--- Body Details ---", details_x, details_y, { 255, 200, 0 });

            // 绘制每个天体的详细信息
            for (int i = 0; i < static_cast<int>(b.size()); i++) {
                int body_y = details_y + 20 + i * 70;
                static const char* names[] = { "Earth", "Moon", "Sun" };

                draw_text(names[i], details_x, body_y, bodyColor[i], details_font);

                string mass_str = "Mass: " + format_number(b[i].mass) + " kg";
                draw_text(mass_str, details_x + 10, body_y + 15, { 200, 200, 200 }, details_font);

                string pos_str = "Pos: (" + format_number(b[i].x) + ", " + format_number(b[i].y) + ")";
                draw_text(pos_str, details_x + 10, body_y + 30, { 200, 200, 200 }, details_font);

                string vel_str = "Vel: (" + format_number(b[i].vx) + ", " + format_number(b[i].vy) + ")";
                draw_text(vel_str, details_x + 10, body_y + 45, { 200, 200, 200 }, details_font);
            }
        }

        // 优化：帮助信息 - 始终显示，但可以简化
        int help_y = WIN_H - 160;
        if (help_y < 200) help_y = 200;

        static const char* help_lines[] = {
            "Space: Pause/Resume",
            "R: Toggle Replay Mode",
            "Left/Right: Navigate History",
            "+/-: Adjust Decimals",
            "G: Toggle Grid",
            "C: Clear Trails",
            "D: Toggle Details",
            "Mouse: Zoom & Drag"
        };

        if (font) {
            for (int i = 0; i < 8; i++) {
                draw_text(help_lines[i], x, help_y + i * 20);
            }
        }

        // 优化：回放模式提示 - 只在需要时显示
        if (replay_mode.load() && font) {
            draw_text("Replay: Use Left/Right to navigate", x, status_y - 20, { 255, 255, 0 });
        }
        };

    auto save_current_history = [&]() {
        lock_guard<mutex> lock(history_mutex);
        HistoryState state;
        state.step = current_step.load();

        for (int i = 0; i < n; i++) {
            state.bodies[i] = b[i];
        }

        state.real_time = current_step.load() * dt;

        history.push_back(state);
        if (history.size() > MAX_HISTORY) {
            history.pop_front();
        }
        };

    auto load_history_state = [&](int target_step) {
        lock_guard<mutex> lock(history_mutex);
        if (history.empty()) return;

        auto best_it = history.begin();
        int best_diff = abs(best_it->step - target_step);

        for (auto it = history.begin(); it != history.end(); ++it) {
            int diff = abs(it->step - target_step);
            if (diff < best_diff) {
                best_diff = diff;
                best_it = it;
            }
        }

        for (int i = 0; i < n; i++) {
            b[i] = best_it->bodies[i];
        }
        current_step.store(best_it->step);
        };

    // 主渲染循环
    Uint32 last_frame_time = SDL_GetTicks();
    const Uint32 frame_interval = FRAME_INTERVAL_MS;

    cout << "Starting render loop..." << endl;
    cout << "Window created successfully: " << (win != nullptr) << endl;
    cout << "Renderer created successfully: " << (ren != nullptr) << endl;
    cout << "Font loaded: " << (font != nullptr) << endl;

    while (!quit) {
        // 事件处理
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                quit = true;
                break;
            }

            if (e.type == SDL_WINDOWEVENT && e.window.event == SDL_WINDOWEVENT_RESIZED) {
                WIN_W = e.window.data1;
                WIN_H = e.window.data2;
                force_render.store(true);
            }

            if (e.type == SDL_KEYDOWN) {
                switch (e.key.keysym.sym) {
                case SDLK_SPACE:
                    paused.store(!paused.load());
                    force_render.store(true);
                    break;
                case SDLK_r:
                    replay_mode.store(!replay_mode.load());
                    if (replay_mode.load()) save_current_history();
                    force_render.store(true);
                    break;
                case SDLK_LEFT:
                    if (replay_mode.load()) {
                        lock_guard<mutex> lock(history_mutex);
                        if (!history.empty()) {
                            for (auto it = history.rbegin(); it != history.rend(); ++it) {
                                if (it->step < current_step.load()) {
                                    for (int i = 0; i < n; i++) b[i] = it->bodies[i];
                                    current_step.store(it->step);
                                    break;
                                }
                            }
                            force_render.store(true);
                        }
                    }
                    break;
                case SDLK_RIGHT:
                    if (replay_mode.load()) {
                        lock_guard<mutex> lock(history_mutex);
                        if (!history.empty()) {
                            for (const auto& state : history) {
                                if (state.step > current_step.load()) {
                                    for (int i = 0; i < n; i++) b[i] = state.bodies[i];
                                    current_step.store(state.step);
                                    break;
                                }
                            }
                            force_render.store(true);
                        }
                    }
                    break;
                case SDLK_PLUS:
                case SDLK_EQUALS:
                    if (decimal_places.load() < 6) {
                        decimal_places.store(decimal_places.load() + 1);
                        force_render.store(true);
                    }
                    break;
                case SDLK_MINUS:
                    if (decimal_places.load() > 0) {
                        decimal_places.store(decimal_places.load() - 1);
                        force_render.store(true);
                    }
                    break;
                case SDLK_g:
                    show_grid.store(!show_grid.load());
                    force_render.store(true);
                    break;
                case SDLK_c:
                    clear_trails();
                    trails_cleared.store(true);
                    force_render.store(true);
                    break;
                case SDLK_d:
                    show_details.store(!show_details.load());
                    force_render.store(true);
                    break;
                }
            }

            if (e.type == SDL_MOUSEWHEEL) {
                double old = scale;
                if (e.wheel.y > 0) scale *= 1.1;
                if (e.wheel.y < 0) scale /= 1.1;
                int mx, my;
                SDL_GetMouseState(&mx, &my);
                offsetX = mx - (mx - offsetX) * (scale / old);
                offsetY = my - (my - offsetY) * (scale / old);
                force_render.store(true);
            }

            if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT) {
                dragging = true;
                lastX = e.button.x;
                lastY = e.button.y;
                force_render.store(true);
            }

            if (e.type == SDL_MOUSEBUTTONUP && e.button.button == SDL_BUTTON_LEFT) {
                dragging = false;
                force_render.store(true);
            }

            if (e.type == SDL_MOUSEMOTION && dragging) {
                offsetX += e.motion.x - lastX;
                offsetY += e.motion.y - lastY;
                lastX = e.motion.x;
                lastY = e.motion.y;
                force_render.store(true);
            }
        }

        if (quit) break;

        // 帧率控制
        Uint32 current_time = SDL_GetTicks();
        bool time_for_frame = (current_time - last_frame_time) >= frame_interval;

        bool should_render = ready.load() || force_render.load() || (time_for_frame && !paused.load());

        if (should_render) {
            if (force_render.load()) force_render.store(false);
            if (ready.load()) ready.store(false);

            if (time_for_frame) last_frame_time = current_time;

            // 优化：只有在回放模式下才保存历史记录
            if (replay_mode.load() && current_step.load() % HISTORY_STORE_INTERVAL == 0) {
                save_current_history();
            }

            SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
            SDL_RenderClear(ren);
            draw_grid();

            for (int i = 0; i < static_cast<int>(b.size()); i++) {
                if (!replay_mode.load() && !trails_cleared.load()) {
                    if (trail_sizes[i] < MAX_TRAIL_POINTS) {
                        trails[i][trail_sizes[i]] = { b[i].x, b[i].y };
                        trail_sizes[i]++;
                    }
                    else {
                        for (int j = 0; j < MAX_TRAIL_POINTS - 1; j++) {
                            trails[i][j] = trails[i][j + 1];
                        }
                        trails[i][MAX_TRAIL_POINTS - 1] = { b[i].x, b[i].y };
                    }
                }

                if (trail_sizes[i] > 1) {
                    SDL_SetRenderDrawColor(ren, bodyColor[i].r, bodyColor[i].g, bodyColor[i].b, 100);
                    vector<SDL_Point> pts(trail_sizes[i]);
                    for (int k = 0; k < trail_sizes[i]; k++) {
                        pts[k].x = wx(trails[i][k].first);
                        pts[k].y = wy(trails[i][k].second);
                    }
                    SDL_RenderDrawLines(ren, pts.data(), pts.size());
                }

                SDL_SetRenderDrawColor(ren, bodyColor[i].r, bodyColor[i].g, bodyColor[i].b, 255);
                double visual_radius = pow(b[i].mass, 1.0 / 3.0) * 1e-5;
                int R = max(2, static_cast<int>(visual_radius * scale));

                int sx = wx(b[i].x);
                int sy = wy(b[i].y);

                for (int dx = -R; dx <= R; dx++) {
                    for (int dy = -R; dy <= R; dy++) {
                        if (dx * dx + dy * dy <= R * R) {
                            SDL_RenderDrawPoint(ren, sx + dx, sy + dy);
                        }
                    }
                }
            }

            draw_legend();
            SDL_RenderPresent(ren);

            trails_cleared.store(false);
        }
        else {
            SDL_Delay(1);
        }
    }

    cout << "Cleaning up resources..." << endl;

    if (details_font && details_font != font) TTF_CloseFont(details_font);
    if (font) TTF_CloseFont(font);
    TTF_Quit();
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();

    cout << "Render thread finished." << endl;
}

void verlet_integration_step(int id, int step, barrier<>& sync_point, vector<double>& old_x, vector<double>& old_y, ofstream& fout) {
    static int check_counter = 0;
    check_counter++;

    if (check_counter % 100 == 0) {
        while ((paused.load() || replay_mode.load()) && !quit) {
            this_thread::sleep_for(chrono::milliseconds(10));
        }
    }

    if (quit) return;

    if (id == 0) {
        current_step.store(step);
    }

    double ax = 0.0, ay = 0.0;
    for (int j = 0; j < n; ++j) {
        if (j == id) continue;
        double dx = old_x[j] - old_x[id];
        double dy = old_y[j] - old_y[id];
        double r2 = dx * dx + dy * dy;
        double r = sqrt(r2);
        double f = G * b[j].mass / (r2 * r);
        ax += f * dx;
        ay += f * dy;
    }
    b[id].ax = ax;
    b[id].ay = ay;

    if (id == 0) {
        ready.store(true);
    }

    sync_point.arrive_and_wait();

    b[id].vx += 0.5 * b[id].ax * dt;
    b[id].vy += 0.5 * b[id].ay * dt;
    b[id].x += b[id].vx * dt;
    b[id].y += b[id].vy * dt;

    sync_point.arrive_and_wait();

    double ax2 = 0.0, ay2 = 0.0;
    for (int j = 0; j < n; ++j) {
        if (j == id) continue;
        double dx = b[j].x - b[id].x;
        double dy = b[j].y - b[id].y;
        double r2 = dx * dx + dy * dy;
        double r = sqrt(r2);
        double f = G * b[j].mass / (r2 * r);
        ax2 += f * dx;
        ay2 += f * dy;
    }

    b[id].vx += 0.5 * ax2 * dt;
    b[id].vy += 0.5 * ay2 * dt;

    old_x[id] = b[id].x;
    old_y[id] = b[id].y;

    sync_point.arrive_and_wait();
    // 输出（只有线程0执行）
    /*if (id == 0 && step % 500 == 0) {
        double real_time = step * dt;
        fout << "Step " << step << " (Time: " << format_time(real_time) << ")\n";
        for (int k = 0; k < n; k++) {
            fout << "Body " << k << ": mass=" << b[k].mass
                << " x=" << format_number(b[k].x) << " y=" << format_number(b[k].y)
                << " vx=" << format_number(b[k].vx) << " vy=" << format_number(b[k].vy) << "\n";
        }
        fout << "\n";
    }*/
}

int main() {
    cout << "Initializing simulation..." << endl;

    ofstream fout("orbit.txt");
    if (!fout.is_open()) {
        cerr << "Warning: Could not open output file." << endl;
    }

    // ==================== 初始条件 ====================
    const double m_s = 1.989e30;
    const double m_e = 5.972e24;
    const double m_m = 7.348e22;
    const double d_earth_sun = AU;
    const double d_earth_moon = 3.844e8;

    const double v_earth_orbit = sqrt(G * m_s / d_earth_sun);
    const double v_moon_orbit = sqrt(G * m_e / d_earth_moon);

    const double p_earth = m_e * v_earth_orbit;
    const double p_moon = m_m * (v_earth_orbit + v_moon_orbit);
    const double p_total = p_earth + p_moon;
    const double p_sun = -p_total;
    const double v_sun = p_sun / m_s;

    // 使用构造函数初始化天体
    b[2] = Body(m_s, 0.0, 0.0, 0.0, v_sun);  // 太阳
    b[0] = Body(m_e, -d_earth_sun, 0.0, 0.0, v_earth_orbit);  // 地球
    b[1] = Body(m_m, -d_earth_sun + d_earth_moon, 0.0, 0.0, v_earth_orbit + v_moon_orbit);  // 月球

    vector<double> old_x(n), old_y(n);
    for (int i = 0; i < n; ++i) {
        old_x[i] = b[i].x;
        old_y[i] = b[i].y;
    }

    cout << "Starting simulation threads..." << endl;

    barrier<> sync_point(n);
    thread renderer(render_thread, ref(b));

    auto worker = [&](int id) {
        cout << "Physics thread " << id << " started" << endl;
        int step = 0;
        while (step < steps && !quit) {
            verlet_integration_step(id, step, sync_point, old_x, old_y, fout);
            step++;
        }
        cout << "Physics thread " << id << " finished" << endl;
        };

    vector<thread> threads;
    for (int i = 0; i < n; ++i) {
        threads.emplace_back(worker, i);
    }

    for (auto& t : threads) t.join();
    renderer.join();

    cout << "Simulation finished." << endl;
    return 0;
}
