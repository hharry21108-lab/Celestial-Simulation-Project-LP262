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
#undef main
using namespace std;
using namespace std::chrono;

const double G = 6.67430e-11;
const double AU = 1.496e11;
const double dt = 100; //步长
const int n = 5;       //天体数量
const int steps = 2e5; //模拟步数

struct Body {
    double mass;
    double x, y;
    double vx, vy;
    double ax, ay; //当前加速度
};

// 历史状态结构体 - 修复未初始化问题
struct HistoryState {
    int step = 0;           // 初始化为0
    vector<Body> bodies;
    double real_time = 0.0; // 初始化为0.0
};

vector<Body> b(n);
///渲染///
atomic<bool> ready(false);
atomic<bool> quit(false);
atomic<bool> paused(false);
atomic<bool> replay_mode(false); // 回放模式标志
atomic<int> current_step(0);
deque<HistoryState> history; // 历史记录队列
const int MAX_HISTORY = 500; // 最大历史记录数
mutex history_mutex;

// 控件参数
atomic<int> decimal_places(2); // 小数显示位数
atomic<bool> show_grid(true); // 显示网格
atomic<bool> force_render(false); // 强制渲染标志
atomic<bool> trails_cleared(false); // 轨迹清空标志
atomic<bool> show_details(true); // 显示详细信息标志

// Verlet积分器函数声明
void verlet_integration_step(int id, int step, barrier<>& sync_point, vector<double>& old_x, vector<double>& old_y, ofstream& fout);

// 格式化数字，根据decimal_places控制小数位数
string format_number(double value) {
    int places = decimal_places.load();
    stringstream ss;
    if (abs(value) < 1e-3 && abs(value) > 0) {
        ss << scientific << setprecision(places) << value;
    }
    else {
        ss << fixed << setprecision(places) << value;
    }
    return ss.str();
}

// 格式化时间
string format_time(double seconds) {
    if (seconds < 60) {
        return format_number(seconds) + "s";
    }
    else if (seconds < 3600) {
        return format_number(seconds / 60) + "min";
    }
    else if (seconds < 86400) {
        return format_number(seconds / 3600) + "h";
    }
    else if (seconds < 31536000) {
        return format_number(seconds / 86400) + "d";
    }
    else {
        return format_number(seconds / 31536000) + "y";
    }
}

// HSL转RGB函数 - 用于生成高对比度的颜色
SDL_Color hsl_to_rgb(double h, double s, double l) {
    // 归一化H到[0, 360]
    h = fmod(h, 360.0);
    if (h < 0) h += 360.0;

    // 确保S和L在[0,1]范围内
    s = max(0.0, min(1.0, s));
    l = max(0.0, min(1.0, l));

    double c = (1 - fabs(2 * l - 1)) * s;
    double x = c * (1 - fabs(fmod(h / 60.0, 2) - 1));
    double m = l - c / 2;

    double r, g, b;

    if (h < 60) {
        r = c; g = x; b = 0;
    }
    else if (h < 120) {
        r = x; g = c; b = 0;
    }
    else if (h < 180) {
        r = 0; g = c; b = x;
    }
    else if (h < 240) {
        r = 0; g = x; b = c;
    }
    else if (h < 300) {
        r = x; g = 0; b = c;
    }
    else {
        r = c; g = 0; b = x;
    }

    // 转换到0-255范围
    SDL_Color color;
    color.r = static_cast<Uint8>((r + m) * 255);
    color.g = static_cast<Uint8>((g + m) * 255);
    color.b = static_cast<Uint8>((b + m) * 255);
    color.a = 255;

    return color;
}

// 生成一组高对比度的颜色
vector<SDL_Color> generate_high_contrast_colors(int count) {
    vector<SDL_Color> colors;

    // 使用黄金分割比例来生成色相，确保颜色差异明显
    const double golden_ratio_conjugate = 0.618033988749895;
    double hue = 0.0;

    for (int i = 0; i < count; i++) {
        // 使用HSL颜色模型生成高对比度颜色
        // 饱和度100%，亮度50%确保颜色鲜艳
        // 色相使用黄金分割比例，确保相邻颜色差异明显
        SDL_Color color = hsl_to_rgb(hue, 1.0, 0.5);
        colors.push_back(color);

        // 下一个色相，使用黄金分割比例
        hue += 360.0 * golden_ratio_conjugate;
        hue = fmod(hue, 360.0);
    }

    return colors;
}

void render_thread(vector<Body>& b) {
    SDL_Init(SDL_INIT_VIDEO);
    TTF_Init();
    int WIN_W = 1500;
    int WIN_H = 780;
    SDL_Window* win = SDL_CreateWindow("N-body Simulation",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WIN_W, WIN_H,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
    SDL_Renderer* ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    // 修复：检查字体加载是否成功
    TTF_Font* font = TTF_OpenFont("C:/Windows/Fonts/arial.ttf", 16);
    if (!font) {
        printf("Font load error: %s\n", TTF_GetError());
        // 使用默认字体或跳过文本渲染
        font = TTF_OpenFont(nullptr, 16); // 尝试使用默认字体
        if (!font) {
            printf("Failed to load any font\n");
            TTF_Quit();
            SDL_DestroyRenderer(ren);
            SDL_DestroyWindow(win);
            SDL_Quit();
            return;
        }
    }

    // 使用稍小的字体用于详细信息显示
    TTF_Font* details_font = TTF_OpenFont("C:/Windows/Fonts/arial.ttf", 12);
    if (!details_font) {
        details_font = font; // 如果加载失败，使用主字体
    }

    vector<vector<pair<double, double>>> trails(b.size());

    // 生成高对比度的颜色
    vector<SDL_Color> bodyColor = generate_high_contrast_colors(b.size());

    double scale = 1e-9;
    double offsetX = WIN_W / 2;
    double offsetY = WIN_H / 2;
    bool dragging = false;
    int lastX = 0, lastY = 0;
    SDL_Event e;

    // 辅助函数定义
    auto wx = [&](double x) { return static_cast<int>(x * scale + offsetX); };
    auto wy = [&](double y) { return static_cast<int>(y * scale + offsetY); };
    auto sx_to_world = [&](int sx) { return (sx - offsetX) / scale; };
    auto sy_to_world = [&](int sy) { return (sy - offsetY) / scale; };

    auto format_with_unit = [&](double meters) {
        double absx = fabs(meters);
        char buf[64];
        if (absx < 1e3)
            snprintf(buf, sizeof(buf), "%s m", format_number(meters).c_str());
        else if (absx < 1e9)
            snprintf(buf, sizeof(buf), "%s km", format_number(meters / 1000.0).c_str());
        else
            snprintf(buf, sizeof(buf), "%s AU", format_number(meters / AU).c_str());
        return string(buf);
        };

    auto draw_text = [&](string text, int x, int y, SDL_Color color = { 200, 200, 200 }, TTF_Font* f = nullptr) {
        TTF_Font* use_font = f ? f : font;
        if (!use_font) return; // 安全检查
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

    auto choose_grid_step = [&]() {
        double steps[] = { 1e6,1e7,1e8,1e9,5e9,1e10,5e10,1e11 };
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
            draw_text(format_with_unit(gx), sx + 3, offsetY + 8);
        }

        for (double gy = floor(startY / grid_step) * grid_step; gy < endY; gy += grid_step) {
            int sy = wy(gy);
            SDL_RenderDrawLine(ren, 0, sy, WIN_W, sy);
            draw_text(format_with_unit(gy), offsetX + 8, sy + 3);
        }

        int x0 = wx(0);
        int y0 = wy(0);
        SDL_SetRenderDrawColor(ren, 120, 120, 120, 255);
        SDL_RenderDrawLine(ren, 0, y0, WIN_W, y0);
        SDL_RenderDrawLine(ren, x0, 0, x0, WIN_H);
        };

    // 清空轨迹
    auto clear_trails = [&]() {
        for (auto& trail : trails) {
            trail.clear();
        }
        };

    // 绘制图例和状态信息
    auto draw_legend = [&]() {
        int x = 20, y = 20;

        // 绘制天体图例 - 使用更明显的标签和颜色
        for (int i = 0; i < static_cast<int>(b.size()); i++) {
            SDL_SetRenderDrawColor(ren,
                bodyColor[i].r,
                bodyColor[i].g,
                bodyColor[i].b,
                255);

            SDL_Rect rect = { x, y + i * 25, 15, 15 };
            SDL_RenderFillRect(ren, &rect);

            // 使用更有描述性的天体名称
            string bodyName;
            switch (i) {
            case 0: bodyName = "Sun"; break;
            case 1: bodyName = "Earth"; break;
            case 2: bodyName = "Moon"; break;
            case 3: bodyName = "L4"; break;
            case 4: bodyName = "L5"; break;
            default: bodyName = "Body " + to_string(i); break;
            }
            draw_text(bodyName, x + 25, y + i * 25);
        }

        // 显示状态信息
        int status_y = y + static_cast<int>(b.size()) * 25 + 10;
        string status;
        if (replay_mode.load()) {
            status = "Replay Mode";
        }
        else if (paused.load()) {
            status = "Paused";
        }
        else {
            status = "Running";
        }
        draw_text("Status: " + status, x, status_y, { 255, 255, 0 });

        // 显示当前步数
        draw_text("Step: " + to_string(current_step.load()), x, status_y + 20);

        // 显示时间
        double real_time = current_step.load() * dt;
        draw_text("Time: " + format_time(real_time), x, status_y + 40, { 0, 255, 0 });

        // 显示小数位数
        draw_text("Decimals: " + to_string(decimal_places.load()), x, status_y + 60);

        // 显示历史记录数
        {
            lock_guard<mutex> lock(history_mutex);
            draw_text("History: " + to_string(history.size()), x, status_y + 80);
        }

        // 显示详细信息（如果启用）- 移至右上角
        if (show_details.load()) {
            int details_x = WIN_W - 320; // 右上角，距离右边320像素
            int details_y = 20; // 与左上角图例相同的y坐标

            // 绘制详细信息标题
            draw_text("--- Body Details ---", details_x, details_y, { 255, 200, 0 });

            for (int i = 0; i < static_cast<int>(b.size()); i++) {
                int body_y = details_y + 20 + i * 70;
                // 天体名称 - 使用更有描述性的名称
                string bodyName;
                switch (i) {
                case 0: bodyName = "Sun"; break;
                case 1: bodyName = "Earth"; break;
                case 2: bodyName = "Moon"; break;
                case 3: bodyName = "L4 Point"; break;
                case 4: bodyName = "L5 Point"; break;
                default: bodyName = "Body " + to_string(i); break;
                }
                draw_text(bodyName, details_x, body_y, bodyColor[i], details_font);

                // 质量
                string mass_str = "Mass: " + format_number(b[i].mass) + " kg";
                draw_text(mass_str, details_x + 10, body_y + 15, { 200, 200, 200 }, details_font);

                // 位置
                string pos_str = "Pos: (" + format_number(b[i].x) + ", " + format_number(b[i].y) + ")";
                draw_text(pos_str, details_x + 10, body_y + 30, { 200, 200, 200 }, details_font);

                // 速度
                string vel_str = "Vel: (" + format_number(b[i].vx) + ", " + format_number(b[i].vy) + ")";
                draw_text(vel_str, details_x + 10, body_y + 45, { 200, 200, 200 }, details_font);

                // 加速度
                string acc_str = "Acc: (" + format_number(b[i].ax) + ", " + format_number(b[i].ay) + ")";
                draw_text(acc_str, details_x + 10, body_y + 60, { 200, 200, 200 }, details_font);
            }
        }

        // 显示帮助信息 - 动态计算位置，确保不超出窗口
        // 计算状态信息占用的高度
        int status_bottom = status_y + 100; // 状态信息占用的高度（包括历史记录数）
        // 计算帮助信息需要的高度（10行，每行20像素，加上边距）
        int help_height = 10 * 20 + 20;

        // 如果窗口高度不够，将帮助信息放在状态信息下方
        int help_y;
        if (WIN_H > status_bottom + help_height + 20) {
            // 窗口高度足够，将帮助信息放在底部
            help_y = WIN_H - help_height - 20;
        }
        else {
            // 窗口高度不够，将帮助信息放在状态信息下方
            help_y = status_bottom + 20;
        }

        // 确保帮助信息不会与右上角的详细信息重叠
        if (help_y < 200) {
            help_y = 200; // 如果太靠上，调整到200像素以下
        }

        // 绘制帮助信息
        draw_text("Space: Pause/Resume", x, help_y);
        draw_text("R: Toggle Replay Mode", x, help_y + 20);
        draw_text("Left/Right: Navigate History", x, help_y + 40);
        draw_text("+/-: Adjust Decimals", x, help_y + 60);
        draw_text("G: Toggle Grid", x, help_y + 80);
        draw_text("C: Clear Trails", x, help_y + 100);
        draw_text("D: Toggle Details", x, help_y + 120);
        draw_text("Mouse: Zoom & Drag (Always)", x, help_y + 140);

        // 回放模式下的特殊提示
        if (replay_mode.load()) {
            draw_text("Replay: Use Left/Right to navigate", x, status_y - 20, { 255, 255, 0 });
        }
        };

    // 保存当前状态到历史记录
    auto save_current_history = [&]() {
        lock_guard<mutex> lock(history_mutex);
        HistoryState state;
        state.step = current_step.load();  // 修复：显式初始化
        state.bodies = b;
        state.real_time = current_step.load() * dt;  // 修复：显式初始化

        history.push_back(state);
        if (history.size() > MAX_HISTORY) {
            history.pop_front();
        }
        };

    // 加载历史状态
    auto load_history_state = [&](int target_step) {
        lock_guard<mutex> lock(history_mutex);
        if (history.empty()) return;

        // 找到最接近目标步数的历史记录
        auto best_it = history.begin();
        int best_diff = abs(best_it->step - target_step);

        for (auto it = history.begin(); it != history.end(); ++it) {
            int diff = abs(it->step - target_step);
            if (diff < best_diff) {
                best_diff = diff;
                best_it = it;
            }
        }

        // 复制历史状态到当前状态
        b = best_it->bodies;
        current_step.store(best_it->step);
        };

    while (!quit) {
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                quit = true;
                break;
            }

            if (e.type == SDL_WINDOWEVENT && e.window.event == SDL_WINDOWEVENT_RESIZED) {
                WIN_W = e.window.data1;
                WIN_H = e.window.data2;
            }

            // 键盘事件处理
            if (e.type == SDL_KEYDOWN) {
                switch (e.key.keysym.sym) {
                case SDLK_SPACE: // 暂停/继续
                    paused.store(!paused.load());
                    force_render.store(true);
                    break;

                case SDLK_r: // 回放模式切换
                    replay_mode.store(!replay_mode.load());
                    if (replay_mode.load()) {
                        // 进入回放模式时保存当前状态
                        save_current_history();
                    }
                    force_render.store(true);
                    break;

                case SDLK_LEFT: // 回溯历史
                    if (replay_mode.load()) {
                        lock_guard<mutex> lock(history_mutex);
                        if (!history.empty()) {
                            // 找到当前步数的前一个历史记录
                            for (auto it = history.rbegin(); it != history.rend(); ++it) {
                                if (it->step < current_step.load()) {
                                    b = it->bodies;
                                    current_step.store(it->step);
                                    break;
                                }
                            }
                            force_render.store(true);
                        }
                    }
                    break;

                case SDLK_RIGHT: // 前进历史
                    if (replay_mode.load()) {
                        lock_guard<mutex> lock(history_mutex);
                        if (!history.empty()) {
                            // 找到当前步数的下一个历史记录
                            for (const auto& state : history) {
                                if (state.step > current_step.load()) {
                                    b = state.bodies;
                                    current_step.store(state.step);
                                    break;
                                }
                            }
                            force_render.store(true);
                        }
                    }
                    break;

                case SDLK_PLUS:
                case SDLK_EQUALS: // 增加小数位数
                    if (decimal_places.load() < 6) {
                        decimal_places.store(decimal_places.load() + 1);
                        force_render.store(true);
                    }
                    break;

                case SDLK_MINUS: // 减少小数位数
                    if (decimal_places.load() > 0) {
                        decimal_places.store(decimal_places.load() - 1);
                        force_render.store(true);
                    }
                    break;

                case SDLK_g: // 显示/隐藏网格
                    show_grid.store(!show_grid.load());
                    force_render.store(true);
                    break;

                case SDLK_c: // 清空轨迹
                    clear_trails();
                    trails_cleared.store(true);
                    force_render.store(true);
                    break;

                case SDLK_d: // 显示/隐藏详细信息
                    show_details.store(!show_details.load());
                    force_render.store(true);
                    break;
                }
            }

            // 鼠标事件处理 - 在暂停状态下也可以缩放和拖动
            if (e.type == SDL_MOUSEWHEEL) {
                double old = scale;
                if (e.wheel.y > 0) scale *= 1.1;
                if (e.wheel.y < 0) scale /= 1.1;
                int mx, my;
                SDL_GetMouseState(&mx, &my);
                offsetX = mx - (mx - offsetX) * (scale / old);
                offsetY = my - (my - offsetY) * (scale / old);
                // 强制渲染
                force_render.store(true);
            }

            if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT) {
                dragging = true;
                lastX = e.button.x;
                lastY = e.button.y;
                // 强制渲染
                force_render.store(true);
            }

            if (e.type == SDL_MOUSEBUTTONUP && e.button.button == SDL_BUTTON_LEFT) {
                dragging = false;
                // 强制渲染
                force_render.store(true);
            }

            if (e.type == SDL_MOUSEMOTION && dragging) {
                offsetX += e.motion.x - lastX;
                offsetY += e.motion.y - lastY;
                lastX = e.motion.x;
                lastY = e.motion.y;
                // 强制渲染
                force_render.store(true);
            }
        }

        if (quit) break;

        // 检查是否需要渲染：ready为true（物理线程更新）或force_render为true（用户操作）
        bool should_render = ready.load() || force_render.load();

        if (should_render) {
            // 如果是用户操作强制渲染，重置标志
            if (force_render.load()) {
                force_render.store(false);
            }
            // 如果是物理线程更新，重置ready标志
            if (ready.load()) {
                ready.store(false);
            }

            // 在回放模式下，定期保存历史记录
            if (replay_mode.load() && current_step.load() % 50 == 0) {
                save_current_history();
            }

            SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
            SDL_RenderClear(ren);
            draw_grid();

            for (int i = 0; i < static_cast<int>(b.size()); i++) {
                // 只有在非回放模式且轨迹未被清空时才更新轨迹
                if (!replay_mode.load() && !trails_cleared.load()) {
                    trails[i].push_back({ b[i].x, b[i].y });
                    if (trails[i].size() > 5000) trails[i].erase(trails[i].begin());
                }

                vector<SDL_Point> pts(trails[i].size());
                for (int k = 0; k < static_cast<int>(pts.size()); k++) {
                    pts[k].x = wx(trails[i][k].first);
                    pts[k].y = wy(trails[i][k].second);
                }

                SDL_SetRenderDrawColor(ren,
                    bodyColor[i].r,
                    bodyColor[i].g,
                    bodyColor[i].b,
                    255);
                if (pts.size() > 1)
                    SDL_RenderDrawLines(ren, pts.data(), pts.size());

                SDL_SetRenderDrawColor(ren,
                    bodyColor[i].r,
                    bodyColor[i].g,
                    bodyColor[i].b,
                    255);

                double visual_radius = pow(b[i].mass, 1.0 / 3.0) * 1e-5;
                int R = max(2, static_cast<int>(visual_radius * scale));

                int sx = wx(b[i].x);
                int sy = wy(b[i].y);

                for (int dx = -R; dx <= R; dx++)
                    for (int dy = -R; dy <= R; dy++)
                        if (dx * dx + dy * dy <= R * R)
                            SDL_RenderDrawPoint(ren, sx + dx, sy + dy);
            }

            draw_legend();

            SDL_RenderPresent(ren);

            // 重置轨迹清空标志
            trails_cleared.store(false);
        }
        else {
            SDL_Delay(1);
        }
    }

    // 修复：确保资源被正确释放
    if (details_font && details_font != font) TTF_CloseFont(details_font);
    if (font) TTF_CloseFont(font);
    TTF_Quit();
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
}

// Verlet积分器函数实现
void verlet_integration_step(int id, int step, barrier<>& sync_point, vector<double>& old_x, vector<double>& old_y, ofstream& fout) {
    // 检查暂停或回放模式
    while ((paused.load() || replay_mode.load()) && !quit) {
        this_thread::sleep_for(chrono::milliseconds(10));
    }

    if (quit) return;

    // 更新当前步数
    if (id == 0) {
        current_step.store(step);
    }

    // --- 阶段 1：用 old_x, old_y 计算 a(t) ---
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
    ready.store(true);
    sync_point.arrive_and_wait();   // 所有人算完 a(t)
    // --- 阶段 2：半步速度 + 位置更新 ---
    b[id].vx += 0.5 * b[id].ax * dt;
    b[id].vy += 0.5 * b[id].ay * dt;
    b[id].x += b[id].vx * dt;
    b[id].y += b[id].vy * dt;
    sync_point.arrive_and_wait();   // 所有人更新完位置
    // --- 阶段 3：用新位置计算 a(t+dt) ---
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
    // --- 阶段 4：第二半步速度 ---
    b[id].vx += 0.5 * ax2 * dt;
    b[id].vy += 0.5 * ay2 * dt;
    // --- 阶段 5：刷新 old_x/old_y ---
    old_x[id] = b[id].x;
    old_y[id] = b[id].y;
    sync_point.arrive_and_wait();   // 所有人同步快照

    // 输出（只有线程0执行）
    if (id == 0 && step % 500 == 0) {
        double real_time = step * dt;
        fout << "Step " << step << " (Time: " << format_time(real_time) << ")\n";
        for (int k = 0; k < n; k++) {
            fout << "Body " << k << ": mass=" << b[k].mass
                << " x=" << format_number(b[k].x) << " y=" << format_number(b[k].y)
                << " vx=" << format_number(b[k].vx) << " vy=" << format_number(b[k].vy) << "\n";
        }
        fout << "\n";
    }
}

int main() {
    ofstream fout("orbit.txt");

    // 物理参数
    const double m_e = 5.972e24;
    const double m_s = 1.989e30;
    const double m_m = 7.348e22;
    const double d_earth_sun = AU;
    const double d_earth_moon = 3.844e8;

    // 1. 太阳
    b[0].mass = m_s;
    b[0].x = 0.0;
    b[0].y = 0.0;
    b[0].vx = 0.0;
    b[0].vy = 0.0;

    // 2. 地球
    double earth_orbit_speed = sqrt(G * m_s / d_earth_sun);
    b[1].mass = m_e;
    b[1].x = -d_earth_sun;
    b[1].y = 0.0;
    b[1].vx = 0.0;
    b[1].vy = earth_orbit_speed;

    // 3. 月球
    double moon_orbit_speed = sqrt(G * m_e / d_earth_moon);
    b[2].mass = m_m;
    b[2].x = b[1].x + d_earth_moon;
    b[2].y = 0.0;
    b[2].vx = 0.0;
    b[2].vy = b[1].vy + moon_orbit_speed;

    // 4. L4点
    double L4_x = -d_earth_sun * 0.5;
    double L4_y = sqrt(3) * d_earth_sun * 0.5;
    double r_L4 = sqrt(L4_x * L4_x + L4_y * L4_y);
    double v_L4 = sqrt(G * (m_s + m_e) / r_L4);
    double angle_L4 = atan2(L4_y, L4_x);
    b[3].mass = 1e10;
    b[3].x = L4_x;
    b[3].y = L4_y;
    b[3].vx = -v_L4 * sin(angle_L4);
    b[3].vy = v_L4 * cos(angle_L4);

    // 5. L5点
    double L5_x = -d_earth_sun * 0.5;
    double L5_y = -sqrt(3) * d_earth_sun * 0.5;
    double r_L5 = sqrt(L5_x * L5_x + L5_y * L5_y);
    double v_L5 = sqrt(G * (m_s + m_e) / r_L5);
    double angle_L5 = atan2(L5_y, L5_x);
    b[4].mass = 1e10;
    b[4].x = L5_x;
    b[4].y = L5_y;
    b[4].vx = -v_L5 * sin(angle_L5);
    b[4].vy = v_L5 * cos(angle_L5);

    // 调整初始条件，使系统总动量为零
    double total_momentum_x = 0.0;
    double total_momentum_y = 0.0;
    for (int i = 1; i < n; i++) {
        total_momentum_x += b[i].mass * b[i].vx;
        total_momentum_y += b[i].mass * b[i].vy;
    }
    // 给太阳一个相反的速度
    b[0].vx = -total_momentum_x / b[0].mass;
    b[0].vy = -total_momentum_y / b[0].mass;

    // 双缓冲：上一轮坐标快照
    vector<double> old_x(n), old_y(n);
    for (int i = 0; i < n; ++i) {
        old_x[i] = b[i].x;
        old_y[i] = b[i].y;
    }

    // barrier：每个时间步里用n次
    barrier<> sync_point(n);
    thread renderer(render_thread, ref(b));

    // 工作线程函数
    auto worker = [&](int id) {
        cout << "Thread " << id << " started" << endl;
        int step = 0;
        while (step < steps && !quit) {
            // 调用Verlet积分器函数
            verlet_integration_step(id, step, sync_point, old_x, old_y, fout);
            step++;
        }
        };

    // 固定线程池
    vector<thread> threads;
    for (int i = 0; i < n; ++i) {
        threads.emplace_back(worker, i);
    }
    for (auto& t : threads) t.join();
    renderer.join();

    return 0;
}
