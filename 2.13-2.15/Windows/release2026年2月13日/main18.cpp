#include <iostream>
#include <fstream> 
#include <vector>
#include <thread>
#include <barrier>
#include <cmath>
#include <SDL2/SDL.h>
#include <atomic>
#include <string>
#include <SDL2/SDL_ttf.h>
#undef main
using namespace std;
const double G = 6.67430e-11;
const double dt = 100;   // Verlet 稳定性很好，可以比 Euler 大很多
struct Body {
    double mass;
    double x, y;
    double vx, vy;
    double ax, ay;   // 当前加速度
};
//渲染
atomic<bool> ready(false);
void render_thread(vector<Body>& b) {
    SDL_Init(SDL_INIT_VIDEO);
    TTF_Init();

    int WIN_W = 1000;
    int WIN_H = 800;

    SDL_Window* win = SDL_CreateWindow("N-body Simulation",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WIN_W, WIN_H, SDL_WINDOW_SHOWN);

    SDL_Renderer* ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    // 加载字体（你可以换成 msyh.ttc）
    TTF_Font* font = TTF_OpenFont("C:/Windows/Fonts/arial.ttf", 16);
    if (!font) {
        printf("Font load error: %s\n", TTF_GetError());
        return;
    }

    vector<vector<pair<double, double>>> trails(b.size());

    double scale = 1e-9;
    double offsetX = WIN_W / 2;
    double offsetY = WIN_H / 2;

    bool dragging = false;
    int lastX = 0, lastY = 0;

    SDL_Event e;

    auto wx = [&](double x) { return int(x * scale + offsetX); };
    auto wy = [&](double y) { return int(y * scale + offsetY); };

    auto sx_to_world = [&](int sx) { return (sx - offsetX) / scale; };
    auto sy_to_world = [&](int sy) { return (sy - offsetY) / scale; };

    // 科学计数法格式化
    auto format_number = [&](double x) {
        if (x == 0) return string("0");
        double absx = fabs(x);
        if (absx < 1e3) return to_string((long long)x);

        int exp = floor(log10(absx));
        double base = x / pow(10, exp);

        char buf[32];
        snprintf(buf, sizeof(buf), "%.2fe%d", base, exp);
        return string(buf);
        };

    // 自动选择网格间距
    auto choose_grid_step = [&]() {
        double steps[] = { 1e6, 1e7, 1e8, 1e9, 5e9, 1e10, 5e10, 1e11 };
        for (double s : steps) {
            if (s * scale > 60) return s;
        }
        return 1e12;
        };

    // 绘制文字
    auto draw_text = [&](string text, int x, int y) {
        SDL_Color color = { 200, 200, 200 };
        SDL_Surface* surf = TTF_RenderText_Solid(font, text.c_str(), color);
        SDL_Texture* tex = SDL_CreateTextureFromSurface(ren, surf);
        SDL_Rect r = { x, y, surf->w, surf->h };
        SDL_RenderCopy(ren, tex, NULL, &r);
        SDL_FreeSurface(surf);
        SDL_DestroyTexture(tex);
        };

    // 绘制网格 + 坐标轴 + 刻度数字
    auto draw_grid = [&]() {
        double grid_step = choose_grid_step();

        SDL_SetRenderDrawColor(ren, 50, 50, 50, 255);

        double startX = sx_to_world(0);
        double endX = sx_to_world(WIN_W);
        double gx = floor(startX / grid_step) * grid_step;

        for (; gx < endX; gx += grid_step) {
            int sx = wx(gx);
            SDL_RenderDrawLine(ren, sx, 0, sx, WIN_H);

            draw_text(format_number(gx), sx + 3, offsetY + 8);
        }

        double startY = sy_to_world(0);
        double endY = sy_to_world(WIN_H);
        double gy = floor(startY / grid_step) * grid_step;

        for (; gy < endY; gy += grid_step) {
            int sy = wy(gy);
            SDL_RenderDrawLine(ren, 0, sy, WIN_W, sy);

            draw_text(format_number(gy), offsetX + 8, sy + 3);
        }

        SDL_SetRenderDrawColor(ren, 120, 120, 120, 255);
        SDL_RenderDrawLine(ren, 0, offsetY, WIN_W, offsetY);
        SDL_RenderDrawLine(ren, offsetX, 0, offsetX, WIN_H);
        };

    // 图例
    auto draw_legend = [&]() {
        int x = 20, y = 20;

        for (int i = 0; i < b.size(); i++) {
            if (i == 0) SDL_SetRenderDrawColor(ren, 100, 150, 255, 255);
            else if (i == 1) SDL_SetRenderDrawColor(ren, 200, 200, 200, 255);
            else SDL_SetRenderDrawColor(ren, 255, 200, 0, 255);

            SDL_Rect rect = { x, y + i * 25, 15, 15 };
            SDL_RenderFillRect(ren, &rect);

            draw_text("Body " + to_string(i), x + 25, y + i * 25);
        }
        };

    while (true) {
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) return;

            if (e.type == SDL_MOUSEWHEEL) {
                double old_scale = scale;
                if (e.wheel.y > 0) scale *= 1.1;
                if (e.wheel.y < 0) scale /= 1.1;

                int mx, my;
                SDL_GetMouseState(&mx, &my);
                offsetX = mx - (mx - offsetX) * (scale / old_scale);
                offsetY = my - (my - offsetY) * (scale / old_scale);
            }

            if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT) {
                dragging = true;
                lastX = e.button.x;
                lastY = e.button.y;
            }
            if (e.type == SDL_MOUSEBUTTONUP && e.button.button == SDL_BUTTON_LEFT) {
                dragging = false;
            }
            if (e.type == SDL_MOUSEMOTION && dragging) {
                offsetX += e.motion.x - lastX;
                offsetY += e.motion.y - lastY;
                lastX = e.motion.x;
                lastY = e.motion.y;
            }
        }

        if (!ready.load()) {
            SDL_Delay(1);
            continue;
        }

        SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
        SDL_RenderClear(ren);

        draw_grid();

        for (int i = 0; i < b.size(); i++) {
            trails[i].push_back({ b[i].x, b[i].y });
            if (trails[i].size() > 5000) trails[i].erase(trails[i].begin());

            vector<SDL_Point> pts(trails[i].size());
            for (int k = 0; k < trails[i].size(); k++) {
                pts[k].x = wx(trails[i][k].first);
                pts[k].y = wy(trails[i][k].second);
            }

            SDL_SetRenderDrawColor(ren, 150, 150, 150, 255);
            if (pts.size() > 1)
                SDL_RenderDrawLines(ren, pts.data(), pts.size());

            if (i == 0) SDL_SetRenderDrawColor(ren, 100, 150, 255, 255);
            else if (i == 1) SDL_SetRenderDrawColor(ren, 200, 200, 200, 255);
            else SDL_SetRenderDrawColor(ren, 255, 200, 0, 255);

            double visual_radius = pow(b[i].mass, 1.0 / 3.0) * 1e-5;
            int R = max(2, int(visual_radius * scale));

            int sx = wx(b[i].x);
            int sy = wy(b[i].y);

            for (int dx = -R; dx <= R; dx++)
                for (int dy = -R; dy <= R; dy++)
                    if (dx * dx + dy * dy <= R * R)
                        SDL_RenderDrawPoint(ren, sx + dx, sy + dy);
        }

        draw_legend();

        SDL_RenderPresent(ren);
        ready.store(false);
        SDL_Delay(16);
    }

    TTF_CloseFont(font);
    TTF_Quit();
}


int main() {
    const int n = 3;          // 天体数量
    const int steps = 2e7;  // 模拟步数
    ofstream fout("orbit.txt");
    vector<Body> b(n);
    // 地球
    b[0].mass = 5.972e24;
    b[0].x = -149600000000.0;
    b[0].y = 0.0;
    b[0].vx = 0.0;
    b[0].vy = -29780.0;
    // 月球
    b[1].mass = 7.348e22;
    b[1].x = -149215600000.0;
    b[1].y = 0.0;
    b[1].vx = 0.0;
    b[1].vy = -28758.0;
    //太阳
	b[2].mass = 1.989e30;
	b[2].x = 0.0;
	b[2].y = 0.0;
	b[2].vx = 0.0;
	b[2].vy = 0.0;
    // 双缓冲：上一轮坐标快照
    vector<double> old_x(n), old_y(n);
    for (int i = 0; i < n; ++i) {
        old_x[i] = b[i].x;
        old_y[i] = b[i].y;
    }
    // barrier：每个时间步里用n次
    std::barrier sync_point(n);
    thread renderer(render_thread, std::ref(b));
    // 工作线程函数
    auto worker = [&](int id) {
        cout << "Thread " << id << " started" << endl;
        for (int step = 0; step < steps; ++step) {

            // --- 阶段 1：用 old_x, old_y 计算 a(t) ---
            double ax = 0.0, ay = 0.0;
            for (int j = 0; j < n; ++j) {
                if (j == id) continue;
                double dx = old_x[j] - old_x[id];
                double dy = old_y[j] - old_y[id];
                double r2 = dx * dx + dy * dy;
                double r = std::sqrt(r2);
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
                double r = std::sqrt(r2);
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

            if (id == 0 && step % 500 == 0) {
                fout << "step " << step << "\n";
                fout << "Earth: x=" << b[0].x << " y=" << b[0].y << "\n";
                fout << "Moon : x=" << b[1].x << " y=" << b[1].y << "\n";
                fout << "Sun  : x=" << b[2].x << " y=" << b[2].y << "\n\n";
            }
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
