#include <iostream>
#include <fstream> 
#include <vector>
#include <thread>
#include <barrier>
#include <cmath>

using namespace std;

const double G = 6.67430e-11;
const double dt = 100;   // Verlet 稳定性很好，可以比 Euler 大很多

struct Body {
    double mass;
    double x, y;
    double vx, vy;
    double ax, ay;   // 当前加速度
};

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

    // barrier：每个时间步里用三次
    std::barrier sync_point(n);

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

    return 0;
}
