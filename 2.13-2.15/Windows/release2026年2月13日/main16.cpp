#include <iostream>
#include <fstream> 
#include <vector>
#include <thread>
#include <barrier>
#include <cmath>
using namespace std;
const double GMsun = 1.32712440018e20;   // m^3/s^2
const double GMearth = 3.986004418e14;    // m^3/s^2
const double Msun = 1.98847e30;        // kg
const double Mearth = 5.9722e24;         // kg
const double G = 6.67430e-11;            // m^3/kg/s^2
const double R = 1.495978707e11;          // 日地平均距离 (m)
const double dt = 10.0;					 // 时间步长 (s)
const double GMmoon = 4.9048695e12;       // m^3/s^2
const double Mmoon = 7.34767309e22;      // kg
const double Rm = 3.844e8;                 // 地月平均距离 (m)
struct Body {
    double mass;
    double x, y;
    double vx, vy;
    double ax, ay;
};

int main() {
    const int n = 4;   // Sun, Earth, L4
    const int steps = 2e6;
    ofstream fout("orbit.txt");
    vector<Body> b(n);
    // ---------------- Sun ----------------
    b[0].mass = Msun;
    b[0].x = 0;
    b[0].y = 0;
    b[0].vx = 0;
    b[0].vy = -0.0890;
    // ---------------- Earth ----------------
    b[1].mass = Mearth;
    b[1].x = R - 4670000;
    b[1].y = 0;
    b[1].vx = 0;
    b[1].vy = 29780.0 - (Mmoon / (Mearth + Mmoon)) * n * Rm;
    // ---------------- Moon ----------------
    b[2].mass = Mmoon;
    b[2].x = R + (Rm - 4670000);
    b[2].y = 0;
    b[2].vx = 0;
    b[2].vy = 29780.0 + (Mearth / (Mearth + Mmoon)) * n * Rm;
    // ---------------- L4 ----------------
    b[3].mass = 1e20;
    b[3].x = R * 0.5;
    b[3].y = R * 0.866025403784;
    b[3].vx = -b[3].y / R * 29780.0;
    b[3].vy = b[3].x / R * 29780.0;

    // 双缓冲：上一轮坐标快照
    vector<double> old_x(n), old_y(n);
    for (int i = 0; i < n; ++i) {
        old_x[i] = b[i].x;
        old_y[i] = b[i].y;
    }

    // barrier：每个时间步里用三次
    barrier sync_point(n);

    // 工作线程函数
    auto worker = [&](int id) {
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
                fout << "Sun  : x=" << b[0].x << " y=" << b[0].y << "\n";
                fout << "Earth: x=" << b[1].x << " y=" << b[1].y << "\n";
                fout << "Moon : x=" << b[2].x << " y=" << b[2].y << "\n";
                fout << "L4   : x=" << b[3].x << " y=" << b[3].y << "\n\n";
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
