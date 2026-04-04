#include <iostream>
#include <vector>
#include <thread>
#include <barrier>
#include <chrono>
using namespace std;
const double G = 6.67430e-11;
constexpr double dt = 0.01;
///引力场强度（矢量，不是标量）
double Gravitational_Field_Strength_x(double dtx, double dty, double M) {
    double r2 = dtx * dtx + dty * dty;
    double r = std::sqrt(r2);
    return -(G * M / (r2 * r)) * dtx;   // GM * dtx / r^3
}

double Gravitational_Field_Strength_y(double dtx, double dty, double M) {
    double r2 = dtx * dtx + dty * dty;
    double r = std::sqrt(r2);
    return -(G * M / (r2 * r)) * dty;   // GM * dty / r^3
}
/// 存储天体参数、坐标哈希表
struct Point {//天体参数
    double x;
    double y;
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
};
struct PointHash {//结构体核对哈希
    size_t operator()(const Point& k) const {
        auto h1 = hash<double>{}(k.x);
        auto h2 = hash<double>{}(k.y);
        return h1 ^ (h2 << 1);
    }
};
struct Point_Quantity{
    double mass = 0.0L;
    double x = 0.0L;
    double y = 0.0L;
    double vx = 0.0L;
    double vy = 0.0L;
    double ax = 0.0L;
    double ay = 0.0L;
};
class Celestial_Body_Grid2D {
public:
    std::vector<Point_Quantity> data;

    // 确保 vector 足够大
    void ensure_size(int index) {
        if (index >= data.size()) {
            data.resize(index + 1);
        }
    }
    // 按 number_of_bodies 访问
    Point_Quantity& operator()(int number_of_bodies) {
        ensure_size(number_of_bodies);
        return data[number_of_bodies];
    }

    bool exists(int number_of_bodies) const {
        return number_of_bodies < data.size();
    }
};

///多线程各天体各运动
int n = 2;//运动体数目
Celestial_Body_Grid2D point;
barrier sync_point(n);
void upgrade_body(int id,double x,double y) {
    using clock = chrono::steady_clock;
    auto next_time = clock::now(); // 下一步开始的目标时间
    for (size_t number = 0; number < point.data.size(); ++number) {
        Point_Quantity& pq = point.data[number];
        if (number == id) {
            continue;
        }
        double dtx = pq.x - x;
        double dty = pq.y - y;
		point.data[id].ax = Gravitational_Field_Strength_x(dtx, dty, pq.mass);
		point.data[id].ay = Gravitational_Field_Strength_y(dtx, dty, pq.mass);		
    }
    sync_point.arrive_and_wait();
    point.data[id].vx += point.data[id].ax * dt;
    point.data[id].vy += point.data[id].ay * dt;
    point.data[id].x += point.data[id].vx * dt;
    point.data[id].y += point.data[id].vy * dt;
    sync_point.arrive_and_wait();
    next_time += chrono::duration_cast<clock::duration>(//时间累计
        chrono::duration<double>(dt)
    );
    
};
int main() {
	point(0).mass = 5.972e24; // 地球质量
	point(0).x = 0.0;
	point(0).y = 0.0;  
	point(0).vx = 0.0;
	point(0).vy = 0.0;
	point(1).mass = 7.348e22; // 月球质量
	point(1).x = 384400000.0; // 月球距离地球平均距离
	point(1).y = 0.0;
	point(1).vx = 0.0;
	point(1).vy = 1022.0; // 月球绕地球公转速度
    int i = 0;
    while (i <= 50) {
        vector<thread> threads;
        for (size_t number = 0; number < point.data.size(); ++number) {
            Point_Quantity& pq = point.data[number];
            threads.emplace_back(upgrade_body, number, pq.x, pq.y);
        }
        for (auto& th : threads) {
            th.join();
		}
		printf("After step %d:\n", i);
		cout << "Earth x:" << point(0).x << "Earth y:" << point(0).y << endl;
		cout << "Moon x:" << point(1).x << "Moon y:" << point(1).y << endl;
        i++;
    }
    return 0;
}