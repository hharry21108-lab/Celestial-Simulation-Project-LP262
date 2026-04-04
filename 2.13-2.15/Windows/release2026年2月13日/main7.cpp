#include <iostream>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <thread>
#include <barrier>
#include <chrono>
using namespace std;
const double G = 6.67430e-11;
constexpr double dt = 0.01;
///引力场强度
double Gravitational_Field_Strength_x(double dtx,double dty,double M){
	double r2 = dtx * dtx + dty * dty;
	return  -(G * M / r2 )* dtx / r2;
}
double Gravitational_Field_Strength_y(double dtx, double dty, double M) {
    double r2 = dtx * dtx + dty * dty;
    return  -(G * M / r2) * dty / r2;
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
int n;
Celestial_Body_Grid2D point;
barrier sync_point(n);
void upgrade_body_g(int id,double x,double y) {
    using clock = chrono::steady_clock;
    auto next_time = clock::now(); // 下一步开始的目标时间
    for (const auto& kv : point.data) {
        const Point& p = kv.first;              // 坐标
        const Point_Quantity& q = kv.second;    // 天体参数
		if (q.number_of_bodies == id) {
            continue;
        }
		double dtx = p.x - x;
		double dty = p.y - y;
		point.data[{x, y}].ax = Gravitational_Field_Strength_x(dtx, dty, q.mass);
        point.data[{x, y}].ay = Gravitational_Field_Strength_y(dtx, dty, q.mass);
		point.data[{x, y}].vx += point.data[{x, y}].ax* dt;
		point.data[{x, y}].vy += point.data[{x, y}].ay* dt;


    }
    sync_point.arrive_and_wait();
    next_time += chrono::duration_cast<clock::duration>(//时间累计
        chrono::duration<double>(dt)
    );

};
int main() {
    
    int i = 0;
	n = point.data.size();
    while (i!=1) {
        vector<thread> threads;
        for (size_t number = 0; number < point.data.size(); ++number) {
            Point_Quantity& pq = point.data[number];
            threads.emplace_back(upgrade_body_g, number, pq.x, pq.y);
            // 使用 pq.mass, pq.vx, pq.vy, pq.ax, pq.ay ...
        }
        i++;
    }
    return 0;
}