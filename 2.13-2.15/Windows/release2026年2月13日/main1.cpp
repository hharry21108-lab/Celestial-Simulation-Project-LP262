#include <iostream>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
//绘制监视库
#include <GL/glew.h>
#include <GLFW/glfw3.h>
using namespace std;
const long double G = 6.67430e-11;
///引力场强度
long double Gravitational_Field_Strength(long double dtx, long double dty, long double M){
	long double r2 = dtx * dtx + dty * dty;
	return -G * M / r2;
}
///存储空间坐标对应加速度
class Grid2D {
public:
    Grid2D(long double xmin, long double xmax,
        long double ymin, long double ymax,
        long double dx, long double dy)
        : xmin(xmin), xmax(xmax), ymin(ymin), ymax(ymax),
        dx(dx), dy(dy){    
        nx = static_cast<size_t>(llround((xmax - xmin) / dx)) + 1;
        ny = static_cast<size_t>(llround((ymax - ymin) / dy)) + 1;
        data = new long double[nx * ny];
    }

    ~Grid2D() {
        delete[] data;
    }

    inline size_t ix(long double x) const {//将小数坐标映射到索引
        return static_cast<size_t>((x - xmin) / dx + 0.5L);
    }
    inline size_t iy(long double y) const {
        return static_cast<size_t>((y - ymin) / dy + 0.5L);
    }

    inline long double& operator()(long double x, long double y) {//通过小数坐标访问
        size_t i = ix(x);
        size_t j = iy(y);
        return data[j * nx + i];
    }
    
    inline long double& at(size_t i, size_t j) {//通过整数索引访问
        return data[j * nx + i];
    }
    
    inline long double xcoord(size_t i) const { return xmin + i * dx; }//获取坐标
    inline long double ycoord(size_t j) const { return ymin + j * dy; }
    size_t width() const { return nx; }
    size_t height() const { return ny; }

private:
    long double xmin, xmax, ymin, ymax;
    long double dx, dy;
    size_t nx, ny;
    long double* data;
};
/// 存储天体参数、坐标哈希表
struct Point {//天体参数
    long double x;
    long double y;
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
};
struct PointHash {//结构体核对哈希
    size_t operator()(const Point& k) const {
        auto h1 = hash<long double>{}(k.x);
        auto h2 = hash<long double>{}(k.y);
        return h1 ^ (h2 << 1);
    }
};
struct Point_Quantity{
    long double mass = 0.0L;
    long double vx = 0.0L;
    long double vy = 0.0L;
};
class Celestial_Body_Grid2D {//天体坐标哈希表
public:
    unordered_map<Point, Point_Quantity, PointHash> data;
    Point_Quantity& operator()(long double x, long double y) {
        return data[{x, y}];
    }
    bool exists(long double x, long double y) const {
        return data.find({ x, y }) != data.end();
    }
};



int main() {
    Grid2D grid(-1.0L, 1.0L, -1.0L, 1.0L, 0.001L, 0.001L);

    // 写入
    grid(0.123456789L, -0.987654321L) = 3.1415926535897932384626L;

    // 读取
    long double v = grid(0.123456789L, -0.987654321L);

    cout << v << "\n";
    Celestial_Body_Grid2D grid1;

    // 写入
    grid1(0.123456789L, -0.987654321L).mass = 6.28000;

    // 读取
    long double v1 = grid1(0.123456789L, -0.987654321L).mass;

    cout << v1 << "\n";


}
