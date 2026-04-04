#define _USE_MATH_DEFINES
#include <iostream>
#include <vector>
#include <thread>
#include <barrier>
#include <cmath>
#include <atomic>
#include <deque>
#include <string>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

using namespace std;

// --- 全局配置 ---
const double G = 6.67430e-11;
const double AU = 1.496e11;
double base_dt = 3600.0; 
float timeScale = 1.0f; // 时间缩放倍率
const int physics_steps = 1000;

struct Body {
    string name;
    double mass;
    glm::dvec3 pos, vel;
    glm::vec3 color;
    deque<glm::dvec3> trail;
    bool showTrail = true;
    bool isSpaceship = false;
};

vector<Body> b;
vector<glm::dvec3> shared_pos;
atomic<bool> quit{ false };
bool showUI = true;
bool showAxes = false;
int focusID = 0;

float camYaw = 0.5f, camPitch = 0.4f, camDist = 150.0f;
double lastX, lastY;

void scroll_callback(GLFWwindow* window, double x, double y) {
    if (y > 0) camDist *= 0.85f; else camDist *= 1.15f;
    camDist = glm::clamp(camDist, 0.05f, 20000.0f);
}

// 物理计算：加入 timeScale 动态调整
void physics_worker(int id, std::barrier<>& sync) {
    while (!quit) {
        int n = b.size(); 
        double dt = base_dt * (double)timeScale;
        
        for (int s = 0; s < physics_steps; ++s) {
            b[id].pos += b[id].vel * dt;
            shared_pos[id] = b[id].pos;
            sync.arrive_and_wait();
            
            glm::dvec3 acc{ 0,0,0 };
            for (int j = 0; j < n; ++j) {
                if (j == id) continue;
                glm::dvec3 diff = shared_pos[j] - shared_pos[id];
                double r2 = glm::dot(diff, diff) + 1e8;
                acc += (G * b[j].mass / (r2 * sqrt(r2))) * diff;
            }
            b[id].vel += acc * dt;
            sync.arrive_and_wait();
        }
    }
}

bool ProjectToScreen(const glm::dvec3& worldPos, const glm::mat4& viewProj, int w, int h, ImVec2& outPos) {
    glm::vec4 clipPos = viewProj * glm::vec4(worldPos, 1.0f);
    if (clipPos.w <= 0) return false;
    glm::vec3 ndc = glm::vec3(clipPos) / clipPos.w;
    outPos.x = (ndc.x + 1.0f) * 0.5f * (float)w;
    outPos.y = (1.0f - ndc.y) * 0.5f * (float)h;
    return true;
}

// 绘制倾斜土星环的函数
void DrawSaturnRings(ImDrawList* drawList, const glm::dvec3& saturnPos, double scale, const glm::mat4& viewProj, int w, int h) {
    const int segments = 64;
    float ringRadii[] = { 1.5f, 1.8f, 2.1f }; // 相对于行星大小的半径
    
    for (float rFactor : ringRadii) {
        vector<ImVec2> points;
        for (int i = 0; i <= segments; i++) {
            float angle = (i / (float)segments) * 2.0f * M_PI;
            // 创建在 XZ 平面上的圆，并增加一点倾斜（27度）
            glm::vec3 localPos = glm::vec3(cos(angle), sin(angle) * 0.2f, sin(angle)) * (float)(15.0 * rFactor); 
            ImVec2 sp;
            if (ProjectToScreen(saturnPos + (glm::dvec3)localPos / scale, viewProj, w, h, sp)) {
                points.push_back(sp);
            }
        }
        if (points.size() > 1)
            drawList->AddPolyline(points.data(), points.size(), IM_COL32(200, 180, 150, 100), 0, 1.5f);
    }
}

void render_loop() {
    if (!glfwInit()) return;
    GLFWwindow* window = glfwCreateWindow(1440, 900, "Solar Lab - Gravity Slingshot & Rings", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSetScrollCallback(window, scroll_callback);

    IMGUI_CHECKVERSION(); ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    while (!glfwWindowShouldClose(window) && !quit) {
        glfwPollEvents();
        if (!ImGui::GetIO().WantCaptureMouse) {
            double mx, my; glfwGetCursorPos(window, &mx, &my);
            if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
                camYaw += (float)(mx - lastX) * 0.005f;
                camPitch += (float)(my - lastY) * 0.005f;
            }
            lastX = mx; lastY = my;
        }

        ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame(); ImGui::NewFrame();
        int w, h; glfwGetFramebufferSize(window, &w, &h);
        glViewport(0, 0, w, h); glClearColor(0.005f, 0.005f, 0.01f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT); glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        double drawScale = 2.0e-9;
        glm::dvec3 focusPos = b[focusID].pos * drawScale;
        glm::mat4 viewProj = glm::perspective(glm::radians(45.0f), (float)w/h, 0.1f, 30000.0f) * glm::lookAt(glm::vec3(focusPos) + glm::vec3(camDist * cos(camPitch) * sin(camYaw), camDist * sin(camPitch), camDist * cos(camPitch) * cos(camYaw)), 
                                         glm::vec3(focusPos), glm::vec3(0, 1, 0));

        ImDrawList* drawList = ImGui::GetBackgroundDrawList();

        for (int i = 0; i < (int)b.size(); i++) {
            glm::dvec3 rPos = b[i].pos * drawScale;
            int r = (int)(b[i].color.x * 255), g = (int)(b[i].color.y * 255), bv = (int)(b[i].color.z * 255);

            // 绘制轨迹
            if (b[i].showTrail && b[i].trail.size() > 1) {
                for (size_t j = 0; j < b[i].trail.size() - 1; j++) {
                    ImVec2 p1, p2;
                    if (ProjectToScreen(b[i].trail[j], viewProj, w, h, p1) && ProjectToScreen(b[i].trail[j+1], viewProj, w, h, p2))
                        drawList->AddLine(p1, p2, IM_COL32(r, g, bv, b[i].isSpaceship ? 255 : 150), b[i].isSpaceship ? 1.5f : 2.5f);
                }
            }
            if (b[i].trail.size() > 3000) b[i].trail.pop_front();
            b[i].trail.push_back(rPos);

            ImVec2 screenPos;
            if (ProjectToScreen(rPos, viewProj, w, h, screenPos)) {
                float size = (i == 0) ? 15.0f : (b[i].mass > 1e26 ? 8.0f : 4.0f);
                if (b[i].isSpaceship) size = 3.0f;
                drawList->AddCircleFilled(screenPos, size * 1.5f, IM_COL32(r, g, bv, 80));
                drawList->AddCircleFilled(screenPos, size, IM_COL32(r, g, bv, 255));
                
                // 如果是土星，画环
                if (b[i].name == "Saturn") DrawSaturnRings(drawList, b[i].pos, drawScale, viewProj, w, h);
            }
        }

        if (showUI) {
            ImGui::SetNextWindowPos(ImVec2(20, 20), ImGuiCond_FirstUseEver);
            ImGui::Begin("Lab Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
            ImGui::SliderFloat("Time Scale", &timeScale, 0.0f, 20.0f, "%.1fx");
            if (ImGui::Button("Launch Voyager (Spaceship)", ImVec2(-1, 0))) {
                // 弹弓实验：从地球附近发射一个极轻的物体
                Body ship = {"Voyager", 1000.0, b[3].pos + glm::dvec3(1e9,0,0), b[3].vel * 1.5, {1,1,1}, {}, true, true};
                b.push_back(ship); 
                shared_pos.push_back(ship.pos);
                // 注意：实际应用中这里需要重启物理线程，简化起见建议初始化时加入或预留空位
            }
            ImGui::Separator();
            for (int i = 0; i < (int)b.size(); i++) {
                if (ImGui::Selectable(b[i].name.c_str(), focusID == i)) focusID = i;
            }
            ImGui::End();
        }
        ImGui::Render(); ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }
    quit = true;
    glfwTerminate();
}

int main() {
    b = {
        {"Sun",     1.989e30, {0,0,0}, {0,0,0}, {1.0f, 0.9f, 0.1f}},
        {"Mercury", 3.301e23, {0.387*AU, 0, 0}, {0, 0, 47400}, {0.7f, 0.7f, 0.7f}},
        {"Venus",   4.867e24, {0.723*AU, 0, 0}, {0, 0, 35000}, {1.0f, 0.8f, 0.4f}},
        {"Earth",   5.972e24, {1.0*AU, 0, 0}, {0, 0, 29780}, {0.1f, 0.5f, 1.0f}},
        {"Mars",    6.39e23,  {1.524*AU, 0, 0}, {0, 0, 24000}, {1.0f, 0.2f, 0.1f}},
        {"Jupiter", 1.898e27, {5.203*AU, 0, 0}, {0, 0, 13070}, {0.8f, 0.7f, 0.5f}},
        {"Saturn",  5.683e26, {9.537*AU, 0, 0}, {0, 0, 9690}, {0.9f, 0.8f, 0.4f}},
        {"Uranus",  8.681e25, {19.191*AU, 0, 0}, {0, 0, 6810}, {0.5f, 0.9f, 1.0f}},
        {"Neptune", 1.024e26, {30.069*AU, 0, 0}, {0, 0, 5430}, {0.2f, 0.3f, 1.0f}},
        {"Voyager", 1e4,      {1.1*AU, 0, 0}, {0, 0, 35000}, {1.0f, 1.0f, 1.0f}, {}, true, true}
    };

    shared_pos.resize(b.size());
    for (int i = 0; i < b.size(); i++) shared_pos[i] = b[i].pos;
    std::barrier sync(b.size());
    vector<thread> workers;
    for (int i = 0; i < b.size(); i++) workers.emplace_back(physics_worker, i, ref(sync));
    render_loop();
    for (auto& t : workers) t.join();
    return 0;
}