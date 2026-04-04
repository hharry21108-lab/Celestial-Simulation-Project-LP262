#define _CRT_SECURE_NO_WARNINGS // 修复 gmtime 安全警告
#define _USE_MATH_DEFINES
#include <iostream>
#include <vector>
#include <thread>
#include <barrier>
#include <cmath>
#include <atomic>
#include <deque>
#include <string>
#include <iomanip>
#include <ctime>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

using namespace std;

enum SimMode { MODE_LIVE, MODE_PAUSED, MODE_PLAYBACK };

// --- 天体结构体 ---
struct Body {
    string name;
    double mass;
    glm::dvec3 pos, vel;
    glm::dvec3 lastAcc{ 0,0,0 };
    glm::vec3 color;
    deque<glm::dvec3> trail;
    vector<glm::dvec3> history;
    bool showTrail = true;
    bool isSpaceship = false;

    Body(string n, double m, glm::dvec3 p, glm::dvec3 v, glm::vec3 c, bool ship = false)
        : name(n), mass(m), pos(p), vel(v), color(c), isSpaceship(ship) {
    }
};

// --- 全局配置 ---
const double G = 6.67430e-11;
const double AU = 1.496e11;
double base_dt = 360.0;
float timeScale = 1.0f;
double totalSimSeconds = 0;
SimMode currentMode = MODE_LIVE;
int playbackFrame = 0;
const int max_history = 15000;

vector<Body> b;
vector<glm::dvec3> shared_pos;
atomic<bool> quit{ false };
bool showUI = true;
int focusID = 0;

// --- 相机控制变量 ---
bool freeCamera = false;
float camYaw = 0.5f, camPitch = 0.4f, camDist = 150.0f;
glm::dvec3 freeCamPos = glm::dvec3(0, 100, 300);
float moveSpeed = 1.0f;
double lastX = 0, lastY = 0;

// 格式化时间函数
string GetSimulatedDateTime(double elapsedSeconds) {
    struct tm start_time = { 0 };
    start_time.tm_year = 126; // 2026年 (1900 + 126)
    start_time.tm_mon = 0;
    start_time.tm_mday = 1;

    time_t raw_start = mktime(&start_time);
    time_t current_raw = raw_start + (time_t)elapsedSeconds;

    struct tm* now = gmtime(&current_raw);
    if (!now) return "N/A";
    char buf[64];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", now);
    return string(buf);
}

void scroll_callback(GLFWwindow* window, double x, double y) {
    if (ImGui::GetIO().WantCaptureMouse) return;
    if (freeCamera) {
        glm::vec3 forward = glm::vec3(sin(camYaw) * cos(camPitch), -sin(camPitch), -cos(camYaw) * cos(camPitch));
        double s = (double)camDist * 0.1;
        if (y > 0) freeCamPos += (glm::dvec3)forward * s; else freeCamPos -= (glm::dvec3)forward * s;
    }
    else {
        if (y > 0) camDist *= 0.85f; else camDist *= 1.15f;
        camDist = glm::clamp(camDist, 0.05f, 80000.0f);
    }
}

void physics_worker(int id, std::barrier<>& sync) {
    while (!quit) {
        if (currentMode != MODE_LIVE) {
            this_thread::sleep_for(chrono::milliseconds(10));
            continue;
        }
        double dt = base_dt * (double)timeScale;
        for (int s = 0; s < 500; ++s) {
            b[id].pos += b[id].vel * dt;
            shared_pos[id] = b[id].pos;
            sync.arrive_and_wait();

            glm::dvec3 acc{ 0,0,0 };
            for (int j = 0; j < (int)b.size(); ++j) {
                if (j == id) continue;
                glm::dvec3 diff = shared_pos[j] - shared_pos[id];
                double r2 = glm::dot(diff, diff) + 1e8;
                acc += (G * b[j].mass / (r2 * sqrt(r2))) * diff;
            }
            b[id].lastAcc = acc;
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

void render_loop() {
    if (!glfwInit()) return;
    GLFWwindow* window = glfwCreateWindow(1440, 900, "Solar Lab Pro - 2026 Edition", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSetScrollCallback(window, scroll_callback);
    IMGUI_CHECKVERSION(); ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    while (!glfwWindowShouldClose(window) && !quit) {
        glfwPollEvents();

        if (currentMode == MODE_LIVE) {
            totalSimSeconds += (base_dt * timeScale * 500) / 60.0;
            for (auto& body : b) {
                body.history.push_back(body.pos);
                if (body.history.size() > max_history) body.history.erase(body.history.begin());
            }
        }

        if (!ImGui::GetIO().WantCaptureMouse) {
            double mx, my; glfwGetCursorPos(window, &mx, &my);
            double dx = mx - lastX; double dy = my - lastY;
            if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
                camYaw += (float)dx * 0.005f; camPitch += (float)dy * 0.005f;
                camPitch = glm::clamp(camPitch, -1.5f, 1.5f);
            }
            lastX = mx; lastY = my;
        }

        ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame(); ImGui::NewFrame();
        int w, h; glfwGetFramebufferSize(window, &w, &h);
        glViewport(0, 0, w, h); glClearColor(0.005f, 0.005f, 0.015f, 1.0f); glClear(GL_COLOR_BUFFER_BIT);

        double drawScale = 2.0e-9;
        glm::dvec3 cameraPos, targetPos;
        if (freeCamera) {
            cameraPos = freeCamPos;
            targetPos = freeCamPos + (glm::dvec3)glm::vec3(sin(camYaw) * cos(camPitch), -sin(camPitch), -cos(camYaw) * cos(camPitch));
        }
        else {
            glm::dvec3 focusWorldPos = b[focusID].pos;
            targetPos = focusWorldPos * drawScale;
            cameraPos = targetPos + (glm::dvec3)glm::vec3(camDist * cos(camPitch) * sin(camYaw), camDist * sin(camPitch), camDist * cos(camPitch) * cos(camYaw));
        }
        glm::mat4 viewProj = glm::perspective(glm::radians(45.0f), (float)w / h, 0.1f, 500000.0f) * glm::lookAt((glm::vec3)cameraPos, (glm::vec3)targetPos, glm::vec3(0, 1, 0));

        ImDrawList* drawList = ImGui::GetBackgroundDrawList();
        for (int i = 0; i < (int)b.size(); i++) {
            glm::dvec3 rPos = b[i].pos * drawScale;
            ImU32 col = IM_COL32((int)(b[i].color.r * 255), (int)(b[i].color.g * 255), (int)(b[i].color.b * 255), 255);

            if (b[i].showTrail && currentMode == MODE_LIVE) {
                b[i].trail.push_back(rPos);
                if (b[i].trail.size() > 2000) b[i].trail.pop_front();
                for (size_t j = 1; j < b[i].trail.size(); j++) {
                    ImVec2 p1, p2;
                    if (ProjectToScreen(b[i].trail[j - 1], viewProj, w, h, p1) && ProjectToScreen(b[i].trail[j], viewProj, w, h, p2))
                        drawList->AddLine(p1, p2, col & 0x00FFFFFF | 0x60000000, 1.5f);
                }
            }
            ImVec2 sp;
            if (ProjectToScreen(rPos, viewProj, w, h, sp)) {
                drawList->AddCircleFilled(sp, (i == 0 ? 10.0f : 4.0f), col);
                drawList->AddText(ImVec2(sp.x + 10, sp.y - 10), 0xFFFFFFFF, b[i].name.c_str());
            }
        }

        // --- 窗口 1: 控制中心 ---
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::Begin("Control Center", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::TextColored(ImVec4(0, 1, 0, 1), "SYSTEM TIME");
        ImGui::Text("%s", GetSimulatedDateTime(totalSimSeconds).c_str());
        ImGui::Separator();
        ImGui::Checkbox("FREE CAMERA MODE", &freeCamera);
        ImGui::SliderFloat("Simulation Speed", &timeScale, 0.0f, 50.0f, "%.1fx");
        if (ImGui::Button("Pause / Resume", ImVec2(-1, 30))) currentMode = (currentMode == MODE_LIVE ? MODE_PAUSED : MODE_LIVE);
        if (ImGui::Button("Reset Universe", ImVec2(-1, 30))) { totalSimSeconds = 0; for (auto& body : b) body.trail.clear(); }
        ImGui::End();

        // --- 窗口 2: 天体监视 ---
        ImGui::SetNextWindowPos(ImVec2((float)w - 360.0f, 10.0f), ImGuiCond_FirstUseEver);
        ImGui::Begin("Telemetry Monitor", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        if (ImGui::BeginTabBar("Bodies")) {
            for (int i = 0; i < (int)b.size(); i++) {
                if (ImGui::BeginTabItem(b[i].name.c_str())) {
                    ImGui::Checkbox("Enable Orbit Trail", &b[i].showTrail);
                    if (ImGui::Button("Focus Camera")) { focusID = i; freeCamera = false; }
                    ImGui::Separator();
                    ImGui::Text("Mass: %.2e kg", b[i].mass);
                    ImGui::Text("Velocity: %.2f km/s", glm::length(b[i].vel) / 1000.0);
                    // 修正此处报错：使用 b[i] 替代之前错误的 b[id]
                    ImGui::Text("Acc: %.4e m/s2", glm::length(b[i].lastAcc));
                    ImGui::Text("Pos(AU): %.3f, %.3f, %.3f", b[i].pos.x / AU, b[i].pos.y / AU, b[i].pos.z / AU);
                    ImGui::EndTabItem();
                }
            }
            ImGui::EndTabBar();
        }
        ImGui::End();

        ImGui::Render(); ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }
    quit = true;
    glfwTerminate();
}

int main() {
    const double AU_val = 1.496e11;
    const double earth_v = 29780.0;

    b.emplace_back("Sun", 1.989e30, glm::dvec3(0), glm::dvec3(0), glm::vec3(1, 0.9, 0.1));
    b.emplace_back("Mercury", 3.3e23, glm::dvec3(0.387 * AU_val, 0, 0), glm::dvec3(0, 0, 47400), glm::vec3(0.7, 0.7, 0.7));
    b.emplace_back("Venus", 4.87e24, glm::dvec3(0.723 * AU_val, 0, 0), glm::dvec3(0, 0, 35000), glm::vec3(1, 0.8, 0.4));
    b.emplace_back("Earth", 5.97e24, glm::dvec3(AU_val, 0, 0), glm::dvec3(0, 0, earth_v), glm::vec3(0.2, 0.6, 1));
    b.emplace_back("Mars", 6.39e23, glm::dvec3(1.52 * AU_val, 0, 0), glm::dvec3(0, 0, 24100), glm::vec3(1, 0.4, 0.3));
    b.emplace_back("Jupiter", 1.9e27, glm::dvec3(5.2 * AU_val, 0, 0), glm::dvec3(0, 0, 13070), glm::vec3(0.8, 0.7, 0.5));
    b.emplace_back("Halley", 2.2e14, glm::dvec3(35.08 * AU_val, 0, 0), glm::dvec3(0, 0, -910), glm::vec3(1, 1, 1));

    shared_pos.resize(b.size());
    std::barrier sync_point((int)b.size());
    vector<thread> workers;
    for (int i = 0; i < (int)b.size(); i++) workers.emplace_back(physics_worker, i, ref(sync_point));
    render_loop();
    for (auto& t : workers) t.join();
    return 0;
}