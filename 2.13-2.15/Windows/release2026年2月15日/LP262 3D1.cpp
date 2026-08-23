#define _USE_MATH_DEFINES
#include <iostream>
#include <vector>
#include <thread>
#include <barrier>
#include <cmath>
#include <atomic>
#include <deque>
#include <string>
#include <chrono>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

using namespace std;

// --- 物理常数 ---
const double G = 6.67430e-11;
const double AU = 1.496e11;
const double dt = 360.0;
const int physics_steps = 1200; // 稍微增加步长，让行星动得更自然

struct Body {
    string name;
    double mass;
    glm::dvec3 pos, vel;
    glm::vec3 color; // 0.0f - 1.0f
    deque<glm::dvec3> trail;
    bool showTrail = true;
};

vector<Body> b;
vector<glm::dvec3> shared_pos;
atomic<bool> quit{ false };
bool showUI = true;
bool showAxes = true;
int focusID = 0;

float camYaw = 0.5f, camPitch = 0.4f, camDist = 150.0f;
double lastX, lastY;

// 滚轮缩放
void scroll_callback(GLFWwindow* window, double x, double y) {
    if (y > 0) camDist *= 0.85f; else camDist *= 1.15f;
    camDist = glm::clamp(camDist, 0.05f, 15000.0f);
}

// 物理计算线程 (N-Body 模拟)
void physics_worker(int id, int n, std::barrier<>& sync) {
    while (!quit) {
        for (int s = 0; s < physics_steps; ++s) {
            b[id].pos += b[id].vel * dt;
            shared_pos[id] = b[id].pos;
            sync.arrive_and_wait();

            glm::dvec3 acc{ 0,0,0 };
            for (int j = 0; j < n; ++j) {
                if (j == id) continue;
                glm::dvec3 diff = shared_pos[j] - shared_pos[id];
                double r2 = glm::dot(diff, diff) + 1e8; // 防止软碰撞
                acc += (G * b[j].mass / (r2 * sqrt(r2))) * diff;
            }
            b[id].vel += acc * dt;
            sync.arrive_and_wait();
        }
    }
}

// 投影坐标
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
    GLFWwindow* window = glfwCreateWindow(1440, 900, "Solar Lab - V4 Rendering Fix", NULL, NULL);
    if (!window) return;
    glfwMakeContextCurrent(window);
    glfwSetScrollCallback(window, scroll_callback);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    vector<glm::vec3> stars;
    for (int i = 0; i < 1000; i++)
        stars.push_back(glm::normalize(glm::vec3(rand() % 200 - 100, rand() % 200 - 100, rand() % 200 - 100)) * 9000.0f);

    while (!glfwWindowShouldClose(window) && !quit) {
        glfwPollEvents();

        static bool hPressed = false, aPressed = false;
        if (glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS && !hPressed) { showUI = !showUI; hPressed = true; }
        if (glfwGetKey(window, GLFW_KEY_H) == GLFW_RELEASE) hPressed = false;
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS && !aPressed) { showAxes = !showAxes; aPressed = true; }
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_RELEASE) aPressed = false;

        if (!ImGui::GetIO().WantCaptureMouse) {
            double mx, my; glfwGetCursorPos(window, &mx, &my);
            if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
                camYaw += (float)(mx - lastX) * 0.005f;
                camPitch += (float)(my - lastY) * 0.005f;
            }
            lastX = mx; lastY = my;
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        int w, h; glfwGetFramebufferSize(window, &w, &h);
        glViewport(0, 0, w, h);
        glClearColor(0.005f, 0.005f, 0.012f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // --- 渲染管道设置 ---
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        double drawScale = 2.0e-9;
        glm::dvec3 focusPos = b[focusID].pos * drawScale;
        glm::mat4 proj = glm::perspective(glm::radians(45.0f), (float)w / h, 0.1f, 25000.0f);
        glm::vec3 relCam = glm::vec3(camDist * cos(camPitch) * sin(camYaw), camDist * sin(camPitch), camDist * cos(camPitch) * cos(camYaw));
        glm::mat4 view = glm::lookAt(glm::vec3(focusPos) + relCam, glm::vec3(focusPos), glm::vec3(0, 1, 0));
        glm::mat4 viewProj = proj * view;

        ImDrawList* drawList = ImGui::GetBackgroundDrawList();

        // 1. 星空
        for (auto& s : stars) {
            ImVec2 sp;
            if (ProjectToScreen(glm::dvec3(s), viewProj, w, h, sp))
                drawList->AddCircleFilled(sp, 0.7f, IM_COL32(255, 255, 255, 140));
        }

        // 2. 坐标轴 (强制色彩亮度)
        if (showAxes) {
            float axisLen = 150.0f;
            ImVec2 center, px, py, pz;
            if (ProjectToScreen(glm::dvec3(0, 0, 0), viewProj, w, h, center)) {
                if (ProjectToScreen(glm::dvec3(axisLen / drawScale, 0, 0), viewProj, w, h, px)) drawList->AddLine(center, px, IM_COL32(255, 50, 50, 255), 2.5f);
                if (ProjectToScreen(glm::dvec3(0, axisLen / drawScale, 0), viewProj, w, h, py)) drawList->AddLine(center, py, IM_COL32(50, 255, 50, 255), 2.5f);
                if (ProjectToScreen(glm::dvec3(0, 0, axisLen / drawScale), viewProj, w, h, pz)) drawList->AddLine(center, pz, IM_COL32(50, 50, 255, 255), 2.5f);
            }
        }

        // 3. 天体渲染循环
        for (int i = 0; i < (int)b.size(); i++) {
            glm::dvec3 rPos = b[i].pos * drawScale;

            // 颜色整数化 (0-255)
            int r = (int)(b[i].color.x * 255);
            int g = (int)(b[i].color.y * 255);
            int b_val = (int)(b[i].color.z * 255);

            // 绘制轨迹 (手动指定颜色，不经过转换函数)
            if (b[i].showTrail && b[i].trail.size() > 1) {
                for (size_t j = 0; j < b[i].trail.size() - 1; j++) {
                    ImVec2 p1, p2;
                    if (ProjectToScreen(b[i].trail[j], viewProj, w, h, p1) && ProjectToScreen(b[i].trail[j + 1], viewProj, w, h, p2)) {
                        drawList->AddLine(p1, p2, IM_COL32(r, g, b_val, 50), 7.0f); // 辉光层
                        drawList->AddLine(p1, p2, IM_COL32(r, g, b_val, 220), 2.5f); // 核心层
                    }
                }
            }
            if (b[i].trail.size() > 2500) b[i].trail.pop_front();
            b[i].trail.push_back(rPos);

            ImVec2 screenPos;
            if (ProjectToScreen(rPos, viewProj, w, h, screenPos)) {
                float sizeFactor = (i == 0) ? 16.0f : (b[i].mass > 1e26 ? 10.0f : 5.0f);
                // 多层光晕
                drawList->AddCircleFilled(screenPos, sizeFactor * 2.2f, IM_COL32(r, g, b_val, 60));
                drawList->AddCircleFilled(screenPos, sizeFactor, IM_COL32(r, g, b_val, 255));
                drawList->AddCircleFilled(screenPos, sizeFactor * 0.3f, IM_COL32(255, 255, 255, 255)); // 纯白核心

                if (showUI) {
                    drawList->AddText(ImVec2(screenPos.x + sizeFactor + 5, screenPos.y - 10), IM_COL32(255, 255, 255, 230), b[i].name.c_str());
                }
            }
        }

        if (showUI) {
            ImGui::SetNextWindowPos(ImVec2(20, 20), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowBgAlpha(0.8f);
            ImGui::Begin("Solar Control V4", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
            ImGui::TextColored(ImVec4(0, 1, 1, 1), "Controls: H: UI | A: Axes");
            ImGui::Separator();
            for (int i = 0; i < (int)b.size(); i++) {
                ImGui::PushID(i);
                ImGui::ColorButton("##dot", ImVec4(b[i].color.x, b[i].color.y, b[i].color.z, 1.0f), 0, ImVec2(12, 12));
                ImGui::SameLine();
                if (ImGui::Selectable(b[i].name.c_str(), focusID == i, 0, ImVec2(100, 0))) focusID = i;
                ImGui::SameLine();
                ImGui::Checkbox("##tr", &b[i].showTrail);
                ImGui::PopID();
            }
            if (ImGui::Button("Reset All Trails", ImVec2(-1, 25))) { for (auto& body : b) body.trail.clear(); }
            ImGui::End();
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    quit = true;
    ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplGlfw_Shutdown(); ImGui::DestroyContext();
    glfwTerminate();
}

int main() {
    // 强制使用高对比度颜色定义
    b = {
        {"Sun",     1.989e30, {0,0,0}, {0,0,0}, {1.0f, 0.9f, 0.1f}},    // 金黄
        {"Mercury", 3.301e23, {0.387 * AU, 0, 0}, {0, 0, 47400}, {0.7f, 0.7f, 0.8f}}, // 灰
        {"Venus",   4.867e24, {0.723 * AU, 0, 0}, {0, 0, 35000}, {1.0f, 0.6f, 0.2f}}, // 橙
        {"Earth",   5.972e24, {1.0 * AU, 0, 0}, {0, 0, 29780}, {0.1f, 0.5f, 1.0f}}, // 蔚蓝
        {"Moon",    7.347e22, {1.0 * AU + 3.8e8, 0, 0}, {0, 0, 29780 + 1022}, {0.9f, 0.9f, 0.9f}}, // 银白
        {"Mars",    6.39e23,  {1.524 * AU, 0, 0}, {0, 0, 24000}, {1.0f, 0.2f, 0.1f}}, // 火红
        {"Jupiter", 1.898e27, {5.203 * AU, 0, 0}, {0, 0, 13070}, {0.8f, 0.7f, 0.5f}}, // 土褐
        {"Saturn",  5.683e26, {9.537 * AU, 0, 0}, {0, 0, 9690}, {0.9f, 0.8f, 0.4f}},  // 淡黄
        {"Uranus",  8.681e25, {19.191 * AU, 0, 0}, {0, 0, 6810}, {0.5f, 0.9f, 1.0f}}, // 青色
        {"Neptune", 1.024e26, {30.069 * AU, 0, 0}, {0, 0, 5430}, {0.2f, 0.3f, 1.0f}}, // 深蓝
        {"Halley",  2.2e14,   {35.1 * AU, 0, 0}, {0, 0, -912}, {0.7f, 0.8f, 1.0f}}   // 彗白
    };

    int n = b.size();
    shared_pos.resize(n);
    for (int i = 0; i < n; i++) shared_pos[i] = b[i].pos;

    std::barrier sync(n);
    vector<thread> workers;
    for (int i = 0; i < n; i++) workers.emplace_back(physics_worker, i, n, ref(sync));
    render_loop();
    for (auto& t : workers) t.join();
    return 0;
}