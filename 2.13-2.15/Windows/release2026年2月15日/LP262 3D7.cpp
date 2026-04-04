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
    glm::dvec3 lastAcc{ 0,0,0 }; // 实时加速度
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
double totalSimSeconds = 0; // 累计模拟时间
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
double lastX, lastY;

// 缩放回调
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

// 物理计算线程
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
            b[id].lastAcc = acc; // 存储实时加速度用于UI显示
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
    GLFWwindow* window = glfwCreateWindow(1440, 900, "Solar Lab Pro - Full Telemetry", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSetScrollCallback(window, scroll_callback);
    IMGUI_CHECKVERSION(); ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    while (!glfwWindowShouldClose(window) && !quit) {
        glfwPollEvents();
        static bool spacePressed = false;
        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS && !spacePressed) {
            currentMode = (currentMode == MODE_LIVE) ? MODE_PAUSED : MODE_LIVE;
            spacePressed = true;
        }
        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_RELEASE) spacePressed = false;

        // 鼠标交互：左键旋转，右键平移(自由模式)
        if (!ImGui::GetIO().WantCaptureMouse) {
            double mx, my; glfwGetCursorPos(window, &mx, &my);
            double dx = mx - lastX; double dy = my - lastY;
            if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
                camYaw += (float)dx * 0.005f; camPitch += (float)dy * 0.005f;
                camPitch = glm::clamp(camPitch, -1.5f, 1.5f);
            }
            if (freeCamera && glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
                glm::vec3 forward = glm::vec3(sin(camYaw), 0, -cos(camYaw));
                glm::vec3 right = glm::normalize(glm::cross(forward, glm::vec3(0, 1, 0)));
                float s = moveSpeed * camDist * 0.001f;
                freeCamPos -= (glm::dvec3)right * (dx * s);
                freeCamPos += (glm::dvec3)forward * (dy * s);
            }
            lastX = mx; lastY = my;
        }

        // 键盘备用控制
        if (freeCamera) {
            glm::vec3 forward = glm::vec3(sin(camYaw) * cos(camPitch), -sin(camPitch), -cos(camYaw) * cos(camPitch));
            glm::vec3 right = glm::normalize(glm::cross(forward, glm::vec3(0, 1, 0)));
            float s = moveSpeed * camDist * 0.01f;
            if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) freeCamPos += (glm::dvec3)forward * (double)s;
            if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) freeCamPos -= (glm::dvec3)forward * (double)s;
            if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) freeCamPos -= (glm::dvec3)right * (double)s;
            if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) freeCamPos += (glm::dvec3)right * (double)s;
            if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) freeCamPos.y += (double)s;
            if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) freeCamPos.y -= (double)s;
        }

        ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame(); ImGui::NewFrame();
        int w, h; glfwGetFramebufferSize(window, &w, &h);
        glViewport(0, 0, w, h); glClearColor(0.005f, 0.005f, 0.01f, 1.0f); glClear(GL_COLOR_BUFFER_BIT);

        if (currentMode == MODE_LIVE) {
            totalSimSeconds += (base_dt * timeScale * 500) / 60.0; // 模拟时间推移
            for (auto& body : b) {
                body.history.push_back(body.pos);
                if (body.history.size() > max_history) body.history.erase(body.history.begin());
            }
        }

        // 相机矩阵
        double drawScale = 2.0e-9;
        glm::dvec3 cameraPos, targetPos;
        if (freeCamera) {
            cameraPos = freeCamPos;
            targetPos = freeCamPos + (glm::dvec3)glm::vec3(sin(camYaw) * cos(camPitch), -sin(camPitch), -cos(camYaw) * cos(camPitch));
        }
        else {
            glm::dvec3 focusWorldPos = (currentMode == MODE_PLAYBACK && !b[focusID].history.empty()) ? b[focusID].history[playbackFrame] : b[focusID].pos;
            targetPos = focusWorldPos * drawScale;
            cameraPos = targetPos + (glm::dvec3)glm::vec3(camDist * cos(camPitch) * sin(camYaw), camDist * sin(camPitch), camDist * cos(camPitch) * cos(camYaw));
            freeCamPos = cameraPos;
        }
        glm::mat4 viewProj = glm::perspective(glm::radians(45.0f), (float)w / h, 0.1f, 200000.0f) * glm::lookAt((glm::vec3)cameraPos, (glm::vec3)targetPos, glm::vec3(0, 1, 0));

        // 渲染星体
        ImDrawList* drawList = ImGui::GetBackgroundDrawList();
        for (int i = 0; i < (int)b.size(); i++) {
            glm::dvec3 visualPos = (currentMode == MODE_PLAYBACK && !b[i].history.empty()) ? b[i].history[playbackFrame] : b[i].pos;
            glm::dvec3 rPos = visualPos * drawScale;
            int r = (int)(b[i].color.x * 255), g = (int)(b[i].color.y * 255), bv = (int)(b[i].color.z * 255);

            if (b[i].showTrail) {
                if (currentMode == MODE_PLAYBACK && !b[i].history.empty()) {
                    for (int j = 1; j <= playbackFrame; j++) {
                        ImVec2 p1, p2;
                        if (ProjectToScreen(b[i].history[j - 1] * drawScale, viewProj, w, h, p1) && ProjectToScreen(b[i].history[j] * drawScale, viewProj, w, h, p2))
                            drawList->AddLine(p1, p2, IM_COL32(r, g, bv, 80), 1.5f);
                    }
                }
                else if (currentMode == MODE_LIVE) {
                    b[i].trail.push_back(rPos);
                    if (b[i].trail.size() > 2500) b[i].trail.pop_front();
                    for (size_t j = 1; j < b[i].trail.size(); j++) {
                        ImVec2 p1, p2;
                        if (ProjectToScreen(b[i].trail[j - 1], viewProj, w, h, p1) && ProjectToScreen(b[i].trail[j], viewProj, w, h, p2))
                            drawList->AddLine(p1, p2, IM_COL32(r, g, bv, 120), 2.0f);
                    }
                }
            }
            ImVec2 sp;
            if (ProjectToScreen(rPos, viewProj, w, h, sp)) {
                float size = (i == 0) ? 12.0f : (b[i].mass > 1e26 ? 6.0f : 3.5f);
                drawList->AddCircleFilled(sp, size, IM_COL32(r, g, bv, 255));
                drawList->AddText(ImVec2(sp.x + 8, sp.y - 8), IM_COL32(255, 255, 255, 180), b[i].name.c_str());
            }
        }

        // UI 遥测面板
        if (showUI) {
            ImGui::Begin("Cosmos Lab Pro - Telemetry", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

            // 真实映射时间
            double years = totalSimSeconds / (365.25 * 24 * 3600);
            double days = fmod(totalSimSeconds, (365.25 * 24 * 3600)) / (24 * 3600);
            ImGui::TextColored(ImVec4(0, 1, 1, 1), "Simulated Time:");
            ImGui::Text("%.0f Years, %.2f Days", years, days);
            ImGui::Separator();

            ImGui::Checkbox("FREE CAMERA", &freeCamera);
            ImGui::SliderFloat("Sim Speed", &timeScale, 0.0f, 20.0f, "%.1fx");

            // 天体数据卡片
            if (ImGui::CollapsingHeader("Celestial Telemetry", ImGuiTreeNodeFlags_DefaultOpen)) {
                if (ImGui::BeginTabBar("Tabs")) {
                    for (int i = 0; i < (int)b.size(); i++) {
                        if (ImGui::BeginTabItem(b[i].name.c_str())) {
                            ImGui::Text("Mass: %.3e kg", b[i].mass);
                            ImGui::Text("Pos(AU): %.3f, %.3f, %.3f", b[i].pos.x / AU, b[i].pos.y / AU, b[i].pos.z / AU);
                            ImGui::Text("Vel(km/s): %.2f", glm::length(b[i].vel) / 1000.0);
                            ImGui::Text("Acc(m/s2): %.2e", glm::length(b[i].lastAcc));
                            ImGui::Separator();
                            if (ImGui::Button("Focus This")) { focusID = i; freeCamera = false; }
                            ImGui::EndTabItem();
                        }
                    }
                    ImGui::EndTabBar();
                }
            }

            if (ImGui::Button("Reset All")) { totalSimSeconds = 0; for (auto& body : b) { body.history.clear(); body.trail.clear(); } }
            ImGui::End();
        }

        ImGui::Render(); ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }
    quit = true;
    glfwTerminate();
}

int main() {
    const double AU = 1.496e11;
    const double earth_v = 29780.0;
    const double moon_dist = 384400000.0;
    const double moon_v_rel = 1022.0;

    // L4/L5 计算
    double l4_a = M_PI / 3.0;
    glm::dvec3 l4_pos(AU * cos(l4_a), 0, -AU * sin(l4_a));
    glm::dvec3 l4_vel(earth_v * sin(l4_a), 0, earth_v * cos(l4_a));
    double l5_a = -M_PI / 3.0;
    glm::dvec3 l5_pos(AU * cos(l5_a), 0, -AU * sin(l5_a));
    glm::dvec3 l5_vel(earth_v * sin(l5_a), 0, earth_v * cos(l5_a));

    b.emplace_back("Sun", 1.989e30, glm::dvec3(0), glm::dvec3(0), glm::vec3(1, 0.9, 0.1));
    b.emplace_back("Mercury", 3.3e23, glm::dvec3(0.387 * AU, 0, 0), glm::dvec3(0, 0, 47400), glm::vec3(0.7, 0.7, 0.7));
    b.emplace_back("Venus", 4.87e24, glm::dvec3(0.723 * AU, 0, 0), glm::dvec3(0, 0, 35000), glm::vec3(1, 0.8, 0.4));
    b.emplace_back("Earth", 5.97e24, glm::dvec3(AU, 0, 0), glm::dvec3(0, 0, earth_v), glm::vec3(0.2, 0.6, 1));
    b.emplace_back("Moon", 7.35e22, glm::dvec3(AU + moon_dist, 0, 0), glm::dvec3(0, 0, earth_v + moon_v_rel), glm::vec3(0.6, 0.6, 0.6));
    b.emplace_back("Earth L4", 1e4, l4_pos, l4_vel, glm::vec3(0, 1, 0.5), true);
    b.emplace_back("Earth L5", 1e4, l5_pos, l5_vel, glm::vec3(0, 1, 0.5), true);
    b.emplace_back("Mars", 6.39e23, glm::dvec3(1.52 * AU, 0, 0), glm::dvec3(0, 0, 24100), glm::vec3(1, 0.4, 0.3));
    b.emplace_back("Jupiter", 1.9e27, glm::dvec3(5.2 * AU, 0, 0), glm::dvec3(0, 0, 13070), glm::vec3(0.8, 0.7, 0.5));
    b.emplace_back("Halley", 2.2e14, glm::dvec3(35.08 * AU, 0, 0), glm::dvec3(0, 0, -910), glm::vec3(1, 1, 1)); // 逆行
    b.emplace_back("Voyager", 1e4, glm::dvec3(1.1 * AU, 0, 0), glm::dvec3(10000, 0, 35000), glm::vec3(1, 1, 1), true);

    shared_pos.resize(b.size());
    std::barrier sync((int)b.size());
    vector<thread> workers;
    for (int i = 0; i < (int)b.size(); i++) workers.emplace_back(physics_worker, i, ref(sync));
    render_loop();
    for (auto& t : workers) t.join();
    return 0;
}