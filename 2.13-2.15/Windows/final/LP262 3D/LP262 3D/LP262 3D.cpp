#define _CRT_SECURE_NO_WARNINGS
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
const double C = 299792458.0 * 0.001; // 光速 (m/s) 可通过调整系数，改变光速，方便观察相对论效应的变化（所导致的进动现象），当前设置为真实光速的千分之一以增强视觉效果。
const double C2 = C * C;

struct Body {
    string name;
    double mass;
    glm::dvec3 pos, vel;
    glm::dvec3 lastAcc{ 0,0,0 }; // 新增：用于遥测显示
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
double totalSimSeconds = 0; // 新增：模拟时长累计
SimMode currentMode = MODE_LIVE;
int playbackFrame = 0;
const int max_history = 15000;

vector<Body> b;
vector<glm::dvec3> shared_pos;
vector<glm::dvec3> shared_vel; // 新增：用于多线程间同步速度信息
atomic<bool> quit{ false };
bool showUI = true;
int focusID = 0;

// --- 相机控制 ---
bool freeCamera = false;
float camYaw = 0.5f, camPitch = 0.4f, camDist = 150.0f;
glm::dvec3 freeCamPos = glm::dvec3(0, 100, 300);
float moveSpeed = 1.0f;
double lastX, lastY;

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
            // --- 第一步：更新位置并同步 ---
            b[id].pos += b[id].vel * dt;
            shared_pos[id] = b[id].pos;
            shared_vel[id] = b[id].vel; // 将当前速度存入共享区
            sync.arrive_and_wait(); // 所有人更新完毕后再进入下一步

            // --- 第二步：计算相对论加速度 ---
            glm::dvec3 acc{ 0,0,0 };
            glm::dvec3 v_i = b[id].vel;
            double v_i2 = glm::dot(v_i, v_i);

            for (int j = 0; j < (int)b.size(); ++j) {
                if (j == id) continue;

                glm::dvec3 r_vec = shared_pos[j] - shared_pos[id];
                double r = glm::length(r_vec);
                if (r < 1e5) r = 1e5; // 防止极近距离导致的数值爆炸
                double r3 = r * r * r;

                glm::dvec3 v_j = shared_vel[j]; // 现在可以安全访问 shared_vel 了
                double v_j2 = glm::dot(v_j, v_j);

                // 牛顿项
                glm::dvec3 Newton = (G * b[j].mass / r3) * r_vec;

                // 1PN 相对论修正
                double term1 = -4.0 * G * b[j].mass / (r * C2);
                double term2 = -v_i2 / C2;
                double term3 = 4.0 * glm::dot(v_i, v_j) / C2;
                double term4 = -2.0 * v_j2 / C2;
                double term5 = 0.75 * pow(glm::dot(r_vec, v_j) / r, 2) / C2;

                double factor = (1.0 + term1 + term2 + term3 + term4 + term5);
                glm::dvec3 v_term = (G * b[j].mass / (C2 * r3)) * glm::dot(r_vec, (3.0 * v_i - 4.0 * v_j)) * (v_i - v_j);

                acc += Newton * factor + v_term;
            }

            // --- 第三步：更新速度并同步 ---
            b[id].lastAcc = acc;
            b[id].vel += acc * dt;
            sync.arrive_and_wait(); // 等待所有人更新完速度，进入下一物理帧
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
    GLFWwindow* window = glfwCreateWindow(1440, 900, "Solar Lab Pro - All Features", NULL, NULL);
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
            // 累计时长：每物理帧 dt * 500步
            totalSimSeconds += (base_dt * (double)timeScale * 500.0) / 60.0;
            for (auto& body : b) {
                body.history.push_back(body.pos);
                if (body.history.size() > max_history) body.history.erase(body.history.begin());
            }
        }

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
        glm::mat4 viewProj = glm::perspective(glm::radians(45.0f), (float)w / h, 0.1f, 150000.0f) * glm::lookAt((glm::vec3)cameraPos, (glm::vec3)targetPos, glm::vec3(0, 1, 0));

        ImDrawList* drawList = ImGui::GetBackgroundDrawList();
        for (int i = 0; i < (int)b.size(); i++) {
            glm::dvec3 visualPos = (currentMode == MODE_PLAYBACK && !b[i].history.empty()) ? b[i].history[playbackFrame] : b[i].pos;
            glm::dvec3 rPos = visualPos * drawScale;
            int r = (int)(b[i].color.x * 255), g = (int)(b[i].color.y * 255), bv = (int)(b[i].color.z * 255);

            // --- 修改后的轨迹绘制逻辑 ---
            if (b[i].showTrail) {
                if (currentMode == MODE_PLAYBACK && !b[i].history.empty()) {
                    // 回放模式：绘制从开始到当前回放帧的完整路径
                    for (int j = 1; j <= playbackFrame; j++) {
                        ImVec2 p1, p2;
                        if (ProjectToScreen(b[i].history[j - 1] * drawScale, viewProj, w, h, p1) &&
                            ProjectToScreen(b[i].history[j] * drawScale, viewProj, w, h, p2))
                            drawList->AddLine(p1, p2, IM_COL32(r, g, bv, 80), 1.5f);
                    }
                }
                // 【修改点】：将 MODE_PAUSED 也加入判断，确保暂停时显示 trail
                else if (currentMode == MODE_LIVE || currentMode == MODE_PAUSED) {
                    if (currentMode == MODE_LIVE) {
                        b[i].trail.push_back(rPos);
                        if (b[i].trail.size() > 2500) b[i].trail.pop_front();
                    }

                    // 只要 trail 里有数据，不论是运行还是暂停，都画出来
                    for (size_t j = 1; j < b[i].trail.size(); j++) {
                        ImVec2 p1, p2;
                        if (ProjectToScreen(b[i].trail[j - 1], viewProj, w, h, p1) &&
                            ProjectToScreen(b[i].trail[j], viewProj, w, h, p2))
                            drawList->AddLine(p1, p2, IM_COL32(r, g, bv, 120), 2.0f);
                    }
                }
            }

            ImVec2 sp;
            if (ProjectToScreen(rPos, viewProj, w, h, sp)) {
                float size = (i == 0) ? 12.0f : (b[i].mass > 1e26 ? 6.0f : 3.5f);
                drawList->AddCircleFilled(sp, size, IM_COL32(r, g, bv, 255));
                if (showUI) drawList->AddText(ImVec2(sp.x + 8, sp.y - 8), IM_COL32(255, 255, 255, 180), b[i].name.c_str());
            }
        }

        // --- 窗口 1: Cosmos Lab Pro (控制窗口) ---
        if (showUI) {
            ImGui::Begin("Cosmos Lab Pro", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
            ImGui::Text("Controls: Left-Drag: Rotate | Right-Drag: Move (Free) | Scroll: Zoom");
            ImGui::Checkbox("FREE CAMERA MODE", &freeCamera);
            if (freeCamera) ImGui::SliderFloat("Move Speed", &moveSpeed, 0.1f, 10.0f);

            ImGui::Separator();
            if (ImGui::Button(currentMode == MODE_LIVE ? "PAUSE (Space)" : "RESUME (Space)"))
                currentMode = (currentMode == MODE_LIVE) ? MODE_PAUSED : MODE_LIVE;
            ImGui::SameLine();
            if (ImGui::Button("CLEAR TRAILS")) { for (auto& body : b) body.trail.clear(); }

            ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(0.0f, 0.6f, 0.6f));
            if (ImGui::Button("RESET ALL HISTORY", ImVec2(-1, 0))) {
                totalSimSeconds = 0;
                for (auto& body : b) { body.history.clear(); body.trail.clear(); }
                playbackFrame = 0; currentMode = MODE_LIVE;
            }
            ImGui::PopStyleColor();

            ImGui::SliderFloat("Sim Speed", &timeScale, 0.0f, 20.0f, "%.1fx");
            ImGui::Separator();

            int maxFrame = (b[0].history.empty()) ? 0 : (int)b[0].history.size() - 1;
            if (ImGui::SliderInt("Replay Seek", &playbackFrame, 0, maxFrame)) {
                if (!b[0].history.empty()) currentMode = MODE_PLAYBACK;
            }
            if (currentMode == MODE_PLAYBACK && ImGui::Button("Return to Live")) currentMode = MODE_LIVE;

            ImGui::Separator();
            for (int i = 0; i < (int)b.size(); i++) {
                ImGui::PushID(i);
                ImGui::Checkbox("##t", &b[i].showTrail); ImGui::SameLine();
                if (ImGui::Selectable(b[i].name.c_str(), focusID == i && !freeCamera)) {
                    focusID = i; freeCamera = false;
                }
                ImGui::PopID();
            }
            ImGui::End();
        }

        // --- 窗口 2: Celestial Telemetry (新增遥测窗口) ---
        if (showUI) {
            ImGui::Begin("Celestial Telemetry", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

            // 计算模拟时间
            double years = totalSimSeconds / (365.25 * 24 * 3600);
            double days = fmod(totalSimSeconds, (365.25 * 24 * 3600)) / (24 * 3600);
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 1.0f, 1.0f), "Simulation Time:");
            ImGui::Text("%d Years, %.2f Days", (int)years, days);

            ImGui::Separator();
            if (ImGui::BeginTabBar("PlanetStats")) {
                for (int i = 0; i < (int)b.size(); i++) {
                    if (ImGui::BeginTabItem(b[i].name.c_str())) {
                        ImGui::Text("Mass: %.3e kg", b[i].mass);
                        ImGui::Text("Pos(AU): %.3f, %.3f, %.3f", b[i].pos.x / AU, b[i].pos.y / AU, b[i].pos.z / AU);
                        ImGui::Text("Vel(km/s): %.2f", glm::length(b[i].vel) / 1000.0);
                        ImGui::Text("Acc(m/s^2): %.2e", glm::length(b[i].lastAcc));
                        ImGui::EndTabItem();
                    }
                }
                ImGui::EndTabBar();
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
    const double AU = 1.496e11;
    const double earth_v = 29780.0;
    const double moon_dist = 384400000.0;
    const double moon_v_rel = 1022.0;

    double l4_angle = M_PI / 3.0;
    glm::dvec3 l4_pos(AU * cos(l4_angle), 0, -AU * sin(l4_angle));
    glm::dvec3 l4_vel(earth_v * sin(l4_angle), 0, earth_v * cos(l4_angle));
    double l5_angle = -M_PI / 3.0;
    glm::dvec3 l5_pos(AU * cos(l5_angle), 0, -AU * sin(l5_angle));
    glm::dvec3 l5_vel(earth_v * sin(l5_angle), 0, earth_v * cos(l5_angle));

    b.clear();
    b.emplace_back("Sun", 1.989e30, glm::dvec3(0), glm::dvec3(0), glm::vec3(1.0f, 0.9f, 0.1f));
    b.emplace_back("Mercury", 3.301e23, glm::dvec3(0.387 * AU, 0, 0), glm::dvec3(0, 0, 47400), glm::vec3(0.7f, 0.7f, 0.7f));
    b.emplace_back("Venus", 4.867e24, glm::dvec3(0.723 * AU, 0, 0), glm::dvec3(0, 0, 35000), glm::vec3(1.0f, 0.8f, 0.4f));
    b.emplace_back("Earth", 5.972e24, glm::dvec3(AU, 0, 0), glm::dvec3(0, 0, earth_v), glm::vec3(0.2f, 0.6f, 1.0f));
    b.emplace_back("Moon", 7.347e22, glm::dvec3(AU + moon_dist, 0, 0), glm::dvec3(0, 0, earth_v + moon_v_rel), glm::vec3(0.6f, 0.6f, 0.6f));
    b.emplace_back("Earth L4", 1e4, l4_pos, l4_vel, glm::vec3(0.0f, 1.0f, 0.5f), true);
    b.emplace_back("Earth L5", 1e4, l5_pos, l5_vel, glm::vec3(0.0f, 1.0f, 0.5f), true);
    b.emplace_back("Mars", 6.39e23, glm::dvec3(1.524 * AU, 0, 0), glm::dvec3(0, 0, 24100), glm::vec3(1.0f, 0.4f, 0.3f));
    b.emplace_back("Jupiter", 1.898e27, glm::dvec3(5.203 * AU, 0, 0), glm::dvec3(0, 0, 13070), glm::vec3(0.8f, 0.7f, 0.5f));
    b.emplace_back("Halley", 2.2e14, glm::dvec3(35.08 * AU, 0, 0), glm::dvec3(0, 0, -910), glm::vec3(1.0f, 1.0f, 1.0f));
    b.emplace_back("Voyager", 1e4, glm::dvec3(1.1 * AU, 0, 0), glm::dvec3(10000, 0, 35000), glm::vec3(1.0f, 1.0f, 1.0f), true);

    shared_pos.resize(b.size());
    shared_vel.resize(b.size()); // 新增：确保大小与天体数量一致
    std::barrier sync((int)b.size());
    vector<thread> workers;
    for (int i = 0; i < (int)b.size(); i++) workers.emplace_back(physics_worker, i, ref(sync));
    render_loop();
    for (auto& t : workers) t.join();
    return 0;
}