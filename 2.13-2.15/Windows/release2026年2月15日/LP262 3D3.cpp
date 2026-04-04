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

enum SimMode { MODE_LIVE, MODE_PAUSED, MODE_PLAYBACK };

struct Body {
    string name;
    double mass;
    glm::dvec3 pos, vel;
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
SimMode currentMode = MODE_LIVE;
int playbackFrame = 0;
const int max_history = 15000;

vector<Body> b;
vector<glm::dvec3> shared_pos;
atomic<bool> quit{ false };
bool showUI = true;
int focusID = 0;

float camYaw = 0.5f, camPitch = 0.4f, camDist = 150.0f;
double lastX, lastY;

void scroll_callback(GLFWwindow* window, double x, double y) {
    if (y > 0) camDist *= 0.85f; else camDist *= 1.15f;
    camDist = glm::clamp(camDist, 0.05f, 60000.0f);
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
    GLFWwindow* window = glfwCreateWindow(1440, 900, "Solar Lab Pro - Replay Trails", NULL, NULL);
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

        ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame(); ImGui::NewFrame();
        int w, h; glfwGetFramebufferSize(window, &w, &h);
        glViewport(0, 0, w, h); glClearColor(0.005f, 0.005f, 0.01f, 1.0f); glClear(GL_COLOR_BUFFER_BIT);

        if (currentMode == MODE_LIVE) {
            for (auto& body : b) {
                body.history.push_back(body.pos);
                if (body.history.size() > max_history) body.history.erase(body.history.begin());
            }
        }

        double drawScale = 2.0e-9;
        glm::dvec3 curPos = (currentMode == MODE_PLAYBACK && !b[focusID].history.empty()) ? b[focusID].history[playbackFrame] : b[focusID].pos;
        glm::mat4 viewProj = glm::perspective(glm::radians(45.0f), (float)w / h, 0.1f, 60000.0f) * glm::lookAt(glm::vec3(curPos * drawScale) + glm::vec3(camDist * cos(camPitch) * sin(camYaw), camDist * sin(camPitch), camDist * cos(camPitch) * cos(camYaw)), glm::vec3(curPos * drawScale), glm::vec3(0, 1, 0));

        ImDrawList* drawList = ImGui::GetBackgroundDrawList();

        for (int i = 0; i < (int)b.size(); i++) {
            glm::dvec3 visualPos = (currentMode == MODE_PLAYBACK && !b[i].history.empty()) ? b[i].history[playbackFrame] : b[i].pos;
            glm::dvec3 rPos = visualPos * drawScale;
            int r = (int)(b[i].color.x * 255), g = (int)(b[i].color.y * 255), bv = (int)(b[i].color.z * 255);

            // --- 轨迹渲染逻辑 ---
            if (b[i].showTrail) {
                if (currentMode == MODE_PLAYBACK && !b[i].history.empty()) {
                    // 回放模式：绘制到当前回放帧为止的所有历史记录
                    for (int j = 1; j <= playbackFrame; j++) {
                        ImVec2 p1, p2;
                        if (ProjectToScreen(b[i].history[j - 1] * drawScale, viewProj, w, h, p1) && ProjectToScreen(b[i].history[j] * drawScale, viewProj, w, h, p2))
                            drawList->AddLine(p1, p2, IM_COL32(r, g, bv, 80), 1.5f);
                    }
                }
                else if (currentMode == MODE_LIVE) {
                    // 实时模式：更新并绘制 trail 队列
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
                if (showUI) drawList->AddText(ImVec2(sp.x + 8, sp.y - 8), IM_COL32(255, 255, 255, 180), b[i].name.c_str());
            }
        }

        if (showUI) {
            ImGui::Begin("Cosmos Lab Pro", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

            // 状态控制
            if (ImGui::Button(currentMode == MODE_LIVE ? "PAUSE (Space)" : "RESUME (Space)"))
                currentMode = (currentMode == MODE_LIVE) ? MODE_PAUSED : MODE_LIVE;

            ImGui::SameLine();
            if (ImGui::Button("CLEAR TRAILS")) {
                for (auto& body : b) body.trail.clear();
            }

            // 新增：彻底重置功能
            ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(0.0f, 0.6f, 0.6f));
            if (ImGui::Button("RESET ALL HISTORY", ImVec2(-1, 0))) {
                for (auto& body : b) {
                    body.history.clear();
                    body.trail.clear();
                }
                playbackFrame = 0;
                currentMode = MODE_LIVE;
            }
            ImGui::PopStyleColor();

            ImGui::SliderFloat("Sim Speed", &timeScale, 0.0f, 20.0f, "%.1fx");
            ImGui::Separator();

            // 回放控制
            int maxFrame = (b[0].history.empty()) ? 0 : (int)b[0].history.size() - 1;
            if (ImGui::SliderInt("Replay Seek", &playbackFrame, 0, maxFrame)) {
                if (!b[0].history.empty()) currentMode = MODE_PLAYBACK;
            }

            if (currentMode == MODE_PLAYBACK && ImGui::Button("Return to Live")) {
                currentMode = MODE_LIVE;
            }

            ImGui::Separator();
            // 行星列表...
            for (int i = 0; i < (int)b.size(); i++) {
                ImGui::PushID(i);
                ImGui::Checkbox("##t", &b[i].showTrail); ImGui::SameLine();
                if (ImGui::Selectable(b[i].name.c_str(), focusID == i)) focusID = i;
                ImGui::PopID();
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
    // 数据来源：NASA 行星概况 (校准值)
    b.emplace_back("Sun", 1.989e30, glm::dvec3(0, 0, 0), glm::dvec3(0, 0, 0), glm::vec3(1.0f, 0.9f, 0.1f));
    b.emplace_back("Mercury", 3.301e23, glm::dvec3(0.387 * AU, 0, 0), glm::dvec3(0, 0, 47400), glm::vec3(0.7f, 0.7f, 0.7f));
    b.emplace_back("Venus", 4.867e24, glm::dvec3(0.723 * AU, 0, 0), glm::dvec3(0, 0, 35000), glm::vec3(1.0f, 0.8f, 0.4f));
    b.emplace_back("Earth", 5.972e24, glm::dvec3(1.0 * AU, 0, 0), glm::dvec3(0, 0, 29780), glm::vec3(0.2f, 0.6f, 1.0f));
    b.emplace_back("Moon", 7.347e22, glm::dvec3(1.0 * AU + 384400000, 0, 0), glm::dvec3(0, 0, 29780 + 1022), glm::vec3(0.6f, 0.6f, 0.6f));
    b.emplace_back("Mars", 6.39e23, glm::dvec3(1.524 * AU, 0, 0), glm::dvec3(0, 0, 24000), glm::vec3(1.0f, 0.4f, 0.3f));
    b.emplace_back("Jupiter", 1.898e27, glm::dvec3(5.203 * AU, 0, 0), glm::dvec3(0, 0, 13070), glm::vec3(0.8f, 0.7f, 0.5f));
    b.emplace_back("Saturn", 5.683e26, glm::dvec3(9.537 * AU, 0, 0), glm::dvec3(0, 0, 9690), glm::vec3(0.9f, 0.8f, 0.4f));
    b.emplace_back("Uranus", 8.681e25, glm::dvec3(19.191 * AU, 0, 0), glm::dvec3(0, 0, 6810), glm::vec3(0.5f, 0.9f, 1.0f));
    b.emplace_back("Neptune", 1.024e26, glm::dvec3(30.069 * AU, 0, 0), glm::dvec3(0, 0, 5430), glm::vec3(0.2f, 0.3f, 1.0f));
    b.emplace_back("Pluto", 1.303e22, glm::dvec3(39.482 * AU, 0, 0), glm::dvec3(0, 0, 4740), glm::vec3(0.7f, 0.5f, 0.3f)); // 第九大行星

    b.emplace_back("Halley", 2.2e14, glm::dvec3(35.08 * AU, 0, 0), glm::dvec3(0, 0, 910), glm::vec3(1.0f, 1.0f, 1.0f));
    b.emplace_back("Voyager", 1e4, glm::dvec3(1.1 * AU, 0, 0), glm::dvec3(0, 0, 35000), glm::vec3(1.0f, 1.0f, 1.0f), true);
    shared_pos.resize(b.size());
    std::barrier sync((int)b.size());
    vector<thread> workers;
    for (int i = 0; i < (int)b.size(); i++) workers.emplace_back(physics_worker, i, ref(sync));
    render_loop();
    for (auto& t : workers) t.join();
    return 0;
}