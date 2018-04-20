#include "ReconstructionPlugin.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw_gl3.h>

ReconstructionPlugin::ReconstructionPlugin(theia::RealtimeReconstructionBuilderOptions options) {
    theia::RealtimeReconstructionBuilder reconstruction_builder(options);
}

void ReconstructionPlugin::init(igl::opengl::glfw::Viewer *_viewer) {
    ViewerPlugin::init(_viewer);
}

bool ReconstructionPlugin::post_draw() {
    // Setup window
    float window_width = 180.0f;
    ImGui::SetNextWindowSize(ImVec2(window_width, 0.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(180.0f, 0.0f), ImGuiCond_FirstUseEver);

    ImGui::Begin("Reconstruction", nullptr, ImGuiWindowFlags_NoSavedSettings);

    if (ImGui::Button("Initialize", ImVec2(-1,0))) {

    }

    if (ImGui::Button("Extend", ImVec2(-1,0))) {
        // TODO
    }

    if (ImGui::Button("Mesh from points", ImVec2(-1,0))) {
        // TODO
    }

    ImGui::End();
    return false;
}

// Mouse IO
bool ReconstructionPlugin::mouse_down(int button, int modifier)
{
    ImGui_ImplGlfwGL3_MouseButtonCallback(viewer->window, button, GLFW_PRESS, modifier);
    return ImGui::GetIO().WantCaptureMouse;
}

bool ReconstructionPlugin::mouse_up(int button, int modifier)
{
    return ImGui::GetIO().WantCaptureMouse;
}

bool ReconstructionPlugin::mouse_move(int mouse_x, int mouse_y)
{
    return ImGui::GetIO().WantCaptureMouse;
}

bool ReconstructionPlugin::mouse_scroll(float delta_y)
{
    ImGui_ImplGlfwGL3_ScrollCallback(viewer->window, 0.f, delta_y);
    return ImGui::GetIO().WantCaptureMouse;
}

// Keyboard IO
bool ReconstructionPlugin::key_pressed(unsigned int key, int modifiers)
{
    ImGui_ImplGlfwGL3_CharCallback(nullptr, key);
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool ReconstructionPlugin::key_down(int key, int modifiers)
{
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_PRESS, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool ReconstructionPlugin::key_up(int key, int modifiers)
{
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_RELEASE, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}