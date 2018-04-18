#include <igl/readOFF.h>
#include <igl/png/readPNG.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include <GLFW/glfw3.h>
#include <glad/glad.h>

#include "webcam.h"
#include "RealtimeReconstructionBuilder.h"

int main(int argc, char *argv[]) {

    // Setup camera
    int image_width = 640;
    int image_height = 480;
    Webcam webcam("/dev/video1", image_width, image_height);

    // Read the mesh
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;
    igl::readOFF("../assets/bunny.off", vertices, faces);

    // Initialize the viewer
    igl::opengl::glfw::Viewer viewer;
    viewer.core.is_animating = true;

    // Create texture for camera view
    GLuint textureID;
    viewer.callback_init = [&](igl::opengl::glfw::Viewer viewer1) -> bool {
        glGenTextures(1, &textureID);
        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image_width, image_height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
        glBindTexture(GL_TEXTURE_2D, 0);
        return false;
    };

    // Attach a menu plugin
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);

    // Reconstruction variables
    int saved_frames_count = 0;

    // Draw additional windows
    menu.callback_draw_custom_window = [&]() {
        // Define next window position
        ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(640, 540), ImGuiCond_FirstUseEver);
        ImGui::Begin("Camera", nullptr, ImGuiWindowFlags_NoSavedSettings);

        // Get frame from webcam
        auto frame = webcam.frame();

        // Replace texture with new frame
        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, image_width, image_height, GL_RGB, GL_UNSIGNED_BYTE, frame.data);
        glBindTexture(GL_TEXTURE_2D, 0);

        // Add an image
        float width = ImGui::GetWindowContentRegionWidth();
        float height = ImGui::GetWindowHeight();
        ImGui::Image(reinterpret_cast<GLuint*>(textureID), ImVec2(width, height - 60));

        // Add a button
        if (ImGui::Button("Capture frame", ImVec2(-1,0)))
        {
            std::cout << "TODO: capture frame.\n";

            saved_frames_count++;
        }
        ImGui::End();
        ImGui::SetNextWindowSize(ImVec2(0, 0), ImGuiCond_Once);
    };

    // Start viewer
    viewer.data().set_mesh(vertices, faces);
    viewer.launch();
}