#include <igl/readOFF.h>
#include <igl/png/readPNG.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include <GLFW/glfw3.h>

int main(int argc, char *argv[]) {

    // Read the mesh
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;
    igl::readOFF("../assets/bunny.off", vertices, faces);

    // Initialize the viewer
    igl::opengl::glfw::Viewer viewer;

    // Attach a menu plugin
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);

    // Read image data into a vector
    int image_size = 400;
    std::stringstream filename;
    filename << "../assets/img400.dat";
    std::ifstream ifs(filename.str(), std::ios::binary|std::ios::ate);
    std::ifstream::pos_type pos = ifs.tellg();
    std::vector<char> result(pos);
    ifs.seekg(0, std::ios::beg);
    ifs.read(&result[0], pos);

    // Draw additional windows
    menu.callback_draw_custom_window = [&]()
    {
        // Define next window position
        ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiSetCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(0, 0), ImGuiSetCond_FirstUseEver);
        ImGui::Begin(
                "Camera", nullptr,
                ImGuiWindowFlags_NoSavedSettings
        );

        // Give the image data to OpenGL
        GLuint textureID;
        glGenTextures(1, &textureID);
        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image_size, image_size, 0, GL_BGR, GL_UNSIGNED_BYTE, result.data());

        // Add an image
        ImGui::Image(reinterpret_cast<GLuint*>(textureID), ImVec2(400, 400));

        // Add a button
        if (ImGui::Button("Capture frame", ImVec2(-1,0)))
        {
            std::cout << "TODO: capture frame.\n";
        }

        ImGui::End();
    };

    // Plot the mesh
    viewer.data().set_mesh(vertices, faces);
    viewer.launch();
}