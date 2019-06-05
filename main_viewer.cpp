#include <string>
#include <ostream>
#include <vector>
#include <memory>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui/imgui.h>
#include <imgui_impl_glfw_gl3.h>
#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>

#include <plugins/ViewMeshPlugin.h>

int main(int argc, char *argv[]) {

    std::string root_folder = "/home/kristian/Documents/reconstruction_code/realtime_reconstruction/dataset_viewer/";

    // Dataset setting
    std::string project_folder = root_folder + "fountain/";

    // Initialize the viewer
    igl::opengl::glfw::Viewer viewer;
    viewer.core.is_animating = true;
    viewer.core.set_rotation_type(igl::opengl::ViewerCore::RotationType::ROTATION_TYPE_TRACKBALL);
    viewer.core.trackball_angle = Eigen::Quaternionf(0, -1, 0, 0);
    viewer.data().point_size = 3;

    // For better screenshots
    viewer.core.background_color = Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f);

    // Setup viewer callbacks for ImGui
    viewer.callback_init = [] (igl::opengl::glfw::Viewer& v) -> bool {
        // Setup ImGui
        ImGui::CreateContext();
        ImGui_ImplGlfwGL3_Init(v.window, false);
        ImGui::GetIO().IniFilename = nullptr;
        ImGui::StyleColorsDark();
        ImGuiStyle& style = ImGui::GetStyle();
        style.FrameRounding = 5.0f;

        ImFontConfig font_cfg;
        font_cfg.GlyphExtraSpacing.x = 1.1f;
        // ImGui::GetIO().Fonts->AddFontFromFileTTF("/home/kristian/Documents/reconstruction_code/realtime_reconstruction/resources/OpenSans-ExtraBold.ttf", 14.0f, &font_cfg);
        ImGui::GetIO().Fonts->AddFontFromFileTTF("/home/kristian/Documents/reconstruction_code/realtime_reconstruction/resources/Roboto-Bold.ttf", 16.0f, &font_cfg);
        return false;
    };

    viewer.callback_pre_draw = [] (igl::opengl::glfw::Viewer& v) -> bool {
        glfwPollEvents();
        ImGui_ImplGlfwGL3_NewFrame();
        return false;
    };

    viewer.callback_post_draw = [] (igl::opengl::glfw::Viewer& v) -> bool {
        ImGui::Render();
        return false;
    };

    viewer.callback_shutdown = [] (igl::opengl::glfw::Viewer& v) -> bool {
        // ImGui cleanup
        ImGui_ImplGlfwGL3_Shutdown();
        ImGui::DestroyContext();
        return false;
    };

    // Attach viewer plugin
    ViewMeshPlugin view_mesh_plugin(project_folder);
    viewer.plugins.push_back(&view_mesh_plugin);

    // Start viewer
    viewer.launch();
}
