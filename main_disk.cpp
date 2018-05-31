#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui/imgui.h>
#include <imgui_impl_glfw_gl3.h>
#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>

#include "helpers.h"
#include "MenuPlugin.h"
#include "CameraPlugin.h"
#include "RealtimeReconstructionBuilder.h"
#include "ReconstructionPlugin.h"

int main(int argc, char *argv[]) {

    // std::string images_path =
    //         "/home/kristian/Documents/reconstruction_code/realtime_reconstruction_theia/dataset/castle_images/";
    // std::string reconstruction_path =
    //         "/home/kristian/Documents/reconstruction_code/realtime_reconstruction_theia/dataset/castle_reconstruction/";

    std::string images_path =
            "/home/kristian/Documents/reconstruction_code/realtime_reconstruction_theia/dataset/vrc_images/";
    std::string reconstruction_path =
            "/home/kristian/Documents/reconstruction_code/realtime_reconstruction_theia/dataset/vrc_reconstruction/";

    std::string calibration_file = reconstruction_path + "prior_calibration.txt";

    // Initialize the viewer
    igl::opengl::glfw::Viewer viewer;
    viewer.core.is_animating = true;
    viewer.core.set_rotation_type(igl::opengl::ViewerCore::RotationType::ROTATION_TYPE_TRACKBALL);
    viewer.data().point_size = 3;

    // Setup viewer callbacks for ImGui
    viewer.callback_init = [] (igl::opengl::glfw::Viewer& v) -> bool {
        // Setup ImGui
        ImGui::CreateContext();
        ImGui_ImplGlfwGL3_Init(v.window, false);
        ImGui::GetIO().IniFilename = nullptr;
        ImGui::StyleColorsDark();
        ImGuiStyle& style = ImGui::GetStyle();
        style.FrameRounding = 5.0f;
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

    // Attach a menu plugin
    MenuPlugin menu_plugin;
    viewer.plugins.push_back(&menu_plugin);

    // Attach reconstruction plugin
    ReconstructionPlugin::Parameters parameters;
    theia::CameraIntrinsicsPrior intrinsics_prior = ReadCalibration(calibration_file);
    theia::RealtimeReconstructionBuilder::Options options = SetRealtimeReconstructionBuilderOptions();

    std::vector<std::string> image_names;
    for (int i = 0; i < 20; i++) {
        std::stringstream ss;
        ss << std::setw(3) << std::setfill('0') << std::to_string(i);
        image_names.emplace_back("frame" + ss.str() + ".png");
    }

    ReconstructionPlugin reconstruction_plugin(
            parameters, images_path, image_names, reconstruction_path, options, intrinsics_prior);
    viewer.plugins.push_back(&reconstruction_plugin);

    // Start viewer
    viewer.launch();
}