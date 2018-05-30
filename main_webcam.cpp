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

    // Initialization
    std::string camera_device = "/dev/video0";
    std::string images_path =
            "/home/kristian/Documents/reconstruction_code/realtime_reconstruction_theia/dataset/webcam_images/";
    std::string reconstruction_path =
            "/home/kristian/Documents/reconstruction_code/realtime_reconstruction_theia/dataset/webcam_reconstruction/";

    std::string calibration_file = reconstruction_path + "prior_calibration.txt";
    theia::CameraIntrinsicsPrior intrinsics_prior = ReadCalibration(calibration_file);

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

    // Attach camera plugin
    int image_width = intrinsics_prior.image_width;
    int image_height = intrinsics_prior.image_height;
    CameraPlugin camera_plugin(camera_device, image_width, image_height, images_path);
    viewer.plugins.push_back(&camera_plugin);

    // Attach reconstruction plugin
    ReconstructionPlugin::Parameters parameters;
    theia::RealtimeReconstructionBuilder::Options options = SetRealtimeReconstructionBuilderOptions();

    const std::vector<std::string>& image_names = camera_plugin.get_captured_image_names();
    ReconstructionPlugin reconstruction_plugin(
            parameters, images_path, image_names, reconstruction_path, options, intrinsics_prior);
    viewer.plugins.push_back(&reconstruction_plugin);

    // Start viewer
    viewer.launch();
}