#include <sstream>
#include <iomanip>
#include <chrono>

// #define IGL_STATIC_LIBRARY

#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include <imgui_impl_glfw_gl3.h>
#include <GLFW/glfw3.h>
#include <MenuPlugin.h>

#include "CameraPlugin.h"
#include "RealtimeReconstructionBuilder.h"

theia::CameraIntrinsicsPrior GetCalibration() {
    theia::CameraIntrinsicsPrior camera_intrinsics_prior;
    camera_intrinsics_prior.image_width = 960;
    camera_intrinsics_prior.image_height = 720;

    camera_intrinsics_prior.focal_length.is_set = true;
    camera_intrinsics_prior.focal_length.value[0] = 928.0;

    camera_intrinsics_prior.principal_point.is_set = true;
    camera_intrinsics_prior.principal_point.value[0] = 444.0;
    camera_intrinsics_prior.principal_point.value[1] = 350.0;

    camera_intrinsics_prior.aspect_ratio.is_set = true;
    camera_intrinsics_prior.aspect_ratio.value[0] = 1.0;

    camera_intrinsics_prior.skew.is_set = true;
    camera_intrinsics_prior.skew.value[0] = 0.0;

    camera_intrinsics_prior.radial_distortion.is_set = true;
    camera_intrinsics_prior.radial_distortion.value[0] = 0.10837192;
    camera_intrinsics_prior.radial_distortion.value[1] = -0.21332444;

    return camera_intrinsics_prior;
}

int main(int argc, char *argv[]) {

    // Setup reconstruction builder
    theia::CameraIntrinsicsPrior intrinsics_prior = GetCalibration();
    theia::RealtimeReconstructionBuilderOptions options;
    options.intrinsics_prior = intrinsics_prior;
    theia::RealtimeReconstructionBuilder reconstruction_builder(options);

    // Read the mesh
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;
    igl::readOFF("../assets/bunny.off", vertices, faces);

    // Initialize the viewer
    igl::opengl::glfw::Viewer viewer;
    viewer.core.is_animating = true;

    // Setup viewer callbacks for ImGui
    viewer.callback_init = [&](igl::opengl::glfw::Viewer v) -> bool {
        // Setup ImGui
        ImGui::CreateContext();
        ImGui_ImplGlfwGL3_Init(v.window, false);
        ImGui::GetIO().IniFilename = nullptr;
        ImGui::StyleColorsDark();
        ImGuiStyle& style = ImGui::GetStyle();
        style.FrameRounding = 5.0f;
        return false;
    };

    viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer v) -> bool {
        glfwPollEvents();
        ImGui_ImplGlfwGL3_NewFrame();
        return false;
    };

    viewer.callback_post_draw = [&](igl::opengl::glfw::Viewer v) -> bool {
        ImGui::Render();
        return false;
    };

    viewer.callback_shutdown = [&](igl::opengl::glfw::Viewer v) -> bool {
        // ImGui cleanup
        ImGui_ImplGlfwGL3_Shutdown();
        ImGui::DestroyContext();
        return false;
    };

    // Attach a menu plugin
    MenuPlugin menu;
    viewer.plugins.push_back(&menu);

    // Attach camera plugin
    int image_width = 640;
    int image_height = 480;
    std::string device = "/dev/video1";
    std::string output_path = "../webcam_images/";
    CameraPlugin cam(device, image_width, image_height, output_path);
    viewer.plugins.push_back(&cam);

    // Start viewer
    viewer.data().set_mesh(vertices, faces);
    viewer.launch();
}