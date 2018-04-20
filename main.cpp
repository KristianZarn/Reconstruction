#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui/imgui.h>
#include <imgui_impl_glfw_gl3.h>
#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
// #include <Eigen/Core>

#include "helpers.h"
#include "MenuPlugin.h"
#include "CameraPlugin.h"
#include "RealtimeReconstructionBuilder.h"
#include "ReconstructionPlugin.h"

int main(int argc, char *argv[]) {

#ifdef IGL_STATIC_LIBRARY
    std::cout << "IGL_STATIC_LIBRARY is defined" << std::endl;
#endif

#ifndef IGL_STATIC_LIBRARY
    std::cout << "IGL IGL_STATIC_LIBRARY is not defined" << std::endl;
#endif

    // Initialization
    std::string calibration_file = "../webcam_reconstruction/prior_calibration.txt";
    std::string camera_device = "/dev/video1";
    std::string camera_output_path = "../webcam_images/";

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
    MenuPlugin menu_plugin;
    viewer.plugins.push_back(&menu_plugin);

    // Attach camera plugin
    // int image_width = intrinsics_prior.image_width;
    // int image_height = intrinsics_prior.image_height;
    // CameraPlugin camera_plugin(camera_device, image_width, image_height, camera_output_path);
    // viewer.plugins.push_back(&camera_plugin);

    // Attach reconstruction plugin
    theia::CameraIntrinsicsPrior intrinsics_prior = read_calibration(calibration_file);
    theia::RealtimeReconstructionBuilderOptions options;
    options.intrinsics_prior = intrinsics_prior;
    ReconstructionPlugin reconstruction_plugin(options);
    viewer.plugins.push_back(&reconstruction_plugin);

    // Start viewer
    //Eigen::MatrixXd V = Eigen::MatrixXd::Ones(3, 3);
    //Eigen::MatrixXi F = Eigen::MatrixXi::Ones(3, 3);
    //viewer.data().set_mesh(V, F);
    viewer.data().set_mesh(vertices, faces);
    viewer.launch();
}