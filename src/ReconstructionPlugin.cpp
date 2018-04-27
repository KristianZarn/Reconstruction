#include "ReconstructionPlugin.h"

#include <sstream>
#include <iomanip>
#include <chrono>
#include <utility>
#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw_gl3.h>
#include "theia/sfm/reconstruction.h"
#include <theia/sfm/reconstruction_estimator.h>
#include <theia/sfm/colorize_reconstruction.h>
#include <theia/io/write_ply_file.h>

ReconstructionPlugin::ReconstructionPlugin(theia::RealtimeReconstructionBuilderOptions options,
                                           std::string images_path,
                                           std::string reconstruction_path)
        : images_path_(std::move(images_path)),
          reconstruction_path_(std::move(reconstruction_path)),
          image_idx_(0),
          point_size_(3) {
    reconstruction_builder_ = std::make_unique<theia::RealtimeReconstructionBuilder>(options);
}

void ReconstructionPlugin::init(igl::opengl::glfw::Viewer *_viewer) {
    ViewerPlugin::init(_viewer);
}

bool ReconstructionPlugin::post_draw() {
    // Setup window
    float window_width = 270.0f;
    ImGui::SetNextWindowSize(ImVec2(window_width, 0.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(180.0f, 0.0f), ImGuiCond_FirstUseEver);

    ImGui::Begin("Reconstruction", nullptr, ImGuiWindowFlags_NoSavedSettings);

    if (ImGui::Button("Initialize", ImVec2(-1,0))) {

        // Read two images and initialize reconstruction
        std::cout << "Starting initialization" << std::endl;
        auto time_begin = std::chrono::steady_clock::now();

        std::string image0 = image_fullpath(image_idx_);
        image_idx_++;
        std::string image1 = image_fullpath(image_idx_);
        image_idx_++;
        theia::ReconstructionEstimatorSummary summary =
                reconstruction_builder_->InitializeReconstruction(image0, image1);

        auto time_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_elapsed = time_end - time_begin;
        std::cout << "Initialization time: " << time_elapsed.count() << " s" << std::endl;

        // Reconstruction summary
        theia::Reconstruction* reconstruction = reconstruction_builder_->GetReconstruction();
        if (summary.success) {
            std::cout << "Initialization successful" << std::endl;
            std::cout << "\n\tNum estimated views = " << summary.estimated_views.size()
                      << "\n\tNum input views = " << reconstruction->NumViews()
                      << "\n\tNum estimated tracks = " << summary.estimated_tracks.size()
                      << "\n\tNum input tracks = " << reconstruction->NumTracks()
                      << "\n\tPose estimation time = " << summary.pose_estimation_time
                      << "\n\tTriangulation time = " << summary.triangulation_time
                      << "\n\tBundle Adjustment time = " << summary.bundle_adjustment_time
                      << "\n\tTotal time = " << summary.total_time << "\n\n";
        } else {
            std::cout << "Initialization failed" << std::endl;
        }

        // Colorize and show in viewer
        if (summary.success) {
            // Colorize reconstruction
            theia::ColorizeReconstruction(images_path_, 4, reconstruction);

            // Show result in viewer
            Eigen::MatrixXd points = reconstruction_builder_->GetReconstructedPoints();
            Eigen::MatrixXd colors = reconstruction_builder_->GetPointColors();
            colors = colors / 255.0;

            viewer->data().clear();
            viewer->data().set_points(points, colors);

            // Center object
            viewer->core.align_camera_center(points);
        }
    }

    if (ImGui::Button("Extend", ImVec2(-1, 0))) {
        // TODO
    }

    ImGui::Spacing();

    if (ImGui::Button("Mesh from points", ImVec2(-1, 0))) {
        // TODO
    }

    ImGui::Spacing();

    ImGui::SliderInt("Point size", &point_size_, 1, 10);
    if (viewer->data().point_size != point_size_) {
        viewer->data().point_size = point_size_;
    }

    ImGui::Spacing();

    if (ImGui::Button("Write PLY", ImVec2(-1, 0))) {
        theia::WritePlyFile(reconstruction_path_ + "reconstruction.ply",
                            *(reconstruction_builder_->GetReconstruction()), 2);
    }

    if (ImGui::Button("Reset reconstruction", ImVec2(-1, 0))) {
        // TODO
    }

    ImGui::End();
    return false;
}

std::string ReconstructionPlugin::image_fullpath(int image_idx) {
    std::stringstream ss;
    ss << std::setw(3) << std::setfill('0') << std::to_string(image_idx);
    std::string fullpath = images_path_ + "frame" + ss.str() + ".png";
    return fullpath;
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