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
#include <theia/sfm/reconstruction_estimator_utils.h>

ReconstructionPlugin::ReconstructionPlugin(theia::RealtimeReconstructionBuilder::Options options,
                                           theia::CameraIntrinsicsPrior intrinsics_prior,
                                           std::string images_path, std::string reconstruction_path)
        : next_image_id_(0),
          images_path_(std::move(images_path)),
          reconstruction_path_(std::move(reconstruction_path)),
          point_size_(3),
          view_to_delete_(0) {
    reconstruction_builder_ = std::make_unique<theia::RealtimeReconstructionBuilder>(options, intrinsics_prior);
}

void ReconstructionPlugin::init(igl::opengl::glfw::Viewer *_viewer) {
    ViewerPlugin::init(_viewer);
}

bool ReconstructionPlugin::post_draw() {
    // Setup window
    float window_width = 270.0f;
    float window_height = 600.0f;
    ImGui::SetNextWindowSize(ImVec2(window_width, window_height), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(180.0f, 0.0f), ImGuiCond_FirstUseEver);

    ImGui::Begin("Reconstruction", nullptr, ImGuiWindowFlags_NoSavedSettings);

    ImGui::Text("Sparse reconstruction:");
    if (ImGui::Button("Initialize", ImVec2(-1,0))) {

        // TODO: Check if images exist (+print used image names)
        // Images for initial reconstruction
        std::string image0 = image_fullpath(next_image_id_);
        next_image_id_++;
        std::string image1 = image_fullpath(next_image_id_);
        next_image_id_++;

        // Initialize reconstruction
        log_stream_ << "Starting initialization" << std::endl;
        auto time_begin = std::chrono::steady_clock::now();

        theia::ReconstructionEstimatorSummary summary =
                reconstruction_builder_->InitializeReconstruction(image0, image1);

        auto time_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_elapsed = time_end - time_begin;
        log_stream_ << "Initialization time: " << time_elapsed.count() << " s" << std::endl;

        // Reconstruction summary
        if (summary.success) {
            reconstruction_builder_->PrintStatistics(log_stream_);
            refresh_viewer_data();
        } else {
            log_stream_ << "Initialization failed: \n";
            log_stream_ << "\n\tMessage = " << summary.message << "\n\n";

            reset_reconstruction();

            log_stream_ << "Reconstruction is reset" << std::endl;
        }
    }
    if (ImGui::Button("Extend", ImVec2(-1, 0))) {

        // TODO: Check if image exist (+print used image name)
        // Image for extend
        std::string image = image_fullpath(next_image_id_);
        next_image_id_++;

        // Extend reconstruction
        log_stream_ << "Extending reconstruction" << std::endl;
        auto time_begin = std::chrono::steady_clock::now();

        theia::ReconstructionEstimatorSummary summary =
                reconstruction_builder_->ExtendReconstruction(image);

        auto time_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_elapsed = time_end - time_begin;
        log_stream_ << "Extend time: " << time_elapsed.count() << " s" << std::endl;

        // Reconstruction summary
        if (summary.success) {
            reconstruction_builder_->PrintStatistics(log_stream_);
            refresh_viewer_data();
        } else {
            log_stream_ << "Extend failed: \n";
            log_stream_ << "\n\tMessage = " << summary.message << "\n\n";

            // TODO remove last view
        }
    }
    ImGui::Spacing();

    ImGui::Text("Edit views:");
    ImGui::PushItemWidth(100.0f);
    ImGui::InputInt("", &view_to_delete_);
    ImGui::PopItemWidth();
    ImGui::SameLine();
    if (ImGui::Button("Remove view", ImVec2(-1, 0))) {
        log_stream_ << "View with id = " << view_to_delete_ << " removed." << std::endl;
        reconstruction_builder_->RemoveView(static_cast<theia::ViewId>(view_to_delete_));
        reconstruction_builder_->PrintStatistics(log_stream_);
        refresh_viewer_data();
    }
    if (ImGui::Button("Remove last view", ImVec2(-1, 0))) {
        // TODO remove last view
    }
    if (ImGui::Button("Reset reconstruction", ImVec2(-1, 0))) {
        log_stream_ << "Reconstruction is reset" << std::endl;
        reset_reconstruction();
    }
    ImGui::Spacing();

    ImGui::Text("Dense reconstruction:");
    if (ImGui::Button("Mesh from points", ImVec2(-1, 0))) {
        // TODO mesh from points (openMVS)
    }
    ImGui::Spacing();

    ImGui::Text("Display options:");
    ImGui::SliderInt("Point size", &point_size_, 1, 10);
    if (viewer->data().point_size != point_size_) {
        viewer->data().point_size = point_size_;
    }
    if (ImGui::Button("Refresh reconstruction")) {
        refresh_viewer_data();
    }
    ImGui::Spacing();

    ImGui::Text("Output");
    if (ImGui::Button("Write PLY", ImVec2(-1, 0))) {
        std::string filename = "reconstruction.ply";
        reconstruction_builder_->WritePly(reconstruction_path_ + filename);
        log_stream_ << "Written to: \n\t" << (reconstruction_path_ + filename) << std::endl;
    }
    ImGui::Spacing();

    ImGui::BeginGroup();
    ImGui::Text("Log:");
    ImGui::BeginChild("log", ImVec2(0, 0), true, ImGuiWindowFlags_HorizontalScrollbar);
    ImGui::Text("%s", log_stream_.str().c_str());
    ImGui::EndChild();
    ImGui::EndGroup();

    ImGui::End();
    return false;
}

void ReconstructionPlugin::refresh_viewer_data() {
    reconstruction_builder_->ColorizeReconstruction(images_path_);
    theia::Reconstruction* reconstruction = reconstruction_builder_->GetReconstruction();

    // Add points and colors
    std::unordered_set<theia::TrackId> track_ids;
    theia::GetEstimatedTracksFromReconstruction(*reconstruction, &track_ids);

    auto num_points = static_cast<int>(track_ids.size());
    Eigen::MatrixXd points(num_points, 3);
    Eigen::MatrixXd colors(num_points, 3);

    int i = 0;
    for (const auto& track_id : track_ids) {
        const theia::Track* track = reconstruction->Track(track_id);

        Eigen::Vector3d point = track->Point().hnormalized();
        points(i, 0) = point(0);
        points(i, 1) = point(1);
        points(i, 2) = point(2);

        Eigen::Matrix<uint8_t, 3, 1> color = track->Color();
        colors(i, 0) = color(0);
        colors(i, 1) = color(1);
        colors(i, 2) = color(2);

        i++;
    }
    colors = colors / 255.0;

    viewer->data().clear();
    viewer->data().set_points(points, colors);

    // Add cameras
    std::unordered_set<theia::ViewId> view_ids;
    theia::GetEstimatedViewsFromReconstruction(*reconstruction, &view_ids);

    auto num_views = static_cast<int>(view_ids.size());
    Eigen::MatrixXd cameras(num_views, 3);

    i = 0;
    for (const auto& view_id : view_ids) {
        Eigen::Vector3d position = reconstruction->View(view_id)->Camera().GetPosition();
        cameras(i, 0) = position(0);
        cameras(i, 1) = position(1);
        cameras(i, 2) = position(2);

        // Add camera label
        viewer->data().add_label(position, std::to_string(view_id));
        i++;
    }
    viewer->data().add_points(cameras, Eigen::RowVector3d(0, 1, 0));

    // Center object
    viewer->core.align_camera_center(points);
}

void ReconstructionPlugin::reset_reconstruction() {
    next_image_id_ = 0;
    reconstruction_builder_->ResetReconstruction();
    viewer->data().clear();
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