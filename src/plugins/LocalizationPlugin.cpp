#include "LocalizationPlugin.h"

#include <utility>
#include <thread>
#include <future>
#include <chrono>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw_gl3.h>

#include <theia/image/image.h>
#include <theia/sfm/two_view_match_geometric_verification.h>

LocalizationPlugin::LocalizationPlugin(std::string images_path,
                                       std::shared_ptr<theia::RealtimeReconstructionBuilder> reconstruction_builder,
                                       RGBImage* camera_frame)
        : images_path_(std::move(images_path)),
          reconstruction_builder_(std::move(reconstruction_builder)),
          camera_frame_(camera_frame) {}

void LocalizationPlugin::init(igl::opengl::glfw::Viewer *_viewer) {
    ViewerPlugin::init(_viewer);

    // Append mesh for camera
    viewer->append_mesh();
    VIEWER_DATA_LOCALIZATION = static_cast<unsigned int>(viewer->data_list.size() - 1);

    // Initial camera pose
    // camera_transformation_ = Eigen::Matrix4d::Identity();
    Eigen::Affine3d scale(Eigen::Scaling(1.0 / 2.0));
    camera_transformation_ = scale * Eigen::Matrix4d::Identity();

    set_camera();
    show_camera(show_camera_);
}

bool LocalizationPlugin::post_draw() {
    // Setup window
    float window_width = 350.0f;
    ImGui::SetNextWindowSize(ImVec2(window_width, 0), ImGuiCond_Always);
    ImGui::SetNextWindowPos(ImVec2(300.0f, 0.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin("Localization", nullptr, ImGuiWindowFlags_NoSavedSettings);

    // Localize image
    ImGui::Text("Image to localize:");
    ImGui::InputText("Filename", filename_buffer_, 64, ImGuiInputTextFlags_AutoSelectAll);
    if (ImGui::Button("Localize image", ImVec2(-1, 0))) {
        localize_image_callback();
    }

    // Display options
    ImGui::Text("Display options:");
    if (ImGui::Checkbox("Show camera", &show_camera_)) {
        show_camera(show_camera_);
    }

    // Camera localization (show only if camera present)
    if (camera_frame_ != nullptr) {
        if (!localization_active_) {
            if (ImGui::Button("Start localization [o]", ImVec2(-1, 0))) {
                toggle_localization_callback(true);
            }
        } else {
            if (ImGui::Button("Stop localization [o]", ImVec2(-1, 0))) {
                toggle_localization_callback(false);
            }

            if (!localization_future_.valid()) {
                localization_future_ = std::async(
                        std::launch::async,
                        &LocalizationPlugin::localize_current_frame,
                        this);
            } else if (localization_future_.valid() &&
                       localization_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                localization_success_ = localization_future_.get();
            }

            ImGui::Text("Status:");
            ImGui::SameLine();
            if (localization_success_) {
                ImGui::ColorButton("Status", ImVec4(0, 1, 0, 0), ImGuiColorEditFlags_NoOptions, ImVec2(50, 0));
                viewer->selected_data_index = VIEWER_DATA_LOCALIZATION;
                viewer->data().set_colors(Eigen::RowVector3d(0, 255, 0)/255.0);
            } else {
                ImGui::ColorButton("Status", ImVec4(1, 0, 0, 0), ImGuiColorEditFlags_NoOptions, ImVec2(50, 0));
                viewer->selected_data_index = VIEWER_DATA_LOCALIZATION;
                viewer->data().set_colors(Eigen::RowVector3d(255, 0, 0)/255.0);
            }
        }
    }

    // Camera
    transform_camera();

    ImGui::End();
    return false;
}

bool LocalizationPlugin::localize_image_callback() {
    log_stream_ << std::endl;

    // Read image
    std::string image_fullpath = images_path_ + std::string(filename_buffer_);
    theia::FloatImage image(image_fullpath);

    // Localize image
    log_stream_ << "Localizing image ..." << std::endl;
    // auto time_begin = std::chrono::steady_clock::now();

    theia::CalibratedAbsolutePose camera_pose;
    std::vector<theia::ViewId> views_to_match = {0, 1};
    bool success = reconstruction_builder_->LocalizeImage(image, views_to_match, camera_pose);

    // auto time_end = std::chrono::steady_clock::now();
    // std::chrono::duration<double> time_elapsed = time_end - time_begin;
    // log_stream_ << "Localize time: " << time_elapsed.count() << " s" << std::endl;

    if (success) {
        log_stream_ << "Localization successful." << std::endl;
    } else {
        log_stream_ << "Localization failed." << std::endl;
    }

    if (success) {
        // Set camera transformation
        Eigen::Affine3d scale(Eigen::Scaling(1.0 / 2.0));
        Eigen::Affine3d rotate;
        rotate = camera_pose.rotation.transpose();
        Eigen::Affine3d translate(Eigen::Translation3d(camera_pose.position));
        camera_transformation_ = (translate * rotate * scale).matrix();
    }

    show_camera(true);
    return success;
}

void LocalizationPlugin::toggle_localization_callback(bool active) {
    localization_active_ = active;
    show_camera(active);
}

bool LocalizationPlugin::localize_current_frame() {
    // log_stream_ << "Localizing image ..." << std::endl;
    // auto time_begin = std::chrono::steady_clock::now();

    // Read image
    camera_frame_data_.reserve(camera_frame_->size);
    for (int i = 0; i < camera_frame_->size; i++) {
        camera_frame_data_[i] = camera_frame_->data[i] / 255.0f;
    }

    theia::FloatImage image(static_cast<const int>(camera_frame_->width),
                            static_cast<const int>(camera_frame_->height),
                            3, camera_frame_data_.data());

    // Localize image
    bool success;
    theia::CalibratedAbsolutePose camera_pose;
    if (localization_success_) {
        success = reconstruction_builder_->LocalizeImage(image, prev_camera_pose_, camera_pose);
    } else {
        success = reconstruction_builder_->LocalizeImage(image, camera_pose);
    }

    if (success) {
        prev_camera_pose_ = camera_pose;

        // Set camera transformation
        Eigen::Affine3d scale(Eigen::Scaling(1.0 / 2.0));
        Eigen::Affine3d rotate;
        rotate = camera_pose.rotation.transpose();
        Eigen::Affine3d translate(Eigen::Translation3d(camera_pose.position));
        camera_transformation_ = (translate * rotate * scale).matrix();
    }

    // auto time_end = std::chrono::steady_clock::now();
    // std::chrono::duration<double> time_elapsed = time_end - time_begin;
    // log_stream_ << "Localize time: " << time_elapsed.count() << " s" << std::endl;

    return success;
}

void LocalizationPlugin::set_camera() {
    // Vertices
    Eigen::MatrixXd tmp_V(5, 3);
    tmp_V << 0, 0, 0,
            -0.75, -0.5, 1,
            -0.75,  0.5, 1,
             0.75,  0.5, 1,
             0.75, -0.5, 1;
    camera_vertices_ = tmp_V;

    // Faces
    Eigen::MatrixXi tmp_F(6, 3);
    tmp_F << 0, 1, 2,
            0, 2, 3,
            0, 3, 4,
            0, 4, 1,
            1, 3, 2,
            1, 4, 3;

    // Set viewer data
    viewer->selected_data_index = VIEWER_DATA_LOCALIZATION;
    viewer->data().clear();
    viewer->data().set_mesh(tmp_V, tmp_F);
    viewer->data().set_face_based(true);
    viewer->data().set_colors(Eigen::RowVector3d(128, 128, 128)/255.0);
}

void LocalizationPlugin::transform_camera() {
    viewer->selected_data_index = VIEWER_DATA_LOCALIZATION;
    if (camera_vertices_.rows() > 0) {
        Eigen::MatrixXd V = camera_vertices_;
        V = (V.rowwise().homogeneous() * camera_transformation_.transpose()).rowwise().hnormalized();
        viewer->data().set_vertices(V);
    }
}

void LocalizationPlugin::show_camera(bool visible) {
    viewer->selected_data_index = VIEWER_DATA_LOCALIZATION;
    viewer->data().show_faces = visible;
    viewer->data().show_lines = visible;
}

// Mouse IO
bool LocalizationPlugin::mouse_down(int button, int modifier) {
    ImGui_ImplGlfwGL3_MouseButtonCallback(viewer->window, button, GLFW_PRESS, modifier);
    return ImGui::GetIO().WantCaptureMouse;
}

bool LocalizationPlugin::mouse_up(int button, int modifier) {
    return ImGui::GetIO().WantCaptureMouse;
}

bool LocalizationPlugin::mouse_move(int mouse_x, int mouse_y) {
    return ImGui::GetIO().WantCaptureMouse;
}

bool LocalizationPlugin::mouse_scroll(float delta_y) {
    ImGui_ImplGlfwGL3_ScrollCallback(viewer->window, 0.f, delta_y);
    return ImGui::GetIO().WantCaptureMouse;
}

// Keyboard IO
bool LocalizationPlugin::key_pressed(unsigned int key, int modifiers) {
    ImGui_ImplGlfwGL3_CharCallback(viewer->window, key);
    if (!ImGui::GetIO().WantTextInput) {
        switch (key) {
            case 'o':
            {
                toggle_localization_callback(!localization_active_);
                return ImGui::GetIO().WantCaptureKeyboard;
            }
            default: break;
        }
    }
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool LocalizationPlugin::key_down(int key, int modifiers) {
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_PRESS, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool LocalizationPlugin::key_up(int key, int modifiers) {
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_RELEASE, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}