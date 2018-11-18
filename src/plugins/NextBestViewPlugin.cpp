#include "NextBestViewPlugin.h"

#include <cmath>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw_gl3.h>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>

#include "nbv/Helpers.h"
#include "nbv/HelpersOptim.h"

NextBestViewPlugin::NextBestViewPlugin(std::shared_ptr<NextBestView> nbv)
        : next_best_view_(std::move(nbv)) {}

void NextBestViewPlugin::init(igl::opengl::glfw::Viewer *_viewer) {
    ViewerPlugin::init(_viewer);

    // Append mesh for camera
    viewer->append_mesh();
    VIEWER_DATA_NBV = static_cast<unsigned int>(viewer->data_list.size() - 1);

    // Initial gizmo pose
    Eigen::Affine3f translation(Eigen::Translation3f(camera_pos_[0], camera_pos_[1], camera_pos_[2]));
    Eigen::Affine3f pitch(Eigen::AngleAxisf(camera_rot_[0], Eigen::Vector3f::UnitX()));
    Eigen::Affine3f yaw(Eigen::AngleAxisf(camera_rot_[1], Eigen::Vector3f::UnitY()));
    Eigen::Affine3f roll(Eigen::AngleAxisf(camera_rot_[2], Eigen::Vector3f::UnitZ()));
    camera_gizmo_ = translation * yaw * pitch * roll * Eigen::Matrix4f::Identity();
}

bool NextBestViewPlugin::pre_draw() {
    ImGuizmo::BeginFrame();
    return false;
}

bool NextBestViewPlugin::post_draw() {
    // Setup window
    float window_width = 350.0f;
    ImGui::SetNextWindowSize(ImVec2(window_width, 0), ImGuiCond_Always);
    ImGui::SetNextWindowPos(ImVec2(350.0f, 0.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin("Next best view", nullptr, ImGuiWindowFlags_NoSavedSettings);

    // Prepare NBV object
    if (ImGui::Button("Initialize NBV", ImVec2(-1, 0))) {
        next_best_view_->Initialize();
        camera_visible_ = true;
    }

    // Optimize camera pose
    ImGui::Text("Camera pose");
    ImGui::InputFloat3("Position", glm::value_ptr(camera_pos_));
    ImGui::InputFloat3("Angles", glm::value_ptr(camera_rot_));
    // ImGui::SliderFloat3("Angles", glm::value_ptr(camera_rot_), -M_PI, M_PI, "%.5f");
    if (ImGui::Button("Optimize position [t]", ImVec2(-1, 0))) {
        optimize_position_callback();
    }
    if (ImGui::Button("Optimize rotation [r]", ImVec2(-1, 0))) {
        optimize_rotation_callback();
    }
    ImGui::Checkbox("Show next best view camera", &camera_visible_);
    show_camera();

    // Pose NBV camera by hand
    ImGui::Checkbox("Pose camera", &pose_camera_);
    if (pose_camera_) {
        // Gizmo setup
        ImGuiIO& io = ImGui::GetIO();
        ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
        Eigen::Affine3f scale_base_zoom(Eigen::Scaling(1.0f / viewer->core.camera_base_zoom));
        Eigen::Affine3f scale_zoom(Eigen::Scaling(1.0f / viewer->core.camera_zoom));
        Eigen::Matrix4f view = scale_base_zoom * scale_zoom * viewer->core.view;

        // Show gizmo
        ImGuizmo::Manipulate(view.data(),
                             viewer->core.proj.data(),
                             gizmo_operation_,
                             gizmo_mode_,
                             camera_gizmo_.data());

        ImGui::Text("Camera options:");
        if (ImGui::RadioButton("Translate", gizmo_operation_ == ImGuizmo::TRANSLATE)) {
            gizmo_operation_ = ImGuizmo::TRANSLATE;
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Rotate", gizmo_operation_ == ImGuizmo::ROTATE)) {
            gizmo_operation_ = ImGuizmo::ROTATE;
        }
    }

    if (pose_camera_) {
        Eigen::Vector4f position = camera_gizmo_.col(3);
        camera_pos_[0] = position(0);
        camera_pos_[1] = position(1);
        camera_pos_[2] = position(2);

        Eigen::Matrix3f tmp = camera_gizmo_.topLeftCorner(3, 3);
        camera_rot_[0] = atan2(-tmp(1,2), tmp(1,1));
        camera_rot_[1] = atan2(-tmp(2,0), tmp(0,0));
        camera_rot_[2] = asin(tmp(1,0));
    } else {
        Eigen::Affine3f translation(Eigen::Translation3f(camera_pos_[0], camera_pos_[1], camera_pos_[2]));
        Eigen::Affine3f pitch(Eigen::AngleAxisf(camera_rot_[0], Eigen::Vector3f::UnitX()));
        Eigen::Affine3f yaw(Eigen::AngleAxisf(camera_rot_[1], Eigen::Vector3f::UnitY()));
        Eigen::Affine3f roll(Eigen::AngleAxisf(camera_rot_[2], Eigen::Vector3f::UnitZ()));
        camera_gizmo_ = translation * yaw * pitch * roll * Eigen::Matrix4f::Identity();
    }

    // Debugging
    if (ImGui::Button("Debug [d]", ImVec2(-1, 0))) {
        debug_callback();
    }

    ImGui::End();
    return false;
}

void NextBestViewPlugin::debug_callback() {
    log_stream_ << "Debug button pressed" << std::endl;

    // Camera parameters
    unsigned int image_width = next_best_view_->mvs_scene_->images.front().width;
    unsigned int image_height = next_best_view_->mvs_scene_->images.front().height;
    double focal_y = next_best_view_->mvs_scene_->images.front().camera.K(1, 1);
    auto view_matrix = generate_view_matrix(camera_pos_, camera_rot_);

    double cost_pos = next_best_view_->CostFunctionPosition(view_matrix, image_width, image_height, focal_y);
    double cost_rot = next_best_view_->CostFunctionRotation(view_matrix, image_width, image_height, focal_y);

    // auto render_data = next_best_view_->RenderFaceIdFromCamera(view_matrix, image_width, image_height, focal_y);
    // writeBufferToFile("/home/kristian/Documents/reconstruction_code/realtime_reconstruction/resources/render.dat",
    //         image_width, image_height, render_data);

    // log_stream_ << "Cost position: " << cost_pos << std::endl;
    // log_stream_ << "Cost rotation: " << cost_rot << std::endl;
}

void NextBestViewPlugin::optimize_position_callback() {

    // Camera parameters
    unsigned int image_width = next_best_view_->mvs_scene_->images.front().width;
    unsigned int image_height = next_best_view_->mvs_scene_->images.front().height;
    double focal_y = next_best_view_->mvs_scene_->images.front().camera.K(1, 1);

    // Optimization parameters
    arma::vec param_pos = arma::zeros(3, 1);
    param_pos(0) = camera_pos_[0];
    param_pos(1) = camera_pos_[1];
    param_pos(2) = camera_pos_[2];

    optim::algo_settings_t optim_pos_settings;
    optim_pos_settings.iter_max = 1;
    optim_pos_settings.gd_method = 0;
    optim::gd_settings_t gd_settings_pos;
    gd_settings_pos.step_size = 0.1;
    optim_pos_settings.gd_settings = gd_settings_pos;

    OptimPosData optim_pos_data{next_best_view_.get(), camera_rot_, image_width, image_height, focal_y};

    // Run optimization
    auto time_begin = std::chrono::steady_clock::now();

    bool success = optim::gd(param_pos, optim_pos_function, &optim_pos_data, optim_pos_settings);
    camera_pos_[0] = param_pos(0);
    camera_pos_[1] = param_pos(1);
    camera_pos_[2] = param_pos(2);

    auto time_end = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_elapsed = time_end - time_begin;
    std::cout << "Elapsed time: " << time_elapsed.count() << " s" << std::endl;

    // Debug
    std::cout << "Parameters: \n" << param_pos;
}

void NextBestViewPlugin::optimize_rotation_callback() {

    // Camera parameters
    unsigned int image_width = next_best_view_->mvs_scene_->images.front().width;
    unsigned int image_height = next_best_view_->mvs_scene_->images.front().height;
    double focal_y = next_best_view_->mvs_scene_->images.front().camera.K(1, 1);

    // Optimization parameters
    arma::vec param_rot = arma::zeros(3, 1);
    param_rot(0) = camera_rot_[0];
    param_rot(1) = camera_rot_[1];
    param_rot(2) = camera_rot_[2];

    optim::algo_settings_t optim_rot_settings;
    optim_rot_settings.iter_max = 1;
    optim_rot_settings.gd_method = 0;
    optim::gd_settings_t gd_settings_rot;
    gd_settings_rot.step_size = 0.1;
    optim_rot_settings.gd_settings = gd_settings_rot;

    OptimRotData optim_rot_data{next_best_view_.get(), camera_pos_, image_width, image_height, focal_y};

    // Run optimization
    auto time_begin = std::chrono::steady_clock::now();

    bool success = optim::gd(param_rot, optim_rot_function, &optim_rot_data, optim_rot_settings);
    camera_rot_[0] = param_rot(0);
    camera_rot_[1] = param_rot(1);
    camera_rot_[2] = param_rot(2);

    auto time_end = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_elapsed = time_end - time_begin;
    std::cout << "Elapsed time: " << time_elapsed.count() << " s" << std::endl;

    // Debug
    std::cout << "Parameters: \n" << param_rot;
}

void NextBestViewPlugin::show_camera() {
    viewer->selected_data_index = VIEWER_DATA_NBV;
    if (camera_visible_) {

        // Compute camera transformation
        Eigen::Matrix4d camera_transformation_;
        Eigen::Affine3d scale(Eigen::Scaling(1.0 / 2.0));
        Eigen::Affine3d translation(Eigen::Translation3d(camera_pos_[0], camera_pos_[1], camera_pos_[2]));
        Eigen::Affine3d pitch(Eigen::AngleAxisd(camera_rot_[0], Eigen::Vector3d::UnitX()));
        Eigen::Affine3d yaw(Eigen::AngleAxisd(camera_rot_[1], Eigen::Vector3d::UnitY()));
        Eigen::Affine3d roll(Eigen::AngleAxisd(camera_rot_[2], Eigen::Vector3d::UnitZ()));
        camera_transformation_ = translation * yaw * pitch * roll * scale * Eigen::Matrix4d::Identity();

        // Vertices
        Eigen::MatrixXd tmp_V(5, 3);
        tmp_V << 0, 0, 0,
                0.75, 0.5, -1,
                -0.75, 0.5, -1,
                -0.75, -0.5, -1,
                0.75, -0.5, -1;

        // Faces
        Eigen::MatrixXi tmp_F(6, 3);
        tmp_F << 0, 1, 2,
                0, 2, 3,
                0, 3, 4,
                0, 4, 1,
                1, 3, 2,
                1, 4, 3;

        // Apply transformation
        tmp_V = (tmp_V.rowwise().homogeneous() * camera_transformation_.transpose()).rowwise().hnormalized();

        // Set viewer data
        viewer->data().clear();
        viewer->data().set_mesh(tmp_V, tmp_F);
        viewer->data().set_face_based(true);
        Eigen::Vector3d blue_color = Eigen::Vector3d(0, 0, 255) / 255.0;
        viewer->data().uniform_colors(blue_color, blue_color, blue_color);

    } else {
        viewer->data().clear();
    }
}

// Mouse IO
bool NextBestViewPlugin::mouse_down(int button, int modifier) {
    ImGui_ImplGlfwGL3_MouseButtonCallback(viewer->window, button, GLFW_PRESS, modifier);
    return ImGui::GetIO().WantCaptureMouse;
}

bool NextBestViewPlugin::mouse_up(int button, int modifier) {
    return ImGui::GetIO().WantCaptureMouse;
}

bool NextBestViewPlugin::mouse_move(int mouse_x, int mouse_y) {
    return ImGui::GetIO().WantCaptureMouse;
}

bool NextBestViewPlugin::mouse_scroll(float delta_y) {
    ImGui_ImplGlfwGL3_ScrollCallback(viewer->window, 0.f, delta_y);
    return ImGui::GetIO().WantCaptureMouse;
}

// Keyboard IO
bool NextBestViewPlugin::key_pressed(unsigned int key, int modifiers) {
    ImGui_ImplGlfwGL3_CharCallback(nullptr, key);
    if (!ImGui::GetIO().WantTextInput) {
        switch (key) {
            case 'r':
            {
                optimize_rotation_callback();
                return true;
            }
            case 't':
            {
                optimize_position_callback();
                return true;
            }
            case 'd':
            {
                debug_callback();
                return true;
            }
            default: break;
        }
    }
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool NextBestViewPlugin::key_down(int key, int modifiers) {
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_PRESS, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool NextBestViewPlugin::key_up(int key, int modifiers) {
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_RELEASE, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}
