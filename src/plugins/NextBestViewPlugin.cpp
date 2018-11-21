#include "NextBestViewPlugin.h"

#include <cmath>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw_gl3.h>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <igl/jet.h>
#include <igl/unproject_onto_mesh.h>

#include "nbv/Helpers.h"
#include "nbv/HelpersOptim.h"

NextBestViewPlugin::NextBestViewPlugin(std::shared_ptr<NextBestView> nbv)
        : next_best_view_(std::move(nbv)) {}

void NextBestViewPlugin::init(igl::opengl::glfw::Viewer *_viewer) {
    ViewerPlugin::init(_viewer);

    // Append mesh for camera
    viewer->append_mesh();
    VIEWER_DATA_CAMERA = static_cast<unsigned int>(viewer->data_list.size() - 1);

    // Append mesh for bounding box
    viewer->append_mesh();
    VIEWER_DATA_BOUNDING_BOX = static_cast<unsigned int>(viewer->data_list.size() - 1);

    // Append mesh for mesh
    viewer->append_mesh();
    VIEWER_DATA_NBV_MESH = static_cast<unsigned int>(viewer->data_list.size() - 1);

    // Initial gizmo pose
    camera_gizmo_ = Eigen::Matrix4f::Identity();
    bounding_box_gizmo_ = Eigen::Matrix4f::Identity();
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

    // Gizmo setup
    ImGuiIO& io = ImGui::GetIO();
    ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
    Eigen::Affine3f scale_base_zoom(Eigen::Scaling(1.0f / viewer->core.camera_base_zoom));
    Eigen::Affine3f scale_zoom(Eigen::Scaling(1.0f / viewer->core.camera_zoom));
    Eigen::Matrix4f gizmo_view = scale_base_zoom * scale_zoom * viewer->core.view;

    // NBV object and pose optimization
    if (ImGui::TreeNodeEx("Optimization", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("Initialize NBV", ImVec2(-1, 0))) {
            initialize_callback();
        }
        if (ImGui::Button("Optimize position [t]", ImVec2(-1, 0))) {
            optimize_position_callback();
        }
        if (ImGui::Button("Optimize rotation [r]", ImVec2(-1, 0))) {
            optimize_rotation_callback();
        }
        ImGui::TreePop();
    }

    // Optimize camera pose
    if (ImGui::TreeNodeEx("Camera pose", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("Show NBV camera", &camera_visible_);
        ImGui::Checkbox("Pose camera", &pose_camera_);
        ImGui::InputFloat3("Position", glm::value_ptr(camera_pos_));
        ImGui::InputFloat3("Angles", glm::value_ptr(camera_rot_));

        if (pose_camera_) {
            // Show gizmo
            ImGuizmo::Manipulate(gizmo_view.data(),
                                 viewer->core.proj.data(),
                                 gizmo_operation_,
                                 gizmo_mode_,
                                 camera_gizmo_.data());

            ImGui::Text("Camera options");
            if (ImGui::RadioButton("Translate", gizmo_operation_ == ImGuizmo::TRANSLATE)) {
                gizmo_operation_ = ImGuizmo::TRANSLATE;
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("Rotate", gizmo_operation_ == ImGuizmo::ROTATE)) {
                gizmo_operation_ = ImGuizmo::ROTATE;
            }
        }
        ImGui::TreePop();
    }
    show_camera();

    // Set camera transformation
    if (pose_camera_) {
        // Camera gizmo -> position and rotation
        glm::vec3 scale;
        ImGuizmo::DecomposeMatrixToComponents(
                camera_gizmo_.data(),
                glm::value_ptr(camera_pos_),
                glm::value_ptr(camera_rot_),
                glm::value_ptr(scale));
    } else {
        // Position and rotation -> camera gizmo
        glm::vec3 scale = glm::vec3(1.0, 1.0, 1.0);
        ImGuizmo::RecomposeMatrixFromComponents(
                glm::value_ptr(camera_pos_),
                glm::value_ptr(camera_rot_),
                glm::value_ptr(scale),
                camera_gizmo_.data());
    }

    // Region of interest
    if (ImGui::TreeNodeEx("Region of interest", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("Show bounding box", &bounding_box_visible_);
        ImGui::Checkbox("Pose bounding box", &pose_bounding_box_);

        if (pose_bounding_box_) {
            // Show gizmo
            ImGuizmo::Manipulate(gizmo_view.data(),
                                 viewer->core.proj.data(),
                                 gizmo_operation_,
                                 gizmo_mode_,
                                 bounding_box_gizmo_.data());

            ImGui::Text("Bounding box options");
            if (ImGui::RadioButton("Translate", gizmo_operation_ == ImGuizmo::TRANSLATE)) {
                gizmo_operation_ = ImGuizmo::TRANSLATE;
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("Rotate", gizmo_operation_ == ImGuizmo::ROTATE)) {
                gizmo_operation_ = ImGuizmo::ROTATE;
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("Scale", gizmo_operation_ == ImGuizmo::SCALE)) {
                gizmo_operation_ = ImGuizmo::SCALE;
            }
        }

        if (ImGui::Button("Apply selection", ImVec2(-1, 0))) {
            apply_selection_callback();
        }
        ImGui::TreePop();
    }
    show_bounding_box();

    // Debugging
    if (ImGui::TreeNodeEx("Debug", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("Recompute local face cost", ImVec2(-1, 0))) {
            local_face_cost_ = next_best_view_->LocalFaceCost(pixels_per_area_);
        }
        ImGui::InputInt("Radius", &next_best_view_->face_cost_radius_);
        ImGui::InputFloat("Alpha", &next_best_view_->alpha_);
        ImGui::InputFloat("Beta", &next_best_view_->beta_);

        if (ImGui::Button("Show PPA", ImVec2(-1, 0))) {
            set_nbv_mesh_color(pixels_per_area_);
        }
        if (ImGui::Button("Show local face cost", ImVec2(-1, 0))) {
            set_nbv_mesh_color(local_face_cost_);
        }
        if (ImGui::Checkbox("Show NBV mesh", &nbv_mesh_visible_)) {
            show_nbv_mesh(nbv_mesh_visible_);
        }
        if (ImGui::Button("Debug [d]", ImVec2(-1, 0))) {
            debug_callback();
        }
        ImGui::TreePop();
    }

    ImGui::End();
    return false;
}

void NextBestViewPlugin::initialize_callback() {
    next_best_view_->Initialize();

    camera_visible_ = true;
    pixels_per_area_ = next_best_view_->PixelsPerArea();
    local_face_cost_ = next_best_view_->LocalFaceCost(pixels_per_area_);

    set_nbv_mesh();
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
    gd_settings_pos.step_size = 0.001;
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
    log_stream_ << "Elapsed time: " << time_elapsed.count() << " s" << std::endl;

    // Debug
    log_stream_ << "Parameters: \n" << param_pos;
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
    gd_settings_rot.step_size = 300;
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
    log_stream_ << "Elapsed time: " << time_elapsed.count() << " s" << std::endl;

    // Debug
    log_stream_ << "Parameters: \n" << param_rot;
}

void NextBestViewPlugin::apply_selection_callback() {

    // Get bounding box vectors
    viewer->selected_data_index = VIEWER_DATA_BOUNDING_BOX;
    const Eigen::MatrixXd& bbox_V = viewer->data().V;
    assert(bbox_V.rows() == 8 && bbox_V.cols() == 3);

    Eigen::Vector3d u = bbox_V.row(4) - bbox_V.row(0);
    Eigen::Vector3d v = bbox_V.row(2) - bbox_V.row(0);
    Eigen::Vector3d w = bbox_V.row(1) - bbox_V.row(0);

    // Add faces inside bounding box to valid faces
    std::unordered_set<unsigned int> valid_faces;
    for (int i = 0; i < next_best_view_->mvs_scene_->mesh.faces.size(); i++) {
        auto mvs_face = next_best_view_->mvs_scene_->mesh.faces[i];

        bool valid = true;
        for (int j = 0; j < 3; j++) {
            auto mvs_vert = next_best_view_->mvs_scene_->mesh.vertices[mvs_face[j]];
            Eigen::Vector3d x;
            x << mvs_vert[0], mvs_vert[1], mvs_vert[2];

            double ux = u.dot(x);
            double vx = v.dot(x);
            double wx = w.dot(x);

            valid = valid && (ux > u.dot(bbox_V.row(0)) && ux < u.dot(bbox_V.row(4)) &&
                              vx > v.dot(bbox_V.row(0)) && vx < v.dot(bbox_V.row(2)) &&
                              wx > w.dot(bbox_V.row(0)) && wx < w.dot(bbox_V.row(1)));
        }

        if (valid) {
            valid_faces.insert(static_cast<unsigned int>(i));
        }
    }

    // Set valid faces in NBV
    if (valid_faces.empty()) {
        log_stream_ << "Valid faces empty: not set." << std::endl;
    } else {
        next_best_view_->SetValidFaces(valid_faces);
        log_stream_ << "Number of valid faces: " << valid_faces.size() << std::endl;
    }
}

void NextBestViewPlugin::debug_callback() {
    log_stream_ << "Debug button pressed" << std::endl;

    // Camera parameters
    unsigned int image_width = next_best_view_->mvs_scene_->images.front().width;
    unsigned int image_height = next_best_view_->mvs_scene_->images.front().height;
    double focal_y = next_best_view_->mvs_scene_->images.front().camera.K(1, 1);

    glm::mat4 view_matrix;
    glm::vec3 scale = glm::vec3(1.0, 1.0, 1.0);
    ImGuizmo::RecomposeMatrixFromComponents(
            glm::value_ptr(camera_pos_),
            glm::value_ptr(camera_rot_),
            glm::value_ptr(scale),
            glm::value_ptr(view_matrix));
    view_matrix = glm::inverse(view_matrix);

    double cost_pos = next_best_view_->CostFunctionPosition(view_matrix, image_width, image_height, focal_y);
    double cost_rot = next_best_view_->CostFunctionRotation(view_matrix, image_width, image_height, focal_y);

    // auto render_data = next_best_view_->RenderFaceIdFromCamera(view_matrix, image_width, image_height, focal_y);
    // writeBufferToFile("/home/kristian/Documents/reconstruction_code/realtime_reconstruction/resources/render.dat",
    //         image_width, image_height, render_data);

    // log_stream_ << "Cost position: " << cost_pos << std::endl;
    // log_stream_ << "Cost rotation: " << cost_rot << std::endl;
}

void NextBestViewPlugin::pick_face_callback() {
    viewer->selected_data_index = VIEWER_DATA_NBV_MESH;

    int face_id;
    Eigen::Vector3f barycentric;
    double x = viewer->current_mouse_x;
    double y = viewer->core.viewport(3) - viewer->current_mouse_y;

    // Cast a ray
    bool hit = igl::unproject_onto_mesh(
            Eigen::Vector2f(x, y),
            viewer->core.view,
            viewer->core.proj,
            viewer->core.viewport,
            viewer->data().V,
            viewer->data().F,
            face_id,
            barycentric);

    if (hit) {
        double ppa = pixels_per_area_[face_id];
        double lfc = local_face_cost_[face_id];
        log_stream_ << "ID: " << face_id << "\tPPA: " << ppa << "\tLFC:" << lfc << std::endl;
    }
}

void NextBestViewPlugin::show_camera() {
    viewer->selected_data_index = VIEWER_DATA_CAMERA;
    if (camera_visible_) {

        // Compute camera transformation
        Eigen::Matrix4f tmp;
        glm::vec3 scale = glm::vec3(1.0 / 2.0, 1.0 / 2.0, 1.0 / 2.0);
        ImGuizmo::RecomposeMatrixFromComponents(
                glm::value_ptr(camera_pos_),
                glm::value_ptr(camera_rot_),
                glm::value_ptr(scale),
                tmp.data());
        Eigen::Matrix4d camera_mat_world = tmp.cast<double>();

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
        tmp_V = (tmp_V.rowwise().homogeneous() * camera_mat_world.transpose()).rowwise().hnormalized();

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

void NextBestViewPlugin::show_bounding_box() {
    viewer->selected_data_index = VIEWER_DATA_BOUNDING_BOX;
    if (bounding_box_visible_) {

        // Bounding box transformation
        Eigen::Matrix4d bbox_mat_world = bounding_box_gizmo_.cast<double>();

        // Vertices
        Eigen::MatrixXd bbox_V(8, 3);
        bbox_V << 0, 0, 0,
                0, 0, 1,
                0, 1, 0,
                0, 1, 1,
                1, 0, 0,
                1, 0, 1,
                1, 1, 0,
                1, 1, 1;
        bbox_V = bbox_V.array() - 0.5;

        // Faces
        Eigen::MatrixXi bbox_F(12, 3);
        bbox_F << 0, 6, 4,
                0, 2, 6,
                0, 3, 2,
                0, 1, 3,
                2, 7, 6,
                2, 3, 7,
                4, 6, 7,
                4, 7, 5,
                0, 4, 5,
                0, 5, 1,
                1, 5, 7,
                1, 7, 3;

        // Apply transformation
        bbox_V = (bbox_V.rowwise().homogeneous() * bbox_mat_world.transpose()).rowwise().hnormalized();

        // Set viewer data
        viewer->data().clear();
        viewer->data().set_mesh(bbox_V, bbox_F);
        viewer->data().set_face_based(true);
        viewer->data().set_colors(Eigen::RowVector4d(0, 255, 0, 64)/255.0);
        viewer->data().show_lines = false;
    } else {
        viewer->data().clear();
    }
}

void NextBestViewPlugin::set_nbv_mesh() {
    viewer->selected_data_index = VIEWER_DATA_NBV_MESH;
    viewer->data().clear();

    // Add vertices
    int num_vertices = next_best_view_->mvs_scene_->mesh.vertices.size();
    Eigen::MatrixXd V(num_vertices, 3);
    for (int i = 0; i < num_vertices; i++) {
        MVS::Mesh::Vertex vertex = next_best_view_->mvs_scene_->mesh.vertices[i];
        V(i, 0) = vertex[0];
        V(i, 1) = vertex[1];
        V(i, 2) = vertex[2];
    }

    // Add faces
    int num_faces = next_best_view_->mvs_scene_->mesh.faces.size();
    Eigen::MatrixXi F(num_faces, 3);
    for (int i = 0; i < num_faces; i++) {
        MVS::Mesh::Face face = next_best_view_->mvs_scene_->mesh.faces[i];
        F(i, 0) = face[0];
        F(i, 1) = face[1];
        F(i, 2) = face[2];
    }

    viewer->data().set_mesh(V, F);
    viewer->data().show_lines = true;
    show_nbv_mesh(true);
}

void NextBestViewPlugin::set_nbv_mesh_color(const std::vector<double>& face_values) {
    viewer->selected_data_index = VIEWER_DATA_NBV_MESH;
    assert(face_values.size() == viewer->data().F.rows());

    int num_faces = face_values.size();
    Eigen::VectorXd tmp(num_faces);
    for (int i = 0; i < num_faces; i++) {
        tmp(i) = face_values[i];
    }

    Eigen::MatrixXd color;
    igl::jet(tmp, true, color);
    viewer->data().set_colors(color);
}

void NextBestViewPlugin::show_nbv_mesh(bool visible) {
    nbv_mesh_visible_ = visible;
    viewer->selected_data_index = VIEWER_DATA_NBV_MESH;
    viewer->data().show_faces = visible;
    viewer->data().show_lines = visible;
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
            case 's':
            {
                if (!ImGui::GetIO().WantCaptureMouse) {
                    pick_face_callback();
                }
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
