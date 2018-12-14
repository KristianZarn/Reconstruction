#include "RenderPlugin.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw_gl3.h>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>

void RenderPlugin::init(igl::opengl::glfw::Viewer* _viewer) {
    ViewerPlugin::init(_viewer);

    // Append mesh for camera
    viewer->append_mesh();
    VIEWER_DATA_CAMERA = static_cast<unsigned int>(viewer->data_list.size() - 1);

    // Initial gizmo pose
    camera_gizmo_ = Eigen::Matrix4f::Identity();
}

bool RenderPlugin::pre_draw() {
    ImGuizmo::BeginFrame();
    return false;
}

bool RenderPlugin::post_draw() {
    // Setup window
    float window_width = 350.0f;
    ImGui::SetNextWindowSize(ImVec2(window_width, 0), ImGuiCond_Always);
    ImGui::SetNextWindowPos(ImVec2(700.0f, 0.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin("Render", nullptr, ImGuiWindowFlags_NoSavedSettings);

    // Gizmo setup
    ImGuiIO& io = ImGui::GetIO();
    ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
    Eigen::Affine3f scale_base_zoom(Eigen::Scaling(1.0f / viewer->core.camera_base_zoom));
    Eigen::Affine3f scale_zoom(Eigen::Scaling(1.0f / viewer->core.camera_zoom));
    Eigen::Matrix4f gizmo_view = scale_base_zoom * scale_zoom * viewer->core.view;

    // Camera pose
    if (ImGui::Checkbox("Show render camera", &camera_visible_));
    show_camera();

    ImGui::Checkbox("Pose camera", &pose_camera_);
    ImGui::InputFloat3("Position", glm::value_ptr(camera_pos_));
    ImGui::InputFloat3("Angles", glm::value_ptr(camera_rot_));
    if (pose_camera_) {
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

    ImGui::End();
    return false;
}

void RenderPlugin::show_camera() {
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

// Mouse IO
bool RenderPlugin::mouse_down(int button, int modifier) {
    ImGui_ImplGlfwGL3_MouseButtonCallback(viewer->window, button, GLFW_PRESS, modifier);
    return ImGui::GetIO().WantCaptureMouse;
}

bool RenderPlugin::mouse_up(int button, int modifier) {
    return ImGui::GetIO().WantCaptureMouse;
}

bool RenderPlugin::mouse_move(int mouse_x, int mouse_y) {
    return ImGui::GetIO().WantCaptureMouse;
}

bool RenderPlugin::mouse_scroll(float delta_y) {
    ImGui_ImplGlfwGL3_ScrollCallback(viewer->window, 0.f, delta_y);
    return ImGui::GetIO().WantCaptureMouse;
}

// Keyboard IO
bool RenderPlugin::key_pressed(unsigned int key, int modifiers) {
    ImGui_ImplGlfwGL3_CharCallback(nullptr, key);
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool RenderPlugin::key_down(int key, int modifiers) {
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_PRESS, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool RenderPlugin::key_up(int key, int modifiers) {
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_RELEASE, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}
