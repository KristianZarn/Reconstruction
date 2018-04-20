#include "MenuPlugin.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw_gl3.h>
#include <igl/project.h>

void MenuPlugin::init(igl::opengl::glfw::Viewer *_viewer) {
    ViewerPlugin::init(_viewer);
}

bool MenuPlugin::post_draw() {
    // Text labels
    draw_labels_window();
    draw_viewer_window();

    return false;
}

// Mouse IO
bool MenuPlugin::mouse_down(int button, int modifier) {
    ImGui_ImplGlfwGL3_MouseButtonCallback(viewer->window, button, GLFW_PRESS, modifier);
    return ImGui::GetIO().WantCaptureMouse;
}

bool MenuPlugin::mouse_up(int button, int modifier) {
    return ImGui::GetIO().WantCaptureMouse;
}

bool MenuPlugin::mouse_move(int mouse_x, int mouse_y) {
    return ImGui::GetIO().WantCaptureMouse;
}

bool MenuPlugin::mouse_scroll(float delta_y) {
    ImGui_ImplGlfwGL3_ScrollCallback(viewer->window, 0.f, delta_y);
    return ImGui::GetIO().WantCaptureMouse;
}

// Keyboard IO
bool MenuPlugin::key_pressed(unsigned int key, int modifiers) {
    ImGui_ImplGlfwGL3_CharCallback(nullptr, key);
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool MenuPlugin::key_down(int key, int modifiers) {
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_PRESS, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool MenuPlugin::key_up(int key, int modifiers) {
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_RELEASE, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}

void MenuPlugin::draw_viewer_window() {
    float menu_width = 180.f;
    ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiSetCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f), ImGuiSetCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(ImVec2(menu_width, -1.0f), ImVec2(menu_width, -1.0f));
    bool _viewer_menu_visible = true;
    ImGui::Begin(
            "Viewer", &_viewer_menu_visible,
            ImGuiWindowFlags_NoSavedSettings
            | ImGuiWindowFlags_AlwaysAutoResize
    );
    ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.4f);
    draw_viewer_menu();
    ImGui::PopItemWidth();
    ImGui::End();
}

void MenuPlugin::draw_viewer_menu() {
    // Workspace
    if (ImGui::CollapsingHeader("Workspace", ImGuiTreeNodeFlags_DefaultOpen)) {
        float w = ImGui::GetContentRegionAvailWidth();
        float p = ImGui::GetStyle().FramePadding.x;
        if (ImGui::Button("Load##Workspace", ImVec2((w - p) / 2.f, 0))) {
            viewer->load_scene();
        }
        ImGui::SameLine(0, p);
        if (ImGui::Button("Save##Workspace", ImVec2((w - p) / 2.f, 0))) {
            viewer->save_scene();
        }
    }

    // Mesh
    if (ImGui::CollapsingHeader("Mesh", ImGuiTreeNodeFlags_DefaultOpen)) {
        float w = ImGui::GetContentRegionAvailWidth();
        float p = ImGui::GetStyle().FramePadding.x;
        if (ImGui::Button("Load##Mesh", ImVec2((w - p) / 2.f, 0))) {
            viewer->open_dialog_load_mesh();
        }
        ImGui::SameLine(0, p);
        if (ImGui::Button("Save##Mesh", ImVec2((w - p) / 2.f, 0))) {
            viewer->open_dialog_save_mesh();
        }
    }

    // Viewing options
    if (ImGui::CollapsingHeader("Viewing Options", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("Center object", ImVec2(-1, 0))) {
            viewer->core.align_camera_center(viewer->data().V, viewer->data().F);
        }
        if (ImGui::Button("Snap canonical view", ImVec2(-1, 0))) {
            viewer->snap_to_canonical_quaternion();
        }

        // Zoom
        ImGui::PushItemWidth(80);
        ImGui::DragFloat("Zoom", &(viewer->core.camera_zoom), 0.05f, 0.1f, 20.0f);

        // Select rotation type
        static int rotation_type = static_cast<int>(viewer->core.rotation_type);
        static Eigen::Quaternionf trackball_angle = Eigen::Quaternionf::Identity();
        static bool orthographic = true;
        if (ImGui::Combo("Camera Type", &rotation_type, "Trackball\0Two Axes\0002D Mode\0\0")) {
            using RT = igl::opengl::ViewerCore::RotationType;
            auto new_type = static_cast<RT>(rotation_type);
            if (new_type != viewer->core.rotation_type) {
                if (new_type == RT::ROTATION_TYPE_NO_ROTATION) {
                    trackball_angle = viewer->core.trackball_angle;
                    orthographic = viewer->core.orthographic;
                    viewer->core.trackball_angle = Eigen::Quaternionf::Identity();
                    viewer->core.orthographic = true;
                } else if (viewer->core.rotation_type == RT::ROTATION_TYPE_NO_ROTATION) {
                    viewer->core.trackball_angle = trackball_angle;
                    viewer->core.orthographic = orthographic;
                }
                viewer->core.set_rotation_type(new_type);
            }
        }

        // Orthographic view
        ImGui::Checkbox("Orthographic view", &(viewer->core.orthographic));
        ImGui::PopItemWidth();
    }

    // Draw options
    if (ImGui::CollapsingHeader("Draw Options", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Checkbox("Face-based", &(viewer->data().face_based))) {
            viewer->data().set_face_based(viewer->data().face_based);
        }
        ImGui::Checkbox("Show texture", &(viewer->data().show_texture));
        if (ImGui::Checkbox("Invert normals", &(viewer->data().invert_normals))) {
            viewer->data().dirty |= igl::opengl::MeshGL::DIRTY_NORMAL;
        }
        ImGui::Checkbox("Show overlay", &(viewer->data().show_overlay));
        ImGui::Checkbox("Show overlay depth", &(viewer->data().show_overlay_depth));
        ImGui::ColorEdit4("Background", viewer->core.background_color.data(),
                          ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_PickerHueWheel);
        ImGui::ColorEdit4("Line color", viewer->data().line_color.data(),
                          ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_PickerHueWheel);
        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.3f);
        ImGui::DragFloat("Shininess", &(viewer->data().shininess), 0.05f, 0.0f, 100.0f);
        ImGui::PopItemWidth();
    }

    // Overlays
    if (ImGui::CollapsingHeader("Overlays", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("Wireframe", &(viewer->data().show_lines));
        ImGui::Checkbox("Fill", &(viewer->data().show_faces));
        ImGui::Checkbox("Show vertex labels", &(viewer->data().show_vertid));
        ImGui::Checkbox("Show faces labels", &(viewer->data().show_faceid));
    }
}

void MenuPlugin::draw_labels_window() {
    // Text labels
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiSetCond_Always);
    ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize, ImGuiSetCond_Always);
    bool visible = true;
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0, 0, 0, 0));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);
    ImGui::Begin("ViewerLabels", &visible,
                 ImGuiWindowFlags_NoTitleBar
                 | ImGuiWindowFlags_NoResize
                 | ImGuiWindowFlags_NoMove
                 | ImGuiWindowFlags_NoScrollbar
                 | ImGuiWindowFlags_NoScrollWithMouse
                 | ImGuiWindowFlags_NoCollapse
                 | ImGuiWindowFlags_NoSavedSettings
                 | ImGuiWindowFlags_NoInputs);
    for (const auto &data : viewer->data_list) {
        draw_labels(data);
    }
    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar();
}

void MenuPlugin::draw_labels(const igl::opengl::ViewerData &data) {
    if (data.show_vertid) {
        for (int i = 0; i < data.V.rows(); ++i) {
            draw_text(data.V.row(i), data.V_normals.row(i), std::to_string(i));
        }
    }

    if (data.show_faceid) {
        for (int i = 0; i < data.F.rows(); ++i) {
            Eigen::RowVector3d p = Eigen::RowVector3d::Zero();
            for (int j = 0; j < data.F.cols(); ++j) {
                p += data.V.row(data.F(i, j));
            }
            p /= (double) data.F.cols();

            draw_text(p, data.F_normals.row(i), std::to_string(i));
        }
    }

    if (data.labels_positions.rows() > 0) {
        for (int i = 0; i < data.labels_positions.rows(); ++i) {
            draw_text(data.labels_positions.row(i), Eigen::Vector3d(0.0, 0.0, 0.0),
                      data.labels_strings[i]);
        }
    }
}

void MenuPlugin::draw_text(Eigen::Vector3d pos, Eigen::Vector3d normal, const std::string &text) {
    Eigen::Matrix4f view_matrix = viewer->core.view * viewer->core.model;
    pos += normal * 0.005f * viewer->core.object_scale;
    Eigen::Vector3f coord = igl::project(Eigen::Vector3f(pos.cast<float>()),
                                         view_matrix, viewer->core.proj, viewer->core.viewport);

    // Draw text labels slightly bigger than normal text
    ImDrawList *drawList = ImGui::GetWindowDrawList();
    drawList->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 1.2f,
                      ImVec2(coord[0], (viewer->core.viewport[3] - coord[1])),
                      ImGui::GetColorU32(ImVec4(0, 0, 10, 255)),
                      &text[0], &text[0] + text.size());
}