#ifndef REALTIME_RECONSTRUCTION_RENDERPLUGIN_H
#define REALTIME_RECONSTRUCTION_RENDERPLUGIN_H

#include <string>
#include <ostream>

#include <imgui/imgui.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/ViewerPlugin.h>
#include <OpenMVS/MVS.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_access.hpp>

#include "imguizmo/ImGuizmo.h"
#include "render/Render.h"

class RenderPlugin : public igl::opengl::glfw::ViewerPlugin {
public:
    explicit RenderPlugin(std::string images_path,
                          std::string reconstruction_path,
                          std::shared_ptr<Render> render);

    void init(igl::opengl::glfw::Viewer *_viewer) override;
    bool pre_draw() override;
    bool post_draw() override;

    std::shared_ptr<std::vector<std::string>> get_rendered_image_names();

    // Mouse IO
    bool mouse_down(int button, int modifier) override;
    bool mouse_up(int button, int modifier) override;
    bool mouse_move(int mouse_x, int mouse_y) override;
    bool mouse_scroll(float delta_y) override;

    // Keyboard IO
    bool key_pressed(unsigned int key, int modifiers) override;
    bool key_down(int key, int modifiers) override;
    bool key_up(int key, int modifiers) override;

private:
    // Viewer data
    unsigned int VIEWER_DATA_CAMERA;
    unsigned int VIEWER_DATA_RENDER_MESH;

    bool camera_visible_ = false;
    bool pose_camera_ = false;
    bool render_mesh_visible_ = false;

    // Gizmo
    ImGuizmo::OPERATION gizmo_operation_ = ImGuizmo::TRANSLATE;
    ImGuizmo::MODE gizmo_mode_ = ImGuizmo::LOCAL;
    Eigen::Matrix4f camera_gizmo_;

    // Image input output
    int next_image_idx_ = 0;
    std::string images_path_;
    std::string reconstruction_path_;
    std::shared_ptr<std::vector<std::string>> image_names_;
    char scene_name_[128] = "filename";

    // Render
    std::shared_ptr<Render> render_;
    glm::vec3 camera_pos_ = glm::vec3(0.0, 0.0, 0.0);
    glm::vec3 camera_rot_ = glm::vec3(180.0, 0.0, 0.0);

    // Log
    std::ostream& log_stream_ = std::cout;

    // Callback functions
    void initialize_scene_callback();
    void render_and_save_callback();

    // Helpers
    void show_camera();
    void set_render_mesh(const MVS::Scene& mvs_scene);
    void show_render_mesh(bool visible);
    void center_object();
};


#endif //REALTIME_RECONSTRUCTION_RENDERPLUGIN_H
