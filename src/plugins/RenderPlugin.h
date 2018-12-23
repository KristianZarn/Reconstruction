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
#include "plugins/ReconstructionPlugin.h"
#include "plugins/NextBestViewPlugin.h"

class RenderPlugin : public igl::opengl::glfw::ViewerPlugin {
public:
    explicit RenderPlugin(std::string images_path,
                          std::string reconstruction_path,
                          std::shared_ptr<Render> render);

    void init(igl::opengl::glfw::Viewer *_viewer) override;
    bool pre_draw() override;
    bool post_draw() override;

    std::shared_ptr<std::vector<std::string>> get_rendered_image_names();

    // Callback functions
    void initialize_scene_callback();
    void render_callback();
    void save_render_callback();
    void set_generated_pose_callback();

    // Plugin link callbacks
    void initialize_reconstruction_callback();
    void extend_reconstruction_callback();

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
    // Plugins
    ReconstructionPlugin* reconstruction_plugin_;
    NextBestViewPlugin* nbv_plugin_;

    // Viewer data
    unsigned int VIEWER_DATA_CAMERA;
    unsigned int VIEWER_DATA_RENDER_CAMERAS;
    unsigned int VIEWER_DATA_RENDER_MESH;

    bool camera_visible_ = false;
    bool pose_camera_ = false;
    bool render_cameras_visible_ = false;
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
    std::vector<unsigned int> render_data_;
    std::vector<glm::mat4> rendered_poses_;
    glm::vec3 camera_pos_ = glm::vec3(0.0, 0.0, 0.0);
    glm::vec3 camera_rot_ = glm::vec3(180.0, 0.0, 0.0);

    // Generated poses
    std::vector<glm::mat4> generated_poses_;
    int selected_pose_ = 0;

    // Log
    std::ostream& log_stream_ = std::cout;

    // Helpers
    void show_camera();
    void set_camera_pose(const glm::mat4& view_matrix);

    void set_render_mesh(const MVS::Scene& mvs_scene);
    void show_render_mesh(bool visible);
    void center_object();

    void set_render_cameras();
    void show_render_cameras(bool visible);
};


#endif //REALTIME_RECONSTRUCTION_RENDERPLUGIN_H
