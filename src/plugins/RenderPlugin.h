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
#include "render/RenderStats.h"
#include "reconstruction/RealtimeReconstructionBuilder.h"
#include "plugins/ReconstructionPlugin.h"
#include "plugins/NextBestViewPlugin.h"

class RenderPlugin : public igl::opengl::glfw::ViewerPlugin {
public:
    explicit RenderPlugin(std::string images_path,
                          std::string reconstruction_path,
                          std::string evaluation_folder,
                          Render::CameraIntrinsic camera_intrinsics,
                          std::shared_ptr<Render> render);

    void init(igl::opengl::glfw::Viewer *_viewer) override;
    bool pre_draw() override;
    bool post_draw() override;

    std::shared_ptr<std::vector<std::string>> get_rendered_image_names();

    // Callback functions
    void load_scene_callback();
    void render_callback();
    void save_render_callback();

    // Plugin link callbacks
    void initialize_generated_callback();
    void extend_generated_callback();
    void extend_nbv_callback();
    void extend_manual_callback(int best_view_pick = -1);
    void align_callback();
    void save_aligned_callback();

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
    ReconstructionPlugin* reconstruction_plugin_ = nullptr;
    NextBestViewPlugin* nbv_plugin_ = nullptr;

    // Viewer data
    unsigned int VIEWER_DATA_CAMERA;
    unsigned int VIEWER_DATA_RENDER_CAMERAS;
    unsigned int VIEWER_DATA_RENDER_MESH;

    bool camera_visible_ = false;
    bool pose_camera_ = false;
    bool render_cameras_visible_ = false;
    bool pose_render_cameras_ = false;
    bool render_mesh_visible_ = false;

    // Gizmo
    ImGuizmo::OPERATION gizmo_operation_ = ImGuizmo::TRANSLATE;
    ImGuizmo::MODE gizmo_mode_ = ImGuizmo::LOCAL;
    Eigen::Matrix4f camera_gizmo_;
    Eigen::Matrix4f render_cameras_gizmo_;

    // Input output
    std::string images_folder_;
    int next_image_idx_ = 0;
    std::shared_ptr<std::vector<std::string>> image_names_;

    std::string reconstruction_folder_;
    char mesh_name_[128] = "filename.ext";

    std::string evaluation_folder_;
    bool auto_save_mesh_ = true;

    // Render
    MVS::Mesh mvs_mesh_;
    glm::mat4 align_transform_ = glm::mat4(1.0f);
    bool auto_align_ = true;

    std::shared_ptr<Render> render_;
    glm::mat4 render_pose_world_aligned_;
    Render::CameraIntrinsic camera_intrinsics_;
    std::vector<unsigned char> render_data_;
    RenderStats render_stats_;

    // Generated poses
    int camera_density_ = 20;
    std::vector<glm::mat4> generated_poses_;
    int selected_pose_ = 0;

    // Log
    std::ostream& log_stream_ = std::cout;

    // Helpers
    void show_camera();

    void set_render_mesh(const MVS::Mesh& mvs_mesh);
    void show_render_mesh(bool visible);
    void center_object();

    void update_render_cameras();
    void set_render_cameras();
    void show_render_cameras(bool visible);
};


#endif //REALTIME_RECONSTRUCTION_RENDERPLUGIN_H
