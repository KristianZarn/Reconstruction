#ifndef REALTIME_RECONSTRUCTION_NEXTBESTVIEWPLUGIN_H
#define REALTIME_RECONSTRUCTION_NEXTBESTVIEWPLUGIN_H

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
#include "nbv/NextBestView.h"

class NextBestViewPlugin : public igl::opengl::glfw::ViewerPlugin{
public:
    explicit NextBestViewPlugin(std::shared_ptr<NextBestView> nbv);

    void init(igl::opengl::glfw::Viewer *_viewer) override;
    bool pre_draw() override;
    bool post_draw() override;

    // Getters
    glm::vec3 get_camera_pos() const;
    glm::vec3 get_camera_rot() const;
    std::vector<glm::mat4> get_initial_best_views() const;

    // Callback functions
    void initialize_callback(const glm::vec3& up = glm::vec3(0.0f, 1.0f, 0.0f));
    void apply_selection_callback();
    void show_clusters_callback();
    void set_nbv_camera_callback(int selected_view);

    void optimize_callback();

    void debug_callback();
    void pick_face_callback();

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
    unsigned int VIEWER_DATA_BOUNDING_BOX;
    unsigned int VIEWER_DATA_NBV_MESH;

    bool camera_visible_ = false;
    bool pose_camera_ = false;
    bool bounding_box_visible_ = false;
    bool pose_bounding_box_ = false;
    bool nbv_mesh_visible_ = false;

    bool auto_apply_selection_ = true;
    bool auto_show_clusters_ = true;

    // Gizmo
    ImGuizmo::OPERATION gizmo_operation_ = ImGuizmo::TRANSLATE;
    ImGuizmo::MODE gizmo_mode_ = ImGuizmo::LOCAL;
    Eigen::Matrix4f camera_gizmo_;
    Eigen::Matrix4f bounding_box_gizmo_;

    // Next best view
    std::shared_ptr<NextBestView> next_best_view_;
    glm::vec3 camera_pos_ = glm::vec3(0.0, 0.0, 0.0);
    glm::vec3 camera_rot_ = glm::vec3(180.0, 0.0, 0.0);

    std::vector<double> pixels_per_area_;
    std::vector<std::pair<std::vector<unsigned int>, double>> clusters_;
    std::vector<int> cluster_id_;
    std::vector<glm::mat4> best_views_init_;
    int selected_view_ = 0;

    // Log
    std::ostream& log_stream_ = std::cout;

    // Helpers
    void show_nbv_camera();
    void show_bounding_box();

    void set_nbv_mesh();
    void set_nbv_mesh_color(const std::vector<double>& face_values);
    void show_nbv_mesh(bool visible);
};


#endif //REALTIME_RECONSTRUCTION_NEXTBESTVIEWPLUGIN_H
