#ifndef REALTIME_RECONSTRUCTION_THEIA_EDITMESHPLUGIN_H
#define REALTIME_RECONSTRUCTION_THEIA_EDITMESHPLUGIN_H

#include <string>
#include <ostream>

#include <imgui/imgui.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/ViewerPlugin.h>
#include <OpenMVS/MVS.h>

#include "imguizmo/ImGuizmo.h"

class EditMeshPlugin : public igl::opengl::glfw::ViewerPlugin {
public:
    enum class SelectionMode {
        PICK,
        BOX,
        PLANE
    };

    struct Parameters {
        // Gizmo
        ImGuizmo::OPERATION gizmo_operation = ImGuizmo::TRANSLATE;
        ImGuizmo::MODE gizmo_mode = ImGuizmo::LOCAL;

        // Display
        Eigen::RowVector3d default_color = Eigen::RowVector3d(1, 1, 1);
        bool show_mesh = false;
        bool show_texture = false;
        bool show_wireframe = false;

        // Selection
        SelectionMode selection_mode = SelectionMode::PICK;

        // Modify
        int decimate_target = 10000;
        int texture_size = 2048;
        int fill_hole_size = 100;
    };

    explicit EditMeshPlugin(std::shared_ptr<MVS::Scene> mvs_scene);

    void init(igl::opengl::glfw::Viewer *_viewer) override;
    bool pre_draw() override;
    bool post_draw() override;

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
    // Viewer data indices
    unsigned int VIEWER_DATA_MESH_EDIT;
    unsigned int VIEWER_DATA_BOUNDING_BOX;
    unsigned int VIEWER_DATA_PLANE;

    // Parameters
    Parameters parameters_;

    // Reconstruction
    std::shared_ptr<MVS::Scene> mvs_scene_;

    // Selection
    std::unordered_set<int> selected_faces_idx_;

    // Bounding box
    Eigen::MatrixXd bounding_box_vertices_;
    Eigen::Matrix4f bounding_box_gizmo_;

    // Plane
    Eigen::MatrixXd plane_vertices_;
    Eigen::Matrix4f plane_gizmo_;

    // Log
    std::ostream& log_stream_ = std::cout;

    // Callback functions
    void reload_mesh_callback();
    void center_object_callback();
    void select_inside_callback();
    void select_faces_below_callback();
    void invert_selection_callback();
    void remove_selection_callback();
    void fit_plane_callback();
    void decimate_callback();
    void fill_holes_callback();

    // Helpers
    void set_mesh();
    void show_mesh(bool visible);

    void set_bounding_box();
    void transform_bounding_box();
    void show_bounding_box(bool visible);

    void set_plane();
    void transform_plane();
    void show_plane(bool visible);

    void color_selection();
    void gizmo_options();
};

#endif //REALTIME_RECONSTRUCTION_THEIA_EDITMESHPLUGIN_H
