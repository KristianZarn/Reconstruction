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
    struct Parameters {
        // Flags
        bool active_edit = false;
        bool active_bounding_box = false;

        // Gizmo
        ImGuizmo::OPERATION gizmo_operation = ImGuizmo::TRANSLATE;
        ImGuizmo::MODE gizmo_mode = ImGuizmo::LOCAL;
    };

    EditMeshPlugin(Parameters parameters,
                   const MVS::Scene& mvs_scene);

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

    // Parameters
    Parameters parameters_;

    // Reconstruction
    const MVS::Scene& mvs_scene_;
    MVS::Scene mvs_scene_edit_;

    // Bounding box
    Eigen::MatrixXd bounding_box_vertices_;
    Eigen::Matrix4f bounding_box_gizmo_;

    // Log
    std::ostream& log_stream_ = std::cout;

    // Callback functions
    void start_edit_callback();
    void pose_bounding_box_callback();
    void done_bounding_box_callback();

    // Helpers
    void set_mesh();
    void show_mesh(bool visible);
    void set_bounding_box();
    void transform_bounding_box();
    void show_bounding_box(bool visible);
};


#endif //REALTIME_RECONSTRUCTION_THEIA_EDITMESHPLUGIN_H
