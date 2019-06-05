#ifndef REALTIME_RECONSTRUCTION_VIEWMESHPLUGIN_H
#define REALTIME_RECONSTRUCTION_VIEWMESHPLUGIN_H

#include <string>
#include <ostream>
#include <algorithm>

#include <imgui/imgui.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/ViewerPlugin.h>
#include <OpenMVS/MVS.h>

class ViewMeshPlugin : public igl::opengl::glfw::ViewerPlugin {
public:
    explicit ViewMeshPlugin(std::string project_folder);

    void init(igl::opengl::glfw::Viewer *_viewer) override;
    bool post_draw() override;

    // Callback functions
    void load_mesh_callback();
    void load_quality_callback();
    void smooth_quality_callback();

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
    unsigned int VIEWER_DATA_MESH;

    std::string project_folder_;
    char mesh_name_[128] = "filename.ext";
    char quality_name_[128] = "filename.ext";

    MVS::Mesh mvs_mesh_;
    std::vector<double> quality_data_;

    bool visible_mesh_ = true;
    bool visible_texture_ = false;
    bool visible_wireframe_ = false;

    std::vector<double> face_area();

    void set_mesh(const MVS::Mesh& mvs_mesh);
    void show_mesh(bool visible);
    void set_mesh_color();
    void center_object();
};


#endif //REALTIME_RECONSTRUCTION_VIEWMESHPLUGIN_H
