#ifndef REALTIME_RECONSTRUCTION_MENUPLUGIN_H
#define REALTIME_RECONSTRUCTION_MENUPLUGIN_H

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/ViewerPlugin.h>
#include <imgui/imgui.h>

class MenuPlugin : public igl::opengl::glfw::ViewerPlugin {
public:
    void init(igl::opengl::glfw::Viewer *_viewer) override;
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

    void draw_viewer_window();

    void draw_viewer_menu();

    void draw_labels_window();

    void draw_labels(const igl::opengl::ViewerData &data);

    void draw_text(Eigen::Vector3d pos, Eigen::Vector3d normal, const std::string &text);

};


#endif //REALTIME_RECONSTRUCTION_MENUPLUGIN_H
