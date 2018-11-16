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

#include "nbv/NextBestView.h"

class NextBestViewPlugin : public igl::opengl::glfw::ViewerPlugin{
public:
    explicit NextBestViewPlugin(std::shared_ptr<NextBestView> nbv);

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

private:
    // Viewer data
    unsigned int VIEWER_DATA_NBV;
    bool camera_visible_ = false;

    // Next best view
    std::shared_ptr<NextBestView> next_best_view_;
    glm::vec3 camera_pos_ = glm::vec3(-1.245946, -2.296472, 0.642335);
    glm::vec3 camera_rot_ = glm::vec3(glm::radians(180.0), 0.0, 0.0);

    // Log
    std::ostream& log_stream_ = std::cout;

    // Callback functions
    void optimize_position_callback();
    void optimize_rotation_callback();

    // Helpers
    void show_camera();
};


#endif //REALTIME_RECONSTRUCTION_NEXTBESTVIEWPLUGIN_H
