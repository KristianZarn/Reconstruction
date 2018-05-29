#ifndef REALTIME_RECONSTRUCTION_CAMERAPLUGIN_H
#define REALTIME_RECONSTRUCTION_CAMERAPLUGIN_H

#include <glad/glad.h>
#include <imgui/imgui.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/ViewerPlugin.h>

#include "webcam.h"

class CameraPlugin : public igl::opengl::glfw::ViewerPlugin {
public:
    CameraPlugin(std::string device, int width, int height, std::string output_path);
    void init(igl::opengl::glfw::Viewer *_viewer) override;
    bool post_draw() override;
    const std::vector<std::string>& get_captured_image_names();

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
    // Constructor parameters
    std::string device_string_;
    int image_width_;
    int image_height_;
    std::string images_path_;

    // Other members
    Webcam webcam_;
    GLuint textureID_;
    std::string camera_message_;
    int saved_frames_count_;
    std::vector<std::string> image_names_;
};


#endif //REALTIME_RECONSTRUCTION_CAMERAPLUGIN_H
