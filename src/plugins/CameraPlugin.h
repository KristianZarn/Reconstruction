#ifndef REALTIME_RECONSTRUCTION_CAMERAPLUGIN_H
#define REALTIME_RECONSTRUCTION_CAMERAPLUGIN_H

#include <glad/glad.h>
#include <imgui/imgui.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/ViewerPlugin.h>

#include "webcam/webcam.h"

class CameraPlugin : public igl::opengl::glfw::ViewerPlugin {
public:
    CameraPlugin(std::string device, int width, int height, std::string output_path);
    void init(igl::opengl::glfw::Viewer *_viewer) override;
    bool post_draw() override;

    RGBImage* get_current_frame();
    std::shared_ptr<std::vector<std::string>> get_captured_image_names();

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

    // Camera members
    GLuint textureID_;
    Webcam webcam_;
    RGBImage current_frame_;
    int saved_frames_count_;
    std::shared_ptr<std::vector<std::string>> image_names_;

    // GUI
    std::string camera_message_;

    // Callback functions
    void capture_frame_callback();
};


#endif //REALTIME_RECONSTRUCTION_CAMERAPLUGIN_H
