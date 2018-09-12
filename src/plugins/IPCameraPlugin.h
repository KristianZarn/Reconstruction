#ifndef REALTIME_RECONSTRUCTION_THEIA_IPCAMERAPLUGIN_H
#define REALTIME_RECONSTRUCTION_THEIA_IPCAMERAPLUGIN_H

#include <glad/glad.h>
#include <imgui/imgui.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/ViewerPlugin.h>

class IPCameraPlugin : public igl::opengl::glfw::ViewerPlugin {
public:
    IPCameraPlugin(int width, int height, std::string images_path);
    void init(igl::opengl::glfw::Viewer *_viewer) override;
    bool post_draw() override;

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
    int image_width_;
    int image_height_;
    std::string images_path_;

    // Camera members
    GLuint textureID_;
    char url_buffer_[128] = "http://192.168.64.104:8080/photo.jpg";
    int saved_frames_count_ = 0;
    std::shared_ptr<std::vector<std::string>> image_names_;

    // GUI
    std::string camera_message_;

    // Callback functions
    void capture_frame_callback();
    // size_t curl_callback(void *ptr, size_t size, size_t nmemb, void* userdata);
};


#endif //REALTIME_RECONSTRUCTION_THEIA_IPCAMERAPLUGIN_H
