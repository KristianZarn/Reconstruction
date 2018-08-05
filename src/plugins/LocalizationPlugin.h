#ifndef REALTIME_RECONSTRUCTION_THEIA_LOCALIZATIONPLUGIN_H
#define REALTIME_RECONSTRUCTION_THEIA_LOCALIZATIONPLUGIN_H

#include <string>
#include <ostream>
#include <chrono>

#include <imgui/imgui.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/ViewerPlugin.h>
#include <theia/image/image.h>

#include "webcam/webcam.h"
#include "reconstruction/RealtimeReconstructionBuilder.h"

class LocalizationPlugin : public igl::opengl::glfw::ViewerPlugin {
public:
    LocalizationPlugin(std::string images_path,
                       std::shared_ptr<theia::RealtimeReconstructionBuilder> reconstruction_builder,
                       RGBImage* camera_frame = nullptr);

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
    // Viewer data index
    unsigned int VIEWER_DATA_LOCALIZATION;

    // Input path
    std::string images_path_;

    // Interface
    char filename_buffer_[64] = "frame003.png"; // TODO: change back after debugging
    bool show_camera_ = false;
    bool localization_active_ = false;

    // Reconstruction
    std::shared_ptr<theia::RealtimeReconstructionBuilder> reconstruction_builder_;

    // Localization
    std::future<bool> localization_future_;
    bool localization_success_;
    theia::CalibratedAbsolutePose prev_camera_pose_;

    // Camera
    RGBImage* camera_frame_;
    std::vector<float> camera_frame_data_;
    Eigen::MatrixXd camera_vertices_;
    Eigen::Matrix4d camera_transformation_;

    // Log
    std::ostream& log_stream_ = std::cout;

    // Callback functions
    bool localize_image_callback();
    void toggle_localization_callback(bool active);

    // Helpers
    bool localize_current_frame();
    void set_camera();
    void transform_camera();
    void show_camera(bool visible);
};


#endif //REALTIME_RECONSTRUCTION_THEIA_LOCALIZATIONPLUGIN_H
