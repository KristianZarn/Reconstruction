#ifndef REALTIME_RECONSTRUCTION_RECONSTRUCTIONPLUGIN_H
#define REALTIME_RECONSTRUCTION_RECONSTRUCTIONPLUGIN_H

#include <string>
#include <ostream>

#include <imgui/imgui.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/ViewerPlugin.h>

#include "RealtimeReconstructionBuilder.h"

class ReconstructionPlugin : public igl::opengl::glfw::ViewerPlugin {
public:
    ReconstructionPlugin(theia::RealtimeReconstructionBuilder::Options options,
                         theia::CameraIntrinsicsPrior intrinsics_prior,
                         std::string images_path, std::string reconstruction_path);

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
    // Used for generating full path of images
    int next_image_id_;

    // Input and output paths
    std::string images_path_;
    std::string reconstruction_path_;

    // Reconstruction builder
    std::unique_ptr<theia::RealtimeReconstructionBuilder> reconstruction_builder_;

    // Log
    std::ostringstream log_stream_;

    // Viewer variables
    int point_size_;
    int view_to_delete_;

    // Helper functions
    void refresh_viewer_data();
    void reset_reconstruction();
    std::string image_fullpath(int image_idx);
};


#endif //REALTIME_RECONSTRUCTION_RECONSTRUCTIONPLUGIN_H
