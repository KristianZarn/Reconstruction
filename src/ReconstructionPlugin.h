#ifndef REALTIME_RECONSTRUCTION_RECONSTRUCTIONPLUGIN_H
#define REALTIME_RECONSTRUCTION_RECONSTRUCTIONPLUGIN_H

#include <string>

#include <imgui/imgui.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/ViewerPlugin.h>

#include "RealtimeReconstructionBuilder.h"

class ReconstructionPlugin : public igl::opengl::glfw::ViewerPlugin {
public:
    ReconstructionPlugin(theia::RealtimeReconstructionBuilderOptions options, std::string images_path);

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
    std::string images_path_;
    int image_idx_;
    std::unique_ptr<theia::RealtimeReconstructionBuilder> reconstruction_builder_;

    // Viewer variables
    int point_size_;

    // Helper functions
    std::string image_fullpath(int image_idx);
};


#endif //REALTIME_RECONSTRUCTION_RECONSTRUCTIONPLUGIN_H
