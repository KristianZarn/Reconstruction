#include <sstream>
#include <iomanip>
#include <chrono>

#include <igl/readOFF.h>
#include <igl/png/readPNG.h>
#include <igl/png/writePNG.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include <GLFW/glfw3.h>
#include <glad/glad.h>

#include "webcam.h"
#include "realtime_reconstruction_builder.h"

theia::CameraIntrinsicsPrior GetCalibration() {
    theia::CameraIntrinsicsPrior camera_intrinsics_prior;
    camera_intrinsics_prior.image_width = 960;
    camera_intrinsics_prior.image_height = 720;

    camera_intrinsics_prior.focal_length.is_set = true;
    camera_intrinsics_prior.focal_length.value[0] = 928.0;

    camera_intrinsics_prior.principal_point.is_set = true;
    camera_intrinsics_prior.principal_point.value[0] = 444.0;
    camera_intrinsics_prior.principal_point.value[1] = 350.0;

    camera_intrinsics_prior.aspect_ratio.is_set = true;
    camera_intrinsics_prior.aspect_ratio.value[0] = 1.0;

    camera_intrinsics_prior.skew.is_set = true;
    camera_intrinsics_prior.skew.value[0] = 0.0;

    camera_intrinsics_prior.radial_distortion.is_set = true;
    camera_intrinsics_prior.radial_distortion.value[0] = 0.10837192;
    camera_intrinsics_prior.radial_distortion.value[1] = -0.21332444;

    return camera_intrinsics_prior;
}

int main(int argc, char *argv[]) {

    // Setup camera
    int image_width = 960;
    int image_height = 720;
    Webcam webcam("/dev/video1", image_width, image_height);

    std::string output_path = "../webcam_images/";
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> red(image_width, image_height);
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> green(image_width, image_height);
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> blue(image_width, image_height);
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> alpha =
            Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>::Ones(image_width, image_height);
    alpha = alpha * 255;

    // Setup reconstruction builder
    theia::CameraIntrinsicsPrior intrinsics_prior = GetCalibration();
    theia::RealtimeReconstructionBuilderOptions options;
    options.intrinsics_prior = intrinsics_prior;
    theia::RealtimeReconstructionBuilder reconstruction_builder(options);

    // Initialize the viewer
    igl::opengl::glfw::Viewer viewer;
    viewer.core.is_animating = true;

    // Create texture for camera view
    GLuint textureID;
    viewer.callback_init = [&](igl::opengl::glfw::Viewer viewer1) -> bool {
        glGenTextures(1, &textureID);
        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image_width, image_height, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
        glBindTexture(GL_TEXTURE_2D, 0);
        return false;
    };

    // Attach a menu plugin
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);

    // Reconstruction variables
    int saved_frames_count = 0;

    // Message strings
    std::string camera_message = "Image saved to: ";

    // Draw additional windows
    menu.callback_draw_custom_window = [&]() {
        // Define next window position
        ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(480.0f, 0.0f), ImGuiCond_FirstUseEver);
        // ImGui::SetNextWindowSizeConstraints(ImVec2(300.0f, -1), ImVec2(FLT_MAX, -1));
        ImGui::Begin("Camera", nullptr, ImGuiWindowFlags_NoSavedSettings);

        // Get frame from webcam
        auto frame = webcam.frame();

        // Replace texture with new frame
        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, image_width, image_height, GL_RGB, GL_UNSIGNED_BYTE, frame.data);
        glBindTexture(GL_TEXTURE_2D, 0);

        // Add an image
        float width = ImGui::GetWindowContentRegionWidth();
        float height = width * ((float) image_height / image_width);
        ImGui::Image(reinterpret_cast<GLuint*>(textureID), ImVec2(width, height));

        // Add a button
        if (ImGui::Button("Capture frame", ImVec2(-1,0))) {
            // Store frame on disk
            std::stringstream ss;
            ss << std::setw(3) << std::setfill('0') << std::to_string(saved_frames_count);
            std::string filename = output_path + "frame" + ss.str() + ".png";

            /*Eigen::Map<Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>, 0, Eigen::Stride<Eigen::Dynamic, 3>>
                    red(frame.data, image_width, image_height, Eigen::Stride<Eigen::Dynamic,3>(image_width*3, 3));
            Eigen::Map<Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>, 0, Eigen::Stride<Eigen::Dynamic, 3>>
                    green(frame.data + 1, image_width, image_height, Eigen::Stride<Eigen::Dynamic,3>(image_width*3, 3));
            Eigen::Map<Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>, 0, Eigen::Stride<Eigen::Dynamic, 3>>
                    blue(frame.data + 2, image_width, image_height, Eigen::Stride<Eigen::Dynamic,3>(image_width*3, 3));

            igl::png::writePNG(red, green, blue, alpha, filename);*/

            red = Eigen::Map<Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>, 0, Eigen::Stride<Eigen::Dynamic, 3>>
                    (frame.data, image_width, image_height, Eigen::Stride<Eigen::Dynamic,3>(image_width*3, 3));
            green = Eigen::Map<Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>, 0, Eigen::Stride<Eigen::Dynamic, 3>>
                    (frame.data + 1, image_width, image_height, Eigen::Stride<Eigen::Dynamic,3>(image_width*3, 3));
            blue = Eigen::Map<Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>, 0, Eigen::Stride<Eigen::Dynamic, 3>>
                    (frame.data + 2, image_width, image_height, Eigen::Stride<Eigen::Dynamic,3>(image_width*3, 3));

            igl::png::writePNG(red.rowwise().reverse(), green.rowwise().reverse(), blue.rowwise().reverse(), alpha, filename);
            saved_frames_count++;

            camera_message = "Image saved to: " + filename;
        }

        // Add camera message
        ImGui::Text("%s", camera_message.c_str());

        ImGui::End();
    };

    // Start viewer
    viewer.launch();
}