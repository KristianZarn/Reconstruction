#include "WebcamPlugin.h"

#include <sstream>
#include <iomanip>
#include <utility>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw_gl3.h>
#include <igl_stb_image.h>
#include <Eigen/Core>

WebcamPlugin::WebcamPlugin(std::string device, int width, int height, std::string images_path)
        : device_string_(std::move(device)),
          image_width_(width),
          image_height_(height),
          images_path_(std::move(images_path)),
          webcam_(device_string_, image_width_, image_height_),
          saved_frames_count_(0),
          image_names_(std::make_shared<std::vector<std::string>>()) {}

void WebcamPlugin::init(igl::opengl::glfw::Viewer *_viewer) {
    ViewerPlugin::init(_viewer);

    // Create texture for camera view (needs glfw context)
    glGenTextures(1, &textureID_);
    glBindTexture(GL_TEXTURE_2D, textureID_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image_width_, image_height_, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    glBindTexture(GL_TEXTURE_2D, 0);
}

bool WebcamPlugin::post_draw() {
    // Setup window
    float window_width = 480.0f;
    float window_height = 450.0f;
    ImGui::SetNextWindowSize(ImVec2(window_width, window_height), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x - window_width, 0.0f), ImGuiCond_FirstUseEver);

    ImGui::Begin("Camera", nullptr, ImGuiWindowFlags_NoSavedSettings);

    // Get frame from webcam
    current_frame_ = webcam_.frame();

    // Replace texture with new frame
    glBindTexture(GL_TEXTURE_2D, textureID_);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, image_width_, image_height_, GL_RGB, GL_UNSIGNED_BYTE, current_frame_.data);
    glBindTexture(GL_TEXTURE_2D, 0);

    // Add an image
    float width = ImGui::GetWindowContentRegionWidth();
    float height = width * ((float) image_height_ / image_width_);
    ImGui::Image(reinterpret_cast<GLuint*>(textureID_), ImVec2(width, height));

    // Add a button
    if (ImGui::Button("Capture frame [space]", ImVec2(-1, 0))) {
        capture_frame_callback();
    }

    // Add camera message
    ImGui::TextWrapped("%s", camera_message_.c_str());

    ImGui::End();
    return false;
}

void WebcamPlugin::capture_frame_callback() {
    // Get frame from webcam
    auto frame = webcam_.frame();

    // Store frame on disk
    std::stringstream ss;
    ss << std::setw(3) << std::setfill('0') << std::to_string(saved_frames_count_);
    std::string filename = "frame" + ss.str() + ".png";
    std::string fullname = images_path_ + filename;

    int channels = 3;
    igl::stbi_write_png(fullname.c_str(), image_width_, image_height_, channels, frame.data, image_width_ * channels);

    image_names_->push_back(filename);
    camera_message_ = "Image saved to: \n" + fullname;
    saved_frames_count_++;
}

RGBImage* WebcamPlugin::get_current_frame() {
    return &current_frame_;
}

std::shared_ptr<std::vector<std::string>> WebcamPlugin::get_captured_image_names() {
    return image_names_;
}


// Mouse IO
bool WebcamPlugin::mouse_down(int button, int modifier)
{
    ImGui_ImplGlfwGL3_MouseButtonCallback(viewer->window, button, GLFW_PRESS, modifier);
    return ImGui::GetIO().WantCaptureMouse;
}

bool WebcamPlugin::mouse_up(int button, int modifier)
{
    return ImGui::GetIO().WantCaptureMouse;
}

bool WebcamPlugin::mouse_move(int mouse_x, int mouse_y)
{
    return ImGui::GetIO().WantCaptureMouse;
}

bool WebcamPlugin::mouse_scroll(float delta_y)
{
    ImGui_ImplGlfwGL3_ScrollCallback(viewer->window, 0.f, delta_y);
    return ImGui::GetIO().WantCaptureMouse;
}

// Keyboard IO
bool WebcamPlugin::key_pressed(unsigned int key, int modifiers)
{
    ImGui_ImplGlfwGL3_CharCallback(nullptr, key);
    if (key == ' ') {
        capture_frame_callback();
        return true;
    }
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool WebcamPlugin::key_down(int key, int modifiers)
{
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_PRESS, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool WebcamPlugin::key_up(int key, int modifiers)
{
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_RELEASE, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}