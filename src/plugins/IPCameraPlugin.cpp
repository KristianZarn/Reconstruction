#include "IPCameraPlugin.h"

#include <sstream>
#include <iomanip>
#include <utility>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw_gl3.h>
#include <igl_stb_image.h>
#include <Eigen/Core>
#include <curl/curl.h>
#include <igl_stb_image.h>

IPCameraPlugin::IPCameraPlugin(int width, int height, std::string images_path)
        : image_width_(width),
          image_height_(height),
          images_path_(std::move(images_path)),
          image_names_(std::make_shared<std::vector<std::string>>()) {}

void IPCameraPlugin::init(igl::opengl::glfw::Viewer *_viewer) {
    ViewerPlugin::init(_viewer);

    // Create texture for camera view (needs glfw context)
    glGenTextures(1, &textureID_);
    glBindTexture(GL_TEXTURE_2D, textureID_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image_width_, image_height_, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    glBindTexture(GL_TEXTURE_2D, 0);
}

bool IPCameraPlugin::post_draw() {
    // Setup window
    float window_width = 480.0f;
    float window_height = 480.0f;
    ImGui::SetNextWindowSize(ImVec2(window_width, window_height), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x - window_width, 0.0f), ImGuiCond_FirstUseEver);

    ImGui::Begin("Camera", nullptr, ImGuiWindowFlags_NoSavedSettings);

    // Add an url text
    ImGui::InputText("URL", url_buffer_, 200);

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

size_t curl_callback(void *ptr, size_t size, size_t nmemb, void* userdata) {
    FILE* stream = (FILE*)userdata;
    if (!stream)
    {
        std::cout << "IP Camera: No stream" << std::endl;
        return 0;
    }

    size_t written = fwrite((FILE*)ptr, size, nmemb, stream);
    return written;
}

void IPCameraPlugin::capture_frame_callback() {
    // Download image and save it to disk
    std::stringstream ss;
    ss << std::setw(3) << std::setfill('0') << std::to_string(saved_frames_count_);
    std::string filename = "frame" + ss.str() + ".jpg";
    std::string fullname = images_path_ + filename;

    FILE* fp = fopen(fullname.c_str(), "wb");

    if (!fp) {
        std::cout << "IP Camera: Failed to create file on the disk." << std::endl;
        return;
    }

    CURL* curlCtx = curl_easy_init();
    curl_easy_setopt(curlCtx, CURLOPT_URL, url_buffer_);
    curl_easy_setopt(curlCtx, CURLOPT_WRITEDATA, fp);
    curl_easy_setopt(curlCtx, CURLOPT_WRITEFUNCTION, curl_callback);
    curl_easy_setopt(curlCtx, CURLOPT_FOLLOWLOCATION, 1);

    CURLcode rc = curl_easy_perform(curlCtx);
    if (rc) {
        std::cout << "IP Camera: Failed to download: " << url_buffer_  << std::endl;
        return;
    }

    long res_code = 0;
    curl_easy_getinfo(curlCtx, CURLINFO_RESPONSE_CODE, &res_code);
    if (!((res_code == 200 || res_code == 201) && rc != CURLE_ABORTED_BY_CALLBACK))
    {
        std::cout << "IP Camera: Response code: " << res_code << std::endl;
        return;
    }

    curl_easy_cleanup(curlCtx);
    fclose(fp);

    image_names_->push_back(filename);
    camera_message_ = "Image saved to: \n" + fullname;
    saved_frames_count_++;

    // Read image from disk
    int x, y, n;
    unsigned char *image_data = igl::stbi_load(fullname.c_str(), &x, &y, &n, 0);

    // Replace texture with new frame
    glBindTexture(GL_TEXTURE_2D, textureID_);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, image_width_, image_height_, GL_RGB, GL_UNSIGNED_BYTE, image_data);
    glBindTexture(GL_TEXTURE_2D, 0);

    free(image_data);
}

std::shared_ptr<std::vector<std::string>> IPCameraPlugin::get_captured_image_names() {
    return image_names_;
}

// Mouse IO
bool IPCameraPlugin::mouse_down(int button, int modifier)
{
    ImGui_ImplGlfwGL3_MouseButtonCallback(viewer->window, button, GLFW_PRESS, modifier);
    return ImGui::GetIO().WantCaptureMouse;
}

bool IPCameraPlugin::mouse_up(int button, int modifier)
{
    return ImGui::GetIO().WantCaptureMouse;
}

bool IPCameraPlugin::mouse_move(int mouse_x, int mouse_y)
{
    return ImGui::GetIO().WantCaptureMouse;
}

bool IPCameraPlugin::mouse_scroll(float delta_y)
{
    ImGui_ImplGlfwGL3_ScrollCallback(viewer->window, 0.f, delta_y);
    return ImGui::GetIO().WantCaptureMouse;
}

// Keyboard IO
bool IPCameraPlugin::key_pressed(unsigned int key, int modifiers)
{
    ImGui_ImplGlfwGL3_CharCallback(nullptr, key);
    if (key == ' ') {
        capture_frame_callback();
        return true;
    }
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool IPCameraPlugin::key_down(int key, int modifiers)
{
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_PRESS, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool IPCameraPlugin::key_up(int key, int modifiers)
{
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_RELEASE, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}