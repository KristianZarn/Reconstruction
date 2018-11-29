#include "IPCameraPlugin.h"

#include <sstream>
#include <iomanip>
#include <utility>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw_gl3.h>
#include <Eigen/Core>
#include <curl/curl.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

IPCameraPlugin::IPCameraPlugin(std::string images_path,
                               std::shared_ptr<theia::RealtimeReconstructionBuilder> reconstruction_builder)
        : images_path_(std::move(images_path)),
          reconstruction_builder_(std::move(reconstruction_builder)),
          image_names_(std::make_shared<std::vector<std::string>>()) {

    curl_ = curl_easy_init();
    curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &curl_stream_);
    curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, &IPCameraPlugin::curl_callback);
    curl_easy_setopt(curl_, CURLOPT_TIMEOUT, 10);
}

IPCameraPlugin::~IPCameraPlugin() {
    curl_easy_cleanup(curl_);
    stbi_image_free(image_data_);
}

void IPCameraPlugin::init(igl::opengl::glfw::Viewer* _viewer) {
    ViewerPlugin::init(_viewer);

    auto intrinsic_prior = reconstruction_builder_->GetOptions().intrinsics_prior;
    int image_width = intrinsic_prior.image_width;
    int image_height = intrinsic_prior.image_height;

    // Create texture for camera view
    glGenTextures(1, &textureID_);
    glBindTexture(GL_TEXTURE_2D, textureID_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image_width, image_height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glBindTexture(GL_TEXTURE_2D, 0);

    // Append mesh for camera
    viewer->append_mesh();
    VIEWER_DATA_LOCALIZATION = static_cast<unsigned int>(viewer->data_list.size() - 1);

    // Initial camera pose
    Eigen::Affine3d scale(Eigen::Scaling(1.0 / 2.0));
    camera_transformation_ = scale * Eigen::Matrix4d::Identity();

    set_camera();
    show_camera(show_camera_);
}

bool IPCameraPlugin::post_draw() {
    // Setup window
    float window_width = 480.0f;
    ImGui::SetNextWindowSize(ImVec2(window_width, 0), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x - window_width, 0.0f), ImGuiCond_FirstUseEver);

    ImGui::Begin("Camera", nullptr, ImGuiWindowFlags_NoSavedSettings);

    // Add an url text
    ImGui::InputText("URL", url_buffer_, 128);

    // Add an image
    auto intrinsic_prior = reconstruction_builder_->GetOptions().intrinsics_prior;
    int image_width = intrinsic_prior.image_width;
    int image_height = intrinsic_prior.image_height;
    float width = ImGui::GetWindowContentRegionWidth();
    float height = width * ((float) image_height / image_width);
    ImGui::Image(reinterpret_cast<GLuint*>(textureID_), ImVec2(width, height));

    // Add a button
    if (ImGui::Button("Capture image [space]", ImVec2(-1, 0))) {
        capture_image_callback();
    }

    if (ImGui::Button("Localize image", ImVec2(-70, 0))) {
        localize_image_callback();
    }
    ImGui::SameLine();
    ImGui::Checkbox("Auto##localize", &auto_localize_);

    if (ImGui::Button("Save image [s]", ImVec2(-70, 0))) {
        save_image_callback();
    }
    ImGui::SameLine();
    ImGui::Checkbox("Auto##save", &auto_save_);

    // Display options
    if (ImGui::Checkbox("Show localization camera", &show_camera_)) {
        show_camera(show_camera_);
    }

    // Camera model
    transform_camera();

    ImGui::End();
    return false;
}

std::shared_ptr<std::vector<std::string>> IPCameraPlugin::get_captured_image_names() {
    return image_names_;
}

void IPCameraPlugin::capture_image_callback() {

    // Download image
    curl_stream_.clear();
    curl_easy_setopt(curl_, CURLOPT_URL, url_buffer_);
    CURLcode res = curl_easy_perform(curl_);
    if (res != CURLE_OK) {
        log_stream_ << "IP Camera: Failed to download: " << url_buffer_ << std::endl;
        return;
    }

    // Decode image
    auto intrinsic_prior = reconstruction_builder_->GetOptions().intrinsics_prior;
    int image_width = intrinsic_prior.image_width;
    int image_height = intrinsic_prior.image_height;
    int channels = 3;

    if (image_data_ != nullptr) {
        stbi_image_free(image_data_);
    }
    image_data_ = stbi_load_from_memory(
            curl_stream_.data(), curl_stream_.size(), &image_width, &image_height, &channels, 0);

    // Replace texture with new frame
    glBindTexture(GL_TEXTURE_2D, textureID_);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, image_width, image_height, GL_RGB, GL_UNSIGNED_BYTE, image_data_);
    glBindTexture(GL_TEXTURE_2D, 0);

    if (auto_localize_) {
        localize_image_callback();
    }
    if (auto_save_) {
        save_image_callback();
    }
}

void IPCameraPlugin::localize_image_callback() {
    log_stream_ << std::endl;

    // Get image dimensions
    auto intrinsic_prior = reconstruction_builder_->GetOptions().intrinsics_prior;
    int image_width = intrinsic_prior.image_width;
    int image_height = intrinsic_prior.image_height;
    int channels = 3;

    // Copy image data to float array
    std::vector<float> image_data_float;
    image_data_float.reserve(image_width * image_height * channels);

    for (int i = 0; i < image_width * image_height * channels; i++) {
        image_data_float[i] = static_cast<float>(image_data_[i]) / 255.0f;
    }

    // Create theia image
    theia::FloatImage image(image_width, image_height, channels, image_data_float.data());

    // Localize image
    bool success = false;
    theia::CalibratedAbsolutePose camera_pose;

    if (prev_localization_success_) {
        log_stream_ << "IP Camera: Localization based on previous pose ... ";
        success = reconstruction_builder_->LocalizeImage(image, prev_camera_pose_, camera_pose);
        log_stream_ << success << std::endl;
    }

    if (!success) {
        log_stream_ << "IP Camera: Global localization ... ";
        success = reconstruction_builder_->LocalizeImage(image, camera_pose);
        log_stream_ << success << std::endl;
    }

    if (success) {
        prev_camera_pose_ = camera_pose;

        // Set camera transformation
        Eigen::Affine3d scale(Eigen::Scaling(1.0 / 2.0));
        Eigen::Affine3d rotate;
        rotate = camera_pose.rotation.transpose();
        Eigen::Affine3d translate(Eigen::Translation3d(camera_pose.position));
        camera_transformation_ = (translate * rotate * scale).matrix();

        // Set camera color
        viewer->selected_data_index = VIEWER_DATA_LOCALIZATION;
        Eigen::Vector3d green_color = Eigen::Vector3d(0, 255, 0) / 255.0;
        viewer->data().uniform_colors(green_color, green_color, green_color);
    } else {
        // Set camera color
        viewer->selected_data_index = VIEWER_DATA_LOCALIZATION;
        Eigen::Vector3d red_color = Eigen::Vector3d(255, 0, 0) / 255.0;
        viewer->data().uniform_colors(red_color, red_color, red_color);
    }
    prev_localization_success_ = success;
    show_camera(true);
}

void IPCameraPlugin::save_image_callback() {
    if (image_data_ != nullptr) {

        // Prepare filename
        std::stringstream ss;
        ss << std::setw(3) << std::setfill('0') << std::to_string(saved_frames_count_);
        std::string filename = "frame" + ss.str() + ".jpg";
        std::string fullname = images_path_ + filename;

        // Write to file
        FILE* fp = fopen(fullname.c_str(), "wb");
        if (!fp) {
            log_stream_ << "IP Camera: Failed to create file on the disk." << std::endl;
            return;
        }

        fwrite(curl_stream_.data(), sizeof(uint8_t), curl_stream_.size(), fp);
        fclose(fp);

        // Succsessful write
        image_names_->push_back(filename);
        saved_frames_count_++;
        log_stream_ << "IP Camera: Image saved to: \n\t" + fullname << std::endl;
    }
}

size_t IPCameraPlugin::curl_callback(char *data, size_t size, size_t nmemb, void *userdata) {
    auto stream = (std::vector<uint8_t>*) userdata;
    size_t length = size * nmemb;
    stream->insert(stream->end(), data, data + length);
    return length;
}

void IPCameraPlugin::set_camera() {

    // Vertices
    Eigen::MatrixXd tmp_V(5, 3);
    tmp_V << 0, 0, 0,
            -0.75, -0.5, 1,
            -0.75, 0.5, 1,
            0.75, 0.5, 1,
            0.75, -0.5, 1;
    camera_vertices_ = tmp_V;

    // Faces
    Eigen::MatrixXi tmp_F(6, 3);
    tmp_F << 0, 1, 2,
            0, 2, 3,
            0, 3, 4,
            0, 4, 1,
            1, 3, 2,
            1, 4, 3;

    // Set viewer data
    viewer->selected_data_index = VIEWER_DATA_LOCALIZATION;
    viewer->data().clear();
    viewer->data().set_mesh(tmp_V, tmp_F);
    viewer->data().set_face_based(true);
    viewer->data().set_colors(Eigen::RowVector3d(128, 128, 128) / 255.0);
}

void IPCameraPlugin::transform_camera() {
    viewer->selected_data_index = VIEWER_DATA_LOCALIZATION;
    if (camera_vertices_.rows() > 0) {
        Eigen::MatrixXd V = camera_vertices_;
        V = (V.rowwise().homogeneous() * camera_transformation_.transpose()).rowwise().hnormalized();
        viewer->data().set_vertices(V);
    }
}

void IPCameraPlugin::show_camera(bool visible) {
    viewer->selected_data_index = VIEWER_DATA_LOCALIZATION;
    viewer->data().show_faces = visible;
    viewer->data().show_lines = visible;
}

// Mouse IO
bool IPCameraPlugin::mouse_down(int button, int modifier) {
    ImGui_ImplGlfwGL3_MouseButtonCallback(viewer->window, button, GLFW_PRESS, modifier);
    return ImGui::GetIO().WantCaptureMouse;
}

bool IPCameraPlugin::mouse_up(int button, int modifier) {
    return ImGui::GetIO().WantCaptureMouse;
}

bool IPCameraPlugin::mouse_move(int mouse_x, int mouse_y) {
    return ImGui::GetIO().WantCaptureMouse;
}

bool IPCameraPlugin::mouse_scroll(float delta_y) {
    ImGui_ImplGlfwGL3_ScrollCallback(viewer->window, 0.f, delta_y);
    return ImGui::GetIO().WantCaptureMouse;
}

// Keyboard IO
bool IPCameraPlugin::key_pressed(unsigned int key, int modifiers) {
    ImGui_ImplGlfwGL3_CharCallback(nullptr, key);
    if (!ImGui::GetIO().WantTextInput) {
        switch (key) {
            case ' ':
            {
                capture_image_callback();
                return true;
            }
            case 's':
            {
                save_image_callback();
                return true;
            }
        }
    }
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool IPCameraPlugin::key_down(int key, int modifiers) {
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_PRESS, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool IPCameraPlugin::key_up(int key, int modifiers) {
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_RELEASE, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}