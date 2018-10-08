#include "NextBestView.h"

#include <cmath>

NextBestView::NextBestView(std::shared_ptr<MVS::Scene> mvs_scene)
        : mvs_scene_(std::move(mvs_scene)) {

    assert(!mvs_scene_->images.empty());
    image_width_ = mvs_scene_->images.front().width;
    image_height_ = mvs_scene_->images.front().height;

    // Initialize GLFW context
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);

    window_ = glfwCreateWindow(image_width_, image_height_, "", NULL, NULL);
    if (!window_) {
        std::cout << "Failed to create GLFW window" << std::endl;
        exit(EXIT_FAILURE);
    }
    glfwMakeContextCurrent(window_);

    // Load all OpenGL function pointers
    if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
        std::cout << "Failed to initialize GLAD" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Framebuffer configuration
    glGenFramebuffers(1, &framebuffer_);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);

    // Color attachment
    glGenTextures(1, &framebufferTexture_);
    glBindTexture(GL_TEXTURE_2D, framebufferTexture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32UI, image_width_, image_height_, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, framebufferTexture_, 0);

    // Depth and stencil attachment
    glGenRenderbuffers(1, &framebufferDepthStencil_);
    glBindRenderbuffer(GL_RENDERBUFFER, framebufferDepthStencil_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, image_width_, image_height_);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, framebufferDepthStencil_);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
    }

    // Prepare shader
    shader_ = std::make_unique<SourceShader>(vertex_shader_source, fragment_shader_source);

    // Convert from OpenMVS to OpenGLMesh
    std::vector<OpenGLMesh::Vertex> vertices;
    unsigned int face_id = 1;
    for (const auto& face_mvs : mvs_scene_->mesh.faces) {
        for (int vert_id = 0; vert_id < 3; vert_id++) {
            const auto& vertex_mvs = mvs_scene_->mesh.vertices[face_mvs[vert_id]];
            OpenGLMesh::Vertex vertex;
            vertex.position = glm::vec3(
                    vertex_mvs.x,
                    vertex_mvs.y,
                    vertex_mvs.z);
            vertex.id = face_id;
            vertices.push_back(vertex);
        }
        face_id++;
    }
    mesh_ = std::make_unique<OpenGLMesh>(vertices);
}

NextBestView::~NextBestView() {
    glDeleteFramebuffers(1, &framebuffer_);
    glDeleteTextures(1, &framebufferTexture_);
    glDeleteRenderbuffers(1, &framebufferDepthStencil_);
    glfwDestroyWindow(window_);
}

int NextBestView::getImageWidth() {
    return image_width_;
}

int NextBestView::getImageHeight() {
    return image_height_;
}

std::vector<unsigned int> NextBestView::renderFromCamera(int camera_id) {

    // Render configuration
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
    glViewport(0, 0, image_width_, image_height_);
    glEnable(GL_DEPTH_TEST);

    // Clear the buffers
    GLuint color_clear_val[4] = {0, 0, 0, 0};
    glClearBufferuiv(GL_COLOR, 0, color_clear_val);
    GLfloat depth_clear_val = 1.0f;
    glClearBufferfv(GL_DEPTH, 0, &depth_clear_val);

    // Render configuration
    shader_->use();

    // Projection matrix
    unsigned int height = mvs_scene_->images[camera_id].height;
    unsigned int width = mvs_scene_->images[camera_id].width;
    double focal_y = mvs_scene_->images[camera_id].camera.K(1,1);
    double fov_y = (2 * std::atan(static_cast<double>(height) / (2*focal_y)));

    glm::mat4 projection = glm::perspective(
            static_cast<float>(fov_y),
            static_cast<float>(width) / static_cast<float>(height),
            0.1f, 100.0f);
    shader_->setMat4("projection", projection);

    // View matrix
    const auto& R = mvs_scene_->images[camera_id].camera.R;
    const auto& T = mvs_scene_->images[camera_id].camera.C;

    glm::mat3 view_R(R(0,0), R(0,1), R(0,2),
                     -R(1,0), -R(1,1), -R(1,2),
                     -R(2,0), -R(2,1), -R(2,2));
    glm::vec3 view_T(T.x, T.y, T.z);

    glm::mat4 tmp_R = glm::mat4(view_R);
    glm::mat4 tmp_T = glm::translate(glm::mat4(1.0f), view_T);
    glm::mat4 view = glm::inverse(tmp_T * tmp_R);
    shader_->setMat4("view", view);

    // Model matrix
    glm::mat4 model = glm::mat4(1.0f);
    shader_->setMat4("model", model);

    // Render the mesh
    mesh_->draw();

    // Read frambuffer texture
    std::vector<unsigned int> render_data(image_width_ * image_height_);
    glReadPixels(0, 0, image_width_, image_height_, GL_RED_INTEGER, GL_UNSIGNED_INT, render_data.data());

    return render_data;
}

std::vector<double> NextBestView::groundSamplingDistance() {
    int num_cameras = mvs_scene_->images.size();
    int num_faces = mvs_scene_->mesh.faces.size();

    // Count visible pixels
    std::vector<int> pixel_count(num_faces, 0);
    for (int camera_idx = 0; camera_idx < num_cameras; camera_idx++) {

        // Pixel count for current camera
        std::vector<int> pixel_count_tmp(num_faces, 0);
        std::vector<unsigned int> render_data = renderFromCamera(camera_idx);
        for (const auto face_id : render_data) {
            if (face_id > 0) {
                pixel_count_tmp[face_id-1]++; // face_id start at 1
            }
        }

        // Replace if number of visible pixels is bigger in current camera
        for (int i = 0; i < pixel_count.size(); i++) {
            if (pixel_count_tmp[i] > pixel_count[i]) {
                pixel_count[i] = pixel_count_tmp[i];
            }
        }
    }

    // Compute GSD
    auto vertices = mesh_->getVertices();
    assert(vertices.size() == 3 * mvs_scene_->mesh.faces.size());

    std::vector<double> gsd(num_faces);
    for (int i = 0; i < num_faces; i++) {

        // Compute triangle area
        const auto& a = vertices[i*3 + 0].position;
        const auto& b = vertices[i*3 + 1].position;
        const auto& c = vertices[i*3 + 2].position;

        glm::vec3 ab = b - a;
        glm::vec3 ac = c - a;

        double face_area = 0.5 * sqrt(
                pow(ab[1] * ac[2] - ab[2] * ac[1] , 2.0) +
                pow(ab[2] * ac[0] - ab[0] * ac[2] , 2.0) +
                pow(ab[0] * ac[1] - ab[1] * ac[0] , 2.0));

        // Compute GSD measure
        gsd[i] = sqrt(face_area / static_cast<double>(pixel_count[i]));
        if (!isfinite(gsd[i])) {
            gsd[i] = 0.0;
        }
    }
    return gsd;
}

std::vector<unsigned int> NextBestView::degreeOfRedundancy() {
    int num_cameras = mvs_scene_->images.size();
    int num_faces = mvs_scene_->mesh.faces.size();

    // Count number of times a face is visible
    std::vector<unsigned int> visibility_count(num_faces, 0);
    for (int camera_idx = 0; camera_idx < num_cameras; camera_idx++) {

        // Pixel count for current camera
        std::vector<int> pixel_count(num_faces, 0);
        std::vector<unsigned int> render_data = renderFromCamera(camera_idx);
        for (const auto face_id : render_data) {
            if (face_id > 0) {
                pixel_count[face_id-1]++; // face_id start at 1
            }
        }

        // Increment visibility if pixel count positive
        for (int i = 0; i < pixel_count.size(); i++) {
            if (pixel_count[i] > 0) {
                visibility_count[i]++;
            }
        }
    }
    return visibility_count;
}

std::vector<double> NextBestView::pixelsPerArea() {
    int num_cameras = mvs_scene_->images.size();
    int num_faces = mvs_scene_->mesh.faces.size();

    // Count visible pixels
    std::vector<int> pixel_count(num_faces, 0);
    for (int camera_idx = 0; camera_idx < num_cameras; camera_idx++) {
        std::vector<unsigned int> render_data = renderFromCamera(camera_idx);
        for (const auto face_id : render_data) {
            if (face_id > 0) {
                pixel_count[face_id-1]++; // face_id start at 1
            }
        }
    }

    // Compute PPA
    auto vertices = mesh_->getVertices();
    assert(vertices.size() == 3 * mvs_scene_->mesh.faces.size());

    std::vector<double> ppa(num_faces);
    for (int i = 0; i < num_faces; i++) {

        // Compute face area
        const auto& a = vertices[i*3 + 0].position;
        const auto& b = vertices[i*3 + 1].position;
        const auto& c = vertices[i*3 + 2].position;

        glm::vec3 ab = b - a;
        glm::vec3 ac = c - a;

        double face_area = 0.5 * sqrt(
                pow(ab[1] * ac[2] - ab[2] * ac[1] , 2.0) +
                pow(ab[2] * ac[0] - ab[0] * ac[2] , 2.0) +
                pow(ab[0] * ac[1] - ab[1] * ac[0] , 2.0));

        // Compute PPA measure
        ppa[i] = static_cast<double>(pixel_count[i]) / face_area;
        if (!isfinite(ppa[i])) {
            ppa[i] = 0.0;
        }
    }
    return ppa;
}

std::vector<double> NextBestView::faceArea() {
    int num_faces = mvs_scene_->mesh.faces.size();
    auto vertices = mesh_->getVertices();
    assert(vertices.size() == 3 * mvs_scene_->mesh.faces.size());

    std::vector<double> fa(num_faces);
    for (int i = 0; i < num_faces; i++) {

        // Compute face area
        const auto& a = vertices[i*3 + 0].position;
        const auto& b = vertices[i*3 + 1].position;
        const auto& c = vertices[i*3 + 2].position;

        glm::vec3 ab = b - a;
        glm::vec3 ac = c - a;

        double face_area = 0.5 * sqrt(
                pow(ab[1] * ac[2] - ab[2] * ac[1] , 2.0) +
                pow(ab[2] * ac[0] - ab[0] * ac[2] , 2.0) +
                pow(ab[0] * ac[1] - ab[1] * ac[0] , 2.0));

        fa[i] = face_area;
    }
    return fa;
}
