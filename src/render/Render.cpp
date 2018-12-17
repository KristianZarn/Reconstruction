#include "Render.h"

#include "stb/stb_image_write.h"

void Render::Initialize(MVS::Scene& mvs_scene) {

    // Load MVS file
    mvs_scene.mesh.ListIncidenteVertices();
    mvs_scene.mesh.ListIncidenteFaces();
    mvs_scene.mesh.ComputeNormalVertices();

    // Fill camera intrinsics
    for (const auto& mvs_image : mvs_scene.images) {
        CameraIntrinsic tmp = {mvs_image.width, mvs_image.height, mvs_image.camera.K(1,1)};
        camera_intrinsics_.push_back(tmp);
    }

    // Fill vertices
    std::vector<Vertex> vertices;
    vertices.reserve(mvs_scene.mesh.faces.size() * 3);
    for (int face_i = 0; face_i < mvs_scene.mesh.faces.size(); face_i++) {
        const auto& mvs_face = mvs_scene.mesh.faces[face_i];

        for (int vert_i = 0; vert_i < 3; vert_i++) {
            const auto& mvs_vert = mvs_scene.mesh.vertices[mvs_face[vert_i]];
            const auto& mvs_normal = mvs_scene.mesh.vertexNormals[mvs_face[vert_i]];
            const auto& mvs_tex_coord = mvs_scene.mesh.faceTexcoords[face_i * 3 + vert_i];

            Vertex vertex;
            vertex.Position = glm::vec3(mvs_vert.x, mvs_vert.y, mvs_vert.z);
            vertex.Normal = glm::vec3(mvs_normal.x, mvs_normal.y, mvs_normal.z);
            vertex.TexCoords = glm::vec2(mvs_tex_coord.x, 1.0 - mvs_tex_coord.y);
            vertices.push_back(vertex);
        }
    }

    // Fill texture
    int width = mvs_scene.mesh.textureDiffuse.width();
    int height = mvs_scene.mesh.textureDiffuse.height();

    unsigned int textureID;
    glGenTextures(1, &textureID);

    glBindTexture(GL_TEXTURE_2D, textureID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_BGR, GL_UNSIGNED_BYTE, mvs_scene.mesh.textureDiffuse.data);
    glGenerateMipmap(GL_TEXTURE_2D);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    mesh_ = std::make_unique<TexturedMesh>(vertices, textureID);
    shader_ = std::make_unique<SourceShader>(vert_source, frag_source);
}


std::shared_ptr<TexturedMesh> Render::GetMesh() const {
    return mesh_;
}

std::shared_ptr<SourceShader> Render::GetShader() const {
    return shader_;
}

Render::CameraIntrinsic Render::GetCameraIntrinsic(int camera_id) const {
    assert(camera_id < camera_intrinsics_.size());
    return camera_intrinsics_[camera_id];
}

std::vector<unsigned int>
Render::RenderFromCamera(const glm::mat4& view_matrix, const CameraIntrinsic& intrinsic) {

    int image_width = intrinsic.image_width;
    int image_height = intrinsic.image_height;
    double focal_y = intrinsic.focal_y;

    // Framebuffer configuration
    unsigned int framebuffer;
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

    // Color attachment
    unsigned int framebufferTexture;
    glGenTextures(1, &framebufferTexture);
    glBindTexture(GL_TEXTURE_2D, framebufferTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image_width, image_height, 0,  GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, framebufferTexture, 0);

    // Depth and stencil attachment
    unsigned int framebufferDepthStencil;
    glGenRenderbuffers(1, &framebufferDepthStencil);
    glBindRenderbuffer(GL_RENDERBUFFER, framebufferDepthStencil);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, image_width, image_height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, framebufferDepthStencil);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
    }

    // Render configuration
    glViewport(0, 0, image_width, image_height);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    // Clear the buffers
    GLfloat color_clear_val[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    glClearBufferfv(GL_COLOR, 0, color_clear_val);
    GLfloat depth_clear_val = 1.0f;
    glClearBufferfv(GL_DEPTH, 0, &depth_clear_val);

    // Render configuration
    shader_->use();

    // Projection matrix
    double fov_y = (2 * std::atan(static_cast<double>(image_height) / (2*focal_y)));

    glm::mat4 projection = glm::perspective(
            static_cast<float>(fov_y),
            static_cast<float>(image_width) / static_cast<float>(image_height),
            0.1f, 100.0f);
    shader_->setMat4("projection", projection);

    // View matrix
    shader_->setMat4("view", view_matrix);

    // Model matrix
    glm::mat4 model = glm::mat4(1.0f);
    shader_->setMat4("model", model);

    // Render the mesh
    mesh_->Draw(*shader_);

    // Read frambuffer texture
    std::vector<unsigned int> render_data(image_width * image_height * 3);
    glReadPixels(0, 0, image_width, image_height,  GL_RGB, GL_UNSIGNED_BYTE, render_data.data());

    // Cleanup
    glDeleteFramebuffers(1, &framebuffer);
    glDeleteTextures(1, &framebufferTexture);
    glDeleteRenderbuffers(1, &framebufferDepthStencil);
    glDisable(GL_CULL_FACE);

    return render_data;
}

void Render::SaveRender(const std::string& filename,
        const CameraIntrinsic& intrinsic,
        const std::vector<unsigned int>& render_data) {

    int image_width = intrinsic.image_width;
    int image_height = intrinsic.image_height;
    int channels = 3;

    stbi_flip_vertically_on_write(true);
    stbi_write_png(filename.c_str(), image_width, image_height,
                   channels, render_data.data(), image_width * channels);
}
