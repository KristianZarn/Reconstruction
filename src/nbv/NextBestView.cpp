#include "NextBestView.h"

#include <glm/gtc/matrix_access.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtx/string_cast.hpp>

NextBestView::NextBestView(std::shared_ptr<MVS::Scene> mvs_scene)
        : mvs_scene_(std::move(mvs_scene)) {}

void NextBestView::Initialize() {

    // Compile shaders
    if (!measure_shader_) {
        measure_shader_ = std::make_unique<SourceShader>(measure_vert_source, measure_frag_source);
    }
    if (!faceid_shader_) {
        faceid_shader_ = std::make_unique<SourceShader>(faceid_vert_source, faceid_frag_source);
    }

    // Update meshes
    UpdateFaceIdMesh();
    ppa_ = PixelsPerArea();
    UpdateMeasureMesh(ppa_);

    // Precompute face centers and normals
    mvs_scene_->mesh.ComputeNormalFaces();
    int num_faces = mvs_scene_->mesh.faces.size();
    face_centers_.clear();
    face_centers_.reserve(num_faces);
    face_normals_.clear();
    face_normals_.reserve(num_faces);
    for (int i = 0; i < num_faces; i++) {

        // Face center
        const auto& mvs_face = mvs_scene_->mesh.faces[i];
        const auto& v1 = mvs_scene_->mesh.vertices[mvs_face[0]];
        const auto& v2 = mvs_scene_->mesh.vertices[mvs_face[1]];
        const auto& v3 = mvs_scene_->mesh.vertices[mvs_face[2]];
        face_centers_.emplace_back(glm::vec3(
                (v1.x + v2.x + v3.x) / 3,
                (v1.y + v2.y + v3.y) / 3,
                (v1.z + v2.z + v3.z) / 3));

        // Face normal
        const auto& mvs_normal = mvs_scene_->mesh.faceNormals[i];
        face_normals_.emplace_back(glm::vec3(mvs_normal.x, mvs_normal.y, mvs_normal.z));
    }

    // Precompute visible faces
    visible_faces_.clear();
    for (const auto& mvs_image : mvs_scene_->images) {

        // Camera parameters
        unsigned int image_width = mvs_image.width;
        unsigned int image_height = mvs_image.height;
        double focal_y = mvs_image.camera.K(1,1);

        // Camera view matrix
        const auto& R = mvs_image.camera.R; // world to view (view coordinates)
        const auto& T = mvs_image.camera.C; // world coordinates

        glm::mat3 view_R(R(0,0), R(0,1), R(0,2),
                         -R(1,0), -R(1,1), -R(1,2),
                         -R(2,0), -R(2,1), -R(2,2));
        glm::vec3 view_T(T.x, T.y, T.z);

        glm::mat4 tmp_R = glm::mat4(view_R);
        glm::mat4 tmp_T = glm::translate(glm::mat4(1.0f), view_T);
        glm::mat4 view_matrix = glm::inverse(tmp_T * tmp_R);

        auto tmp = VisibleFaces(
                view_matrix,
                static_cast<int>(image_width / downscale_factor_),
                static_cast<int>(image_height / downscale_factor_),
                focal_y / downscale_factor_);

        visible_faces_.emplace_back(tmp);
    }
}

void NextBestView::UpdateFaceIdMesh() {

    // Convert from OpenMVS to FaceIdMesh
    // Faces are assigned ids starting from 1 (background is zero)
    std::vector<FaceIdMesh::Vertex> vertices;
    unsigned int face_id = 1;
    for (const auto& face_mvs : mvs_scene_->mesh.faces) {
        for (int vert_id = 0; vert_id < 3; vert_id++) {
            const auto& vertex_mvs = mvs_scene_->mesh.vertices[face_mvs[vert_id]];
            FaceIdMesh::Vertex vertex;
            vertex.position = glm::vec3(
                    vertex_mvs.x,
                    vertex_mvs.y,
                    vertex_mvs.z);
            vertex.id = face_id;
            vertices.push_back(vertex);
        }
        face_id++;
    }
    faceid_mesh_ = std::make_unique<FaceIdMesh>(vertices);
}

void NextBestView::UpdateMeasureMesh(const std::vector<double>& measure) {

    // Convert from OpenMVS to MeasureMesh
    assert(mvs_scene_->mesh.faces.size() == measure.size());
    std::vector<MeasureMesh::Vertex> vertices;
    for (int i = 0; i < mvs_scene_->mesh.faces.size(); i++) {
        const auto& face_mvs = mvs_scene_->mesh.faces[i];
        for (int vert_id = 0; vert_id < 3; vert_id++) {
            const auto& vertex_mvs = mvs_scene_->mesh.vertices[face_mvs[vert_id]];
            MeasureMesh::Vertex vertex;
            vertex.position = glm::vec3(
                    vertex_mvs.x,
                    vertex_mvs.y,
                    vertex_mvs.z);
            vertex.measure = static_cast<float>(measure[i]);
            vertices.push_back(vertex);
        }
    }
    measure_mesh_ = std::make_unique<MeasureMesh>(vertices);
}

std::vector<unsigned int>
NextBestView::RenderFaceIdFromCamera(const glm::mat4& view_matrix, int image_width, int image_height, double focal_y) {

    // Framebuffer configuration
    unsigned int framebuffer;
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

    // Color attachment
    unsigned int framebufferTexture;
    glGenTextures(1, &framebufferTexture);
    glBindTexture(GL_TEXTURE_2D, framebufferTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32UI, image_width, image_height, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, NULL);
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
    GLuint color_clear_val[4] = {0, 0, 0, 0};
    glClearBufferuiv(GL_COLOR, 0, color_clear_val);
    GLfloat depth_clear_val = 1.0f;
    glClearBufferfv(GL_DEPTH, 0, &depth_clear_val);

    // Render configuration
    faceid_shader_->use();

    // Projection matrix
    double fov_y = (2 * std::atan(static_cast<double>(image_height) / (2*focal_y)));

    glm::mat4 projection = glm::perspective(
            static_cast<float>(fov_y),
            static_cast<float>(image_width) / static_cast<float>(image_height),
            0.1f, 100.0f);
    faceid_shader_->setMat4("projection", projection);

    // View matrix
    faceid_shader_->setMat4("view", view_matrix);

    // Model matrix
    glm::mat4 model = glm::mat4(1.0f);
    faceid_shader_->setMat4("model", model);

    // Render the mesh
    faceid_mesh_->draw();

    // Read frambuffer texture
    std::vector<unsigned int> render_data(image_width * image_height);
    glReadPixels(0, 0, image_width, image_height, GL_RED_INTEGER, GL_UNSIGNED_INT, render_data.data());

    // Cleanup
    glDeleteFramebuffers(1, &framebuffer);
    glDeleteTextures(1, &framebufferTexture);
    glDeleteRenderbuffers(1, &framebufferDepthStencil);

    return render_data;
}

std::vector<float>
NextBestView::RenderMeasureFromCamera(const glm::mat4& view_matrix, int image_width, int image_height, double focal_y) {

    // Framebuffer configuration
    unsigned int framebuffer;
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

    // Color attachment
    unsigned int framebufferTexture;
    glGenTextures(1, &framebufferTexture);
    glBindTexture(GL_TEXTURE_2D, framebufferTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, image_width, image_height, 0, GL_RED, GL_FLOAT, NULL);
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
    GLuint color_clear_val[4] = {0, 0, 0, 0};
    glClearBufferuiv(GL_COLOR, 0, color_clear_val);
    GLfloat depth_clear_val = 1.0f;
    glClearBufferfv(GL_DEPTH, 0, &depth_clear_val);

    // Render configuration
    measure_shader_->use();

    // Projection matrix
    double fov_y = (2 * std::atan(static_cast<double>(image_height) / (2*focal_y)));

    glm::mat4 projection = glm::perspective(
            static_cast<float>(fov_y),
            static_cast<float>(image_width) / static_cast<float>(image_height),
            0.1f, 100.0f);
    measure_shader_->setMat4("projection", projection);

    // View matrix
    measure_shader_->setMat4("view", view_matrix);

    // Model matrix
    glm::mat4 model = glm::mat4(1.0f);
    measure_shader_->setMat4("model", model);

    // Render the mesh
    measure_mesh_->draw();

    // Read frambuffer texture
    std::vector<float> render_data(image_width * image_height);
    glReadPixels(0, 0, image_width, image_height, GL_RED, GL_FLOAT, render_data.data());

    // Cleanup
    glDeleteFramebuffers(1, &framebuffer);
    glDeleteTextures(1, &framebufferTexture);
    glDeleteRenderbuffers(1, &framebufferDepthStencil);

    return render_data;
}

std::unordered_set<unsigned int>
NextBestView::VisibleFaces(const glm::mat4& view_matrix, int image_width, int image_height, double focal_y) {

    // Returned face_id start at 0
    std::vector<unsigned int> render_data = RenderFaceIdFromCamera(view_matrix, image_width, image_height, focal_y);
    std::unordered_set<unsigned int> visible_faces;
    for (const auto& face_id : render_data) {
        if (face_id > 0) {
            visible_faces.insert(face_id - 1); // subtract 1 to start at 0
        }
    }
    return visible_faces;
}

std::unordered_map<unsigned int, double>
NextBestView::FaceAngles(const std::unordered_set<unsigned int>& faces, const glm::mat4& view_matrix) {

    glm::mat4 camera_world = glm::inverse(view_matrix);
    glm::vec3 camera_front = -glm::column(camera_world, 2);
    std::unordered_map<unsigned int, double> face_angles;

    for (const auto& face_id : faces) {
        auto face_normal = face_normals_[face_id];
        double alpha = glm::angle(face_normal, -camera_front);
        face_angles[face_id] = alpha;
    }
    return face_angles;
}

std::unordered_map<unsigned int, double>
NextBestView::FaceDistances(const std::unordered_set<unsigned int>& faces, const glm::mat4& view_matrix) {

    glm::vec3 camera_center_world = glm::column(glm::inverse(view_matrix), 3);
    std::unordered_map<unsigned int, double> face_distances;

    for (const auto& face_id : faces) {

        // Compute distance
        auto face_center = face_centers_[face_id];
        double distance = glm::distance(face_center, camera_center_world);
        face_distances[face_id] = distance;
    }
    return face_distances;
}

std::unordered_map<int, double> NextBestView::CameraDistances(const glm::mat4& view_matrix) {

    glm::vec3 camera_center_world = glm::column(glm::inverse(view_matrix), 3);
    std::unordered_map<int, double> camera_distances;

    int num_cameras = mvs_scene_->images.size();
    for (int camera_idx = 0; camera_idx < num_cameras; camera_idx++) {

        // Reconstructed camera center
        const auto& T = mvs_scene_->images[camera_idx].camera.C;
        glm::vec3 T_world(T.x, T.y, T.z);

        double distance = glm::distance(T_world, camera_center_world);
        camera_distances[camera_idx] = distance;
    }

    return camera_distances;
}

int NextBestView::ClosestCameraID(const glm::mat4& view_matrix) {

    auto camera_distances = CameraDistances(view_matrix);
    assert(!camera_distances.empty());

    int closest_id = camera_distances.begin()->first;
    double closest_distance = camera_distances.begin()->second;
    for (const auto& [camera_id, distance] : camera_distances) {
        if (distance < closest_distance) {
            closest_distance = distance;
            closest_id = camera_id;
        }
    }
    return closest_id;
}

std::vector<double> NextBestView::PixelsPerArea() {

    assert(faceid_mesh_->getVertices().size() == 3 * mvs_scene_->mesh.faces.size());
    int num_cameras = mvs_scene_->images.size();
    int num_faces = mvs_scene_->mesh.faces.size();

    // Count visible pixels
    std::vector<int> pixel_count(num_faces, 0);
    for (int camera_idx = 0; camera_idx < num_cameras; camera_idx++) {

        // Camera parameters
        unsigned int image_width = mvs_scene_->images[camera_idx].width;
        unsigned int image_height = mvs_scene_->images[camera_idx].height;
        double focal_y = mvs_scene_->images[camera_idx].camera.K(1,1);

        // Camera view matrix
        const auto& R = mvs_scene_->images[camera_idx].camera.R; // world to view (view coordinates)
        const auto& T = mvs_scene_->images[camera_idx].camera.C; // world coordinates

        glm::mat3 view_R(R(0,0), R(0,1), R(0,2),
                         -R(1,0), -R(1,1), -R(1,2),
                         -R(2,0), -R(2,1), -R(2,2));
        glm::vec3 view_T(T.x, T.y, T.z);

        glm::mat4 tmp_R = glm::mat4(view_R);
        glm::mat4 tmp_T = glm::translate(glm::mat4(1.0f), view_T);
        glm::mat4 view_matrix = glm::inverse(tmp_T * tmp_R);

        // Render and count pixels
        auto render_data = RenderFaceIdFromCamera(view_matrix, image_width, image_height, focal_y);
        for (const auto face_id : render_data) {
            if (face_id > 0) {
                pixel_count[face_id - 1]++; // face_id start at 1
            }
        }
    }

    // Compute PPA
    std::vector<double> face_area = FaceArea();
    std::vector<double> ppa(num_faces);
    for (int i = 0; i < num_faces; i++) {
        ppa[i] = sqrt(static_cast<double>(pixel_count[i]) / face_area[i]);
        if (!isfinite(ppa[i])) {
            ppa[i] = 0.0;
        }
    }
    return ppa;
}

std::vector<double> NextBestView::FaceArea() {
    int num_faces = mvs_scene_->mesh.faces.size();
    std::vector<double> fa(num_faces);
    for (int i = 0; i < num_faces; i++) {

        const auto& face = mvs_scene_->mesh.faces[i];
        const auto& a = mvs_scene_->mesh.vertices[face[0]];
        const auto& b = mvs_scene_->mesh.vertices[face[1]];
        const auto& c = mvs_scene_->mesh.vertices[face[2]];

        // Compute face area
        double ab_x = b.x - a.x;
        double ab_y = b.y - a.y;
        double ab_z = b.z - a.z;

        double ac_x = c.x - a.x;
        double ac_y = c.y - a.y;
        double ac_z = c.z - a.z;

        double face_area = 0.5 * sqrt(
                pow(ab_y * ac_z - ab_z * ac_y , 2.0) +
                pow(ab_z * ac_x - ab_x * ac_z , 2.0) +
                pow(ab_x * ac_y - ab_y * ac_x , 2.0));

        fa[i] = face_area;
    }
    return fa;
}

double NextBestView::CostFunctionPosition(
        const glm::mat4& view_matrix, int image_width, int image_height, double focal_y) {

    // Visible faces
    auto visible_faces = VisibleFaces(
            view_matrix,
            static_cast<int>(image_width / downscale_factor_),
            static_cast<int>(image_height / downscale_factor_),
            focal_y / downscale_factor_);

    // assert(!visible_view.empty());
    if (visible_faces.size() < visible_faces_target_) {
        return std::numeric_limits<double>::max();
    }

    // Get face quality
    std::unordered_set<double> face_quality;
    for (const auto& face_id : visible_faces) {
        face_quality.insert(ppa_[face_id]);
    }

    // Mean
    double sum = 0.0;
    for (double val : face_quality) {
        sum += val;
    }
    double mean = sum / face_quality.size();

    // Variance
    double tmp = 0.0;
    for (double val : face_quality) {
        tmp += (val - mean) * (val - mean);
    }
    double sd = sqrt(tmp / (face_quality.size() - 1));

    // Cost value (minimization)
    double alpha = 3.0; // gives bigger importance to standard deviation (should mean closer cameras)
    double cost = mean - alpha * sd;
    std::cout << "Cost T: " << cost << " (M: " << mean << " SD: " << sd << ")" << std::endl;
    return cost;
}

double NextBestView::CostFunctionRotation(
        const glm::mat4& view_matrix, int image_width, int image_height, double focal_y) {

    // Get visible faces
    auto visible_faces = VisibleFaces(
            view_matrix,
            static_cast<int>(image_width / downscale_factor_),
            static_cast<int>(image_height / downscale_factor_),
            focal_y / downscale_factor_);

    // Add all faces if threshold not met
    if (visible_faces.size() < visible_faces_target_) {
        visible_faces.clear();
        for (int face_id = 0; face_id < mvs_scene_->mesh.faces.size(); face_id++) {
            visible_faces.insert(face_id);
        }
    }

    // Compute average angle
    auto face_angles = FaceAngles(visible_faces, view_matrix);
    double angle_sum = 0;
    for (const auto& [face_id, face_angle] : face_angles) {
        angle_sum += face_angle;
    }
    double angle_avg = angle_sum / face_angles.size();

    // Cost value
    std::cout << "Cost R: " << angle_avg << std::endl;
    return angle_avg;
}


