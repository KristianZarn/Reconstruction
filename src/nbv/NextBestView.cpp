#include "NextBestView.h"

#include <cmath>
#include <algorithm>

#include <glm/gtc/matrix_access.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtx/string_cast.hpp>

NextBestView::NextBestView(std::shared_ptr<MVS::Scene> mvs_scene)
        : mvs_scene_(std::move(mvs_scene)) {}

void NextBestView::Initialize() {

    // Compile shaders
    if (!faceid_shader_) {
        faceid_shader_ = std::make_unique<SourceShader>(faceid_vert_source, faceid_frag_source);
    }

    // Update neighborhood information
    mvs_scene_->mesh.ListIncidenteVertices();
    mvs_scene_->mesh.ListIncidenteFaces();

    // Update mesh
    UpdateFaceIdMesh();
    ppa_ = PixelsPerArea();

    double percentage = TargetPercentage(ppa_);
    if (percentage > percentage_increment_) {
        target_quality_ += target_quality_increment_;
    }

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

    // Set valid faces to all faces
    valid_faces_.clear();
    for (int i = 0; i < num_faces; i++) {
        valid_faces_.insert(i);
    }
}

void NextBestView::SetValidFaces(const std::unordered_set<unsigned int>& valid_faces) {
    valid_faces_ = valid_faces;
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
    glDisable(GL_CULL_FACE);

    return render_data;
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

std::unordered_set<unsigned int> NextBestView::FaceNeighbours(unsigned int face_id, int radius) {
    const auto& mvs_face = mvs_scene_->mesh.faces[face_id];

    // Vertex neighbourhood
    std::unordered_set<unsigned int> neighbourhood_V;
    neighbourhood_V.insert(mvs_face[0]);
    neighbourhood_V.insert(mvs_face[1]);
    neighbourhood_V.insert(mvs_face[2]);

    for (int r = 1; r < radius; r++) {

        // Expand neighbourhood
        std::unordered_set<unsigned int> neighbourhood_exp;
        for (const auto& vertex_id : neighbourhood_V) {
            const auto& adjacent_vertices = mvs_scene_->mesh.vertexVertices[vertex_id];
            for (const auto& adj_vertex_id : adjacent_vertices) {
                neighbourhood_exp.insert(adj_vertex_id);
            }
        }
        neighbourhood_V = neighbourhood_exp;
    }

    // Convert to face neighbourhood
    std::unordered_set<unsigned int> neighbourhood_F;
    for (const auto& vertex_id : neighbourhood_V) {
        const auto& adjacent_faces = mvs_scene_->mesh.vertexFaces[vertex_id];
        for (const auto& adj_face_id : adjacent_faces) {
            neighbourhood_F.insert(adj_face_id);
        }
    }

    return neighbourhood_F;
}

std::vector<std::pair<std::vector<unsigned int>, double>>
NextBestView::FaceClusters(const std::vector<double>& quality_measure) {
    int num_faces = mvs_scene_->mesh.faces.size();
    std::vector<bool> processed(num_faces, false);

    // Clamp quality to max value and discard invalid faces
    for (unsigned int face_i = 0; face_i < num_faces; face_i++) {
        if ((quality_measure[face_i] > target_quality_) || (valid_faces_.find(face_i) == valid_faces_.end())) {
            processed[face_i] = true;
        }
    }

    // Find clusters
    std::vector<std::vector<unsigned int>> clusters;
    for (unsigned int face_i = 0; face_i < num_faces; face_i++) {
        if (!processed[face_i]) {

            // Add current face to a new cluster
            std::vector<unsigned int> queue;
            queue.push_back(face_i);
            glm::vec3 normal_ref = face_normals_[face_i];
            processed[face_i] = true;
            for (int j = 0; j < queue.size(); j++) {

                // Search for a set of valid neighbors
                unsigned int face_j = queue[j];
                std::unordered_set<unsigned int> neighbors = FaceNeighbours(face_j, 1);
                for (const auto& face_n : neighbors) {
                    if (!processed[face_n] && queue.size() < cluster_max_size_) {

                        // Check if valid
                        glm::vec3 normal_n = face_normals_[face_n];
                        double angle = glm::degrees(glm::angle(normal_n, normal_ref));
                        if (angle < cluster_angle_) {

                            // Add to queue
                            queue.push_back(face_n);
                            processed[face_n] = true;

                            // Update reference values
                            normal_ref = normal_ref + (normal_n - normal_ref) / static_cast<float>(queue.size());
                        }
                    }
                }
            }

            // Add new cluster if big enough
            if (queue.size() > cluster_min_size_) {
                clusters.push_back(queue);
            }
        }
    }

    // Compute cluster costs
    std::vector<std::pair<std::vector<unsigned int>, double>> cluster_costs;
    for (const auto& cluster : clusters) {

        // Get faces quality
        std::unordered_set<double> face_quality;
        for (const auto& face_id : cluster) {
            double q = std::min(quality_measure[face_id], static_cast<double>(target_quality_));
            face_quality.insert(q);
        }

        // Cost value
        auto mean_sd = MeanDeviation(face_quality);
        double weight = pow(cluster.size() / static_cast<double>(cluster_max_size_), 2.0);
        double cost = (init_alpha_ * mean_sd.first + init_beta_ * mean_sd.second) * weight;
        cluster_costs.emplace_back(cluster, cost);
    }

    // Sort by cost ascending
    std::sort(cluster_costs.begin(), cluster_costs.end(), [](auto &left, auto &right) {
        return left.second < right.second;
    });

    return cluster_costs;
}

std::vector<glm::mat4>
NextBestView::BestViewInit(const std::vector<std::pair<std::vector<unsigned int>, double>>& cluster_costs,
                           const glm::vec3& up) {

    std::vector<glm::mat4> best_views;

    // Initial views
    std::vector<double> face_area = FaceArea();
    int num_views = std::min(static_cast<int>(cluster_costs.size()), init_num_views_);
    for (int i = 0; i < num_views; i++) {

        // Average center and normal
        const std::vector<unsigned int>& cluster = cluster_costs[i].first;
        glm::vec3 center_sum = glm::vec3(0);
        glm::vec3 normal_sum = glm::vec3(0);
        for (const auto& face_id : cluster) {
            center_sum += face_centers_[face_id];
            normal_sum += face_normals_[face_id] * static_cast<float>(sqrt(face_area[face_id]));
        }
        glm::vec3 cluster_center = center_sum / static_cast<float>(cluster.size());
        glm::vec3 cluster_normal = glm::normalize(normal_sum / static_cast<float>(cluster.size()));

        // Compute camera distance
        double cluster_area = 0.0;
        for (const auto& face_i : cluster) {
            cluster_area += face_area[face_i];
        }
        double camera_distance = cbrt(cluster_area) * dist_alpha_;

        // Generate view matrix
        glm::vec3 eye = cluster_center + cluster_normal * static_cast<float>(camera_distance);
        glm::mat4 view = glm::lookAt(eye, cluster_center, up);
        best_views.push_back(view);
    }

    // Sort by cost function
    unsigned int image_width = mvs_scene_->images.front().width;
    unsigned int image_height = mvs_scene_->images.front().height;
    double focal_y = mvs_scene_->images.front().camera.K(1, 1);

    std::vector<std::pair<int, double>> best_views_cost;
    for (int i = 0; i < num_views; i++) {
        double view_cost = CostFunction2(best_views[i], image_height, focal_y, image_width);
        best_views_cost.emplace_back(i, view_cost);
    }
    std::sort(best_views_cost.begin(), best_views_cost.end(), [](auto &left, auto &right) {
        return left.second < right.second;
    });

    std::vector<glm::mat4> best_views_sorted;
    for (int i = 0; i < num_views; i++) {
        int idx = best_views_cost[i].first;
        best_views_sorted.push_back(best_views[idx]);
    }

    return best_views_sorted;
}

double NextBestView::TargetPercentage(const std::vector<double>& quality_measure) {
    double count = 0;
    for (const auto& face_id : valid_faces_) {
        if (quality_measure[face_id] > target_quality_) {
            count++;
        }
    }
    return (count / valid_faces_.size());
}

std::unordered_set<unsigned int>
NextBestView::VisibleFaces(const glm::mat4& view_matrix, int image_width, int image_height, double focal_y) {

    // Returned face_id start at 0
    std::vector<unsigned int> render_data = RenderFaceIdFromCamera(view_matrix, image_width, image_height, focal_y);
    std::unordered_set<unsigned int> visible_faces;
    for (const auto& tmp : render_data) {
        unsigned int face_id = tmp - 1; // subtract 1 to start at 0
        if (face_id >= 0 && (valid_faces_.find(face_id) != valid_faces_.end())) {
            visible_faces.insert(face_id);
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

std::pair<glm::vec3, glm::vec3> NextBestView::ClusterCenterNormal(
        const std::pair<std::vector<unsigned int>, double>& cluster) {
    glm::vec3 center_sum = glm::vec3(0.0f);
    glm::vec3 normal_sum = glm::vec3(0.0f);
    for (const auto& face_id : cluster.first) {
        center_sum += face_centers_[face_id];
        normal_sum += face_normals_[face_id];
    }
    glm::vec3 cluster_center = center_sum / static_cast<float>(cluster.first.size());
    glm::vec3 cluster_normal = normal_sum / static_cast<float>(cluster.first.size());

    return std::make_pair(cluster_center, glm::normalize(cluster_normal));
}

double NextBestView::CameraToTargetAngle(const glm::mat4& view_matrix, const glm::vec3& target) {
    glm::mat4 camera_world = glm::inverse(view_matrix);
    glm::vec3 camera_center = glm::column(camera_world, 3);
    glm::vec3 camera_front = -glm::column(camera_world, 2);

    glm::vec3 target_direction = glm::normalize(target - camera_center);
    double phi = glm::angle(target_direction, camera_front);
    return phi;
}

std::pair<double, double> NextBestView::MeanDeviation(const std::unordered_set<double>& face_quality) {
    if (face_quality.size() <= 1) {
        return std::make_pair(0.0, 0.0);
    }

    // Mean
    double sum = 0.0;
    for (double val : face_quality) {
        sum += val;
    }
    double mean = sum / face_quality.size();

    // Standard deviation
    double tmp = 0.0;
    for (double val : face_quality) {
        tmp += (val - mean) * (val - mean);
    }
    double sd = sqrt(tmp / (face_quality.size() - 1));

    return std::make_pair(mean, sd);
}

double NextBestView::CostFunction(const glm::mat4& view_matrix, int image_height, double focal_y, int image_width,
                                  const glm::vec3& cluster_center, const glm::vec3& cluster_normal) {
    glm::mat4 camera_world = glm::inverse(view_matrix);
    glm::vec3 camera_center = glm::column(camera_world, 3);
    glm::vec3 camera_front = -glm::column(camera_world, 2);

    // Cluster angle
    double cluster_normal_angle = glm::angle(cluster_normal, -camera_front);
    double cluster_target_angle = CameraToTargetAngle(view_matrix, cluster_center);

    // Cluster distance
    double cluster_distance = glm::distance(camera_center, cluster_center);

    // Visible faces
    auto visible_faces = VisibleFaces(
            view_matrix,
            static_cast<int>(image_width / downscale_factor_),
            static_cast<int>(image_height / downscale_factor_),
            focal_y / downscale_factor_);

    double visibility_weight = 1.0 / (1.0 + exp(-1.0 * (visible_faces.size() - visible_faces_tresh_)));

    // Quality cost
    std::unordered_set<double> face_quality;
    std::vector<double> face_area = FaceArea();
    for (const auto& face_id : visible_faces) {
        double q = ppa_[face_id];
        face_quality.insert(q);
    }
    auto mean_sd = MeanDeviation(face_quality);
    double face_cost = optim_alpha_ * mean_sd.first + optim_beta_ * mean_sd.second;

    // Combined cost (minimization)
    double target_angle_component = (1 - visibility_weight) * (cluster_target_angle) * 100;
    double normal_angle_component = -cos(cluster_normal_angle) * 50;
    double distance_component = pow(cluster_distance - distance_tresh_, 2.0) * 100;
    double face_component = visibility_weight * face_cost;
    double visibility_component = (visible_faces_tresh_ / 5.0) /
            (visible_faces.size() + (visible_faces_tresh_ / 5.0)) * 2000;
    // double cost = target_angle_component + distance_component + face_component;
    double cost = face_component;

    // Debug
    std::cout << "Cost: " << cost
              // << " F: " << visible_faces.size()
              << " M: " << mean_sd.first
              << " SD: " << mean_sd.second
              // << " TAC: " << target_angle_component
              // << " CAC: " << normal_angle_component
              // << " DC: " << distance_component
              // << " VC: " << visibility_component
              << " FC: " << face_component
              << std::endl;

    return cost;
}

double NextBestView::CostFunction2(const glm::mat4& view_matrix, int image_height, double focal_y, int image_width) {

    // TODO: upostevaj meje rekonstrukcije

    // Visible faces
    auto visible_faces = VisibleFaces(
            view_matrix,
            static_cast<int>(image_width / downscale_factor_),
            static_cast<int>(image_height / downscale_factor_),
            focal_y / downscale_factor_);

    // Quality cost
    std::unordered_set<double> face_quality;
    std::vector<double> face_area = FaceArea();
    for (const auto& face_id : visible_faces) {
        double q = ppa_[face_id];
        if (q > target_quality_) {
            face_quality.insert(target_quality_);
        } else {
            face_quality.insert(q);
        }
    }
    auto mean_sd = MeanDeviation(face_quality);
    double face_cost = optim_alpha_ * mean_sd.first + optim_beta_ * mean_sd.second;
    return face_cost;
}
