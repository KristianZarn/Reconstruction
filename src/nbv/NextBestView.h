#ifndef SANDBOX_NBV_NEXTBESTVIEW_H
#define SANDBOX_NBV_NEXTBESTVIEW_H

#include <string>
#include <memory>
#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <OpenMVS/MVS.h>
#include <glm/glm.hpp>

#include "SourceShader.h"
#include "MeasureMesh.h"
#include "FaceIdMesh.h"

class NextBestView {
public:
    explicit NextBestView(std::shared_ptr<MVS::Scene> mvs_scene);
    void Initialize();
    void SetValidFaces(const std::unordered_set<unsigned int>& valid_faces);

    std::vector<unsigned int>
    RenderFaceIdFromCamera(const glm::mat4& view_matrix, int image_width, int image_height, double focal_y);

    // Initialization functions
    std::vector<double> FaceArea();
    std::vector<double> PixelsPerArea();
    std::vector<std::pair<std::vector<unsigned int>, double>> FaceClusters(const std::vector<double>& quality_measure);
    std::vector<glm::mat4> BestViewInit(const std::vector<std::pair<std::vector<unsigned int>, double>>& cluster_costs,
                                        const glm::vec3& up = glm::vec3(0.0f, 1.0f, 0.0f));

    // Helpers
    double TargetPercentage(const std::vector<double>& quality_measure);
    std::unordered_set<unsigned int>
    VisibleFaces(const glm::mat4& view_matrix, int image_width, int image_height, double focal_y);
    std::unordered_map<unsigned int, double>
    FaceAngles(const std::unordered_set<unsigned int>& faces, const glm::mat4& view_matrix);
    std::unordered_map<unsigned int, double>
    FaceDistances(const std::unordered_set<unsigned int>& faces, const glm::mat4& view_matrix);
    std::pair<glm::vec3, glm::vec3> ClusterCenterNormal(const std::pair<std::vector<unsigned int>, double>& cluster);
    double CameraToTargetAngle(const glm::mat4& view_matrix, const glm::vec3& target);

    // Optimization functions
    std::pair<double, double> MeanDeviation(const std::unordered_set<double>& face_quality);
    double CostFunction(const glm::mat4& view_matrix, int image_height, double focal_y, int image_width,
                        const glm::vec3& cluster_center, const glm::vec3& cluster_normal);
    double CostFunction2(const glm::mat4& view_matrix, int image_height, double focal_y, int image_width);

private:
    void UpdateFaceIdMesh();
    std::unordered_set<unsigned int> FaceNeighbours(unsigned int face_id, int radius);

public:
    // Reconstruction members
    std::shared_ptr<MVS::Scene> mvs_scene_;

    // General parameters
    float max_quality_ = 1500;

    // Clustering parameters
    int cluster_min_size_ = 100; // min size of cluster
    int cluster_max_size_ = 300; // max size of cluster
    float cluster_angle_ = 100; // max angle deviation from mean to be considered part of cluster

    // Best view parameters
    int init_num_views_ = 10; // max number of returned views
    float init_alpha_ = 1.0f; // mean multiplier for initialization
    float init_beta_ = -5.0f; // standard deviation multiplier for initialization
    float dist_alpha_ = 4.0f;

    // Cost function parameters
    double downscale_factor_ = 2.0;
    float visible_faces_tresh_ = 500.0f;
    float distance_tresh_ = 4.0f;
    float optim_alpha_ = 1.0f; // mean multiplier for optimization
    float optim_beta_ = -5.0f; // standard deviation multiplier for optimization

private:
    // Rendering members
    std::unique_ptr<SourceShader> faceid_shader_;
    std::unique_ptr<FaceIdMesh> faceid_mesh_;
    std::unordered_set<unsigned int> valid_faces_;

    // Speedup variables
    std::vector<glm::vec3> face_centers_;
    std::vector<glm::vec3> face_normals_;

    // Optimization
    std::vector<double> ppa_;

    // Shaders
    const std::string faceid_vert_source =
            "#version 400 core\n"
            "layout (location = 0) in vec3 in_position;\n"
            "layout (location = 1) in uint in_face_id;\n"
            "flat out uint face_id;\n"
            "uniform mat4 model;\n"
            "uniform mat4 view;\n"
            "uniform mat4 projection;\n"
            "void main()\n"
            "{\n"
            "    face_id = in_face_id;\n"
            "    gl_Position = projection * view * model * vec4(in_position, 1.0);\n"
            "}\n";

    const std::string faceid_frag_source =
            "#version 400 core\n"
            "out uint out_face_id;\n"
            "flat in uint face_id;\n"
            "void main()\n"
            "{\n"
            "    out_face_id = face_id;\n"
            "}\n";
};

#endif //SANDBOX_NBV_NEXTBESTVIEW_H
