#ifndef SANDBOX_NBV_NEXTBESTVIEW_H
#define SANDBOX_NBV_NEXTBESTVIEW_H

#include <string>
#include <memory>
#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <OpenMVS/MVS.h>
#include <glm/glm.hpp>
#include <optim.hpp>

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

    std::vector<double> FaceArea();
    std::vector<double> PixelsPerArea();
    std::vector<double> LocalFaceCost(const std::vector<double>& quality_measure);
    std::vector<std::vector<unsigned int>> FaceClusters(const std::vector<double>& face_values);
    glm::mat4 BestViewInit(const std::vector<std::vector<unsigned int>>& clusters,
                           const std::vector<double>& face_values);

    std::unordered_set<unsigned int>
    VisibleFaces(const glm::mat4& view_matrix, int image_width, int image_height, double focal_y);
    std::unordered_map<unsigned int, double>
    FaceAngles(const std::unordered_set<unsigned int>& faces, const glm::mat4& view_matrix);
    std::unordered_map<unsigned int, double>
    FaceDistances(const std::unordered_set<unsigned int>& faces, const glm::mat4& view_matrix);

    double CostFunctionPosition(const glm::mat4& view_matrix, int image_width, int image_height, double focal_y);
    double CostFunctionRotation(const glm::mat4& view_matrix, int image_width, int image_height, double focal_y);

private:
    void UpdateFaceIdMesh();
    std::unordered_set<unsigned int> FaceNeighbours(unsigned int face_id, int radius);

public:
    // Reconstruction members
    std::shared_ptr<MVS::Scene> mvs_scene_;

    // LFC parameters
    double max_quality_ = 1000;
    double min_face_area_ = 0.001;
    int face_cost_radius_ = 1;

    // Clustering parameters
    int cluster_max_size_ = 500; // max size of cluster
    float cluster_angle_ = 20; // max angle deviation from mean to be considered part of cluster
    float cluster_min_lfc_ = 1.0;

    // Best view parameters
    float dist_mult_ = 2.0;

    // Cost function parameters
    double downscale_factor_ = 4.0;
    int visible_faces_target_ = 50;
    float alpha_ = 1.0; // importance of standard deviation in cost
    float beta_ = 0.0; // importance of mean in cost

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
