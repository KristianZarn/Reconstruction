#ifndef SANDBOX_NBV_QUALITYMEASURE_H
#define SANDBOX_NBV_QUALITYMEASURE_H

#include <string>
#include <memory>
#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <OpenMVS/MVS.h>

#include "SourceShader.h"
#include "FaceIdMesh.h"

class QualityMeasure {
public:
    explicit QualityMeasure(std::shared_ptr<MVS::Scene> mvs_scene);

    void initialize();
    void updateMesh();

    std::vector<unsigned int> renderFromCamera(int camera_id);
    std::vector<double> groundSamplingDistance();
    std::vector<unsigned int> degreeOfRedundancy();
    std::vector<double> pixelsPerArea();
    std::vector<double> meanPixelsPerArea();
    std::vector<double> faceArea();

private:
    // Reconstruction members
    std::shared_ptr<MVS::Scene> mvs_scene_;

    // Rendering members
    std::unique_ptr<SourceShader> shader_;
    std::unique_ptr<FaceIdMesh> mesh_;

    // Shaders
    const std::string vertex_shader_source =
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

    const std::string fragment_shader_source =
            "#version 400 core\n"
            "out uint out_face_id;\n"
            "flat in uint face_id;\n"
            "void main()\n"
            "{\n"
            "    out_face_id = face_id;\n"
            "}\n";
};


#endif //SANDBOX_NBV_QUALITYMEASURE_H
