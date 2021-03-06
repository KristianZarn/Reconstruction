#ifndef RENDER_H
#define RENDER_H

#include <string>
#include <memory>
#include <vector>

#include "TexturedMesh.h"
#include "nbv/SourceShader.h"

class Render {
public:
    struct CameraIntrinsic {
        unsigned int image_width;
        unsigned int image_height;
        double focal_y;
    };

    void Initialize(MVS::Mesh& mvs_mesh);
    std::shared_ptr<TexturedMesh> GetMesh() const;
    std::shared_ptr<SourceShader> GetShader() const;

    std::vector<glm::mat4> RenderPosesDome(const glm::mat4& transform, int camera_density) const;
    std::vector<unsigned char> RenderFromCamera(const glm::mat4& view_matrix, const CameraIntrinsic& intrinsic);
    void SaveRender(const std::string& filename,
            const CameraIntrinsic& intrinsic,
            const std::vector<unsigned char>& render_data) const;

private:
    std::shared_ptr<TexturedMesh> mesh_;
    std::shared_ptr<SourceShader> shader_;

    // Shaders
    const std::string vert_source =
            "#version 330 core\n"
            "layout (location = 0) in vec3 aPos;\n"
            "layout (location = 1) in vec3 aNormal;\n"
            "layout (location = 2) in vec2 aTexCoords;\n"
            "out vec2 TexCoords;\n"
            "uniform mat4 model;\n"
            "uniform mat4 view;\n"
            "uniform mat4 projection;\n"
            "void main()\n"
            "{\n"
            "    TexCoords = aTexCoords;\n"
            "    gl_Position = projection * view * model * vec4(aPos, 1.0);\n"
            "}\n";

    const std::string frag_source =
            "#version 330 core\n"
            "out vec4 FragColor;\n"
            "in vec2 TexCoords;\n"
            "uniform sampler2D texture_diffuse;\n"
            "void main()\n"
            "{\n"
            "    if (gl_FrontFacing) FragColor = texture(texture_diffuse, TexCoords);\n"
            "    else FragColor = vec4(1.0, 1.0, 1.0, 1.0);\n"
            "}\n";
};

#endif //RENDER_H
