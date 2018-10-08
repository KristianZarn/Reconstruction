#ifndef SANDBOX_NBV_MESH_H
#define SANDBOX_NBV_MESH_H

#include <vector>

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

class OpenGLMesh {
public:
    struct Vertex {
        glm::vec3 position;
        unsigned int id;
    };

    explicit OpenGLMesh(std::vector<Vertex> vertices);
    ~OpenGLMesh();

    const std::vector<Vertex>& getVertices();
    void draw();

private:
    unsigned int VAO_;
    unsigned int VBO_;
    std::vector<Vertex> vertices_;
};


#endif //SANDBOX_NBV_MESH_H
