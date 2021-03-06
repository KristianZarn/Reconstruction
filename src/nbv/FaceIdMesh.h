#ifndef SANDBOX_NBV_FACEIDMESH_H
#define SANDBOX_NBV_FACEIDMESH_H

#include <vector>

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

class FaceIdMesh {
public:
    struct Vertex {
        glm::vec3 position;
        unsigned int id;
    };

    explicit FaceIdMesh(std::vector<Vertex> vertices);
    ~FaceIdMesh();

    const std::vector<Vertex>& getVertices();
    void draw();

private:
    unsigned int VAO_;
    unsigned int VBO_;
    std::vector<Vertex> vertices_;
};


#endif //SANDBOX_NBV_FACEIDMESH_H
