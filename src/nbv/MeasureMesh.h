#ifndef SANDBOX_NBV_MEASUREMESH_H
#define SANDBOX_NBV_MEASUREMESH_H

#include <vector>

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

class MeasureMesh {
public:
    struct Vertex {
        glm::vec3 position;
        float measure;
    };

    explicit MeasureMesh(std::vector<Vertex> vertices);
    ~MeasureMesh();

    const std::vector<Vertex>& getVertices();
    void draw();

private:
    unsigned int VAO_;
    unsigned int VBO_;
    std::vector<Vertex> vertices_;
};


#endif //SANDBOX_NBV_MEASUREMESH_H
