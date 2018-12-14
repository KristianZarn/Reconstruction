#ifndef TEXTURED_MESH_H
#define TEXTURED_MESH_H

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <OpenMVS/MVS.h>

#include "nbv/SourceShader.h"

struct Vertex {
    // position
    glm::vec3 Position;
    // normal
    glm::vec3 Normal;
    // texCoords
    glm::vec2 TexCoords;
};

class TexturedMesh {
public:
    // Members
    std::vector<Vertex> vertices;
    unsigned int texture_id;
    unsigned int VAO, VBO;

    // Functions
    TexturedMesh(std::vector<Vertex> vertices, unsigned int texture_id) {
        this->vertices = vertices;
        this->texture_id = texture_id;

        // now that we have all the required data, set the vertex buffers and its attribute pointers.
        SetupMesh();
    }

    ~TexturedMesh() {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
        glDeleteTextures(1, &texture_id);
    }

    // render the mesh
    void Draw(const SourceShader& shader) {
        // bind texture
        glActiveTexture(GL_TEXTURE0);
        // now set the sampler to the correct texture unit
        glUniform1i(glGetUniformLocation(shader.ID, "texture_diffuse"), 0);
        // and finally bind the texture
        glBindTexture(GL_TEXTURE_2D, texture_id);

        // draw mesh
        glBindVertexArray(VAO);
        // glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vertices.size()));
        glBindVertexArray(0);

        // always good practice to set everything back to defaults once configured.
        glActiveTexture(GL_TEXTURE0);
    }

private:

    // initializes all the buffer objects/arrays
    void SetupMesh() {
        // create buffers/arrays
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);

        glBindVertexArray(VAO);
        // load data into vertex buffer
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);

        // set the vertex attribute pointers
        // vertex Positions
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*) 0);
        // vertex normals
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*) offsetof(Vertex, Normal));
        // vertex texture coords
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*) offsetof(Vertex, TexCoords));

        glBindVertexArray(0);
    }
};

#endif //TEXTURED_MESH_H
