#include "EquilibriumMesh.h"

#include <algorithm>
#include <cstddef>
#include <vector>

namespace
{
    struct Vertex
    {
        float x;
        float y;
        float r;
        float g;
        float b;
    };
}

EquilibriumMesh::~EquilibriumMesh()
{
    Destroy();
}

bool EquilibriumMesh::Create()
{
    Destroy();

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);

    if (vao_ == 0 || vbo_ == 0) {
        Destroy();
        return false;
    }

    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);

    glVertexAttribPointer(
        0,
        2,
        GL_FLOAT,
        GL_FALSE,
        sizeof(Vertex),
        reinterpret_cast<void*>(offsetof(Vertex, x))
    );
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(
        1,
        3,
        GL_FLOAT,
        GL_FALSE,
        sizeof(Vertex),
        reinterpret_cast<void*>(offsetof(Vertex, r))
    );
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    return true;
}

bool EquilibriumMesh::SetPoints(
    const std::vector<Vec2f>& points,
    float red,
    float green,
    float blue
)
{
    if (vbo_ == 0) {
        return false;
    }

    red = std::clamp(red, 0.0f, 1.0f);
    green = std::clamp(green, 0.0f, 1.0f);
    blue = std::clamp(blue, 0.0f, 1.0f);

    std::vector<Vertex> vertices;
    vertices.reserve(points.size());

    for (const Vec2f& point : points) {
        vertices.push_back(Vertex{
            point.x,
            point.y,
            red,
            green,
            blue
            });
    }

    glBindBuffer(GL_ARRAY_BUFFER, vbo_);

    glBufferData(
        GL_ARRAY_BUFFER,
        static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)),
        vertices.empty() ? nullptr : vertices.data(),
        GL_DYNAMIC_DRAW
    );

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    pointCount_ = static_cast<GLsizei>(vertices.size());
    return true;
}

void EquilibriumMesh::SetSize(float sizePixels)
{
    sizePixels_ = std::max(1.0f, sizePixels);
}

void EquilibriumMesh::Draw() const
{
    if (vao_ == 0 || pointCount_ == 0) {
        return;
    }

    glPointSize(sizePixels_);

    glBindVertexArray(vao_);
    glDrawArrays(GL_POINTS, 0, pointCount_);
    glBindVertexArray(0);
}

void EquilibriumMesh::Destroy()
{
    if (vao_ != 0) {
        glDeleteVertexArrays(1, &vao_);
        vao_ = 0;
    }

    if (vbo_ != 0) {
        glDeleteBuffers(1, &vbo_);
        vbo_ = 0;
    }

    pointCount_ = 0;
}