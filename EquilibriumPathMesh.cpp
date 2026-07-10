#include "EquilibriumPathMesh.h"

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

EquilibriumPathMesh::~EquilibriumPathMesh()
{
    Destroy();
}

bool EquilibriumPathMesh::Create()
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

bool EquilibriumPathMesh::SetPaths(
    const std::vector<std::vector<Vec2f>>& paths,
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
    drawRanges_.clear();

    for (const std::vector<Vec2f>& path : paths) {
        if (path.size() < 2) {
            continue;
        }

        drawRanges_.push_back(DrawRange{
            static_cast<GLint>(vertices.size()),
            static_cast<GLsizei>(path.size())
            });

        for (const Vec2f& point : path) {
            vertices.push_back(Vertex{
                point.x,
                point.y,
                red,
                green,
                blue
                });
        }
    }

    glBindBuffer(GL_ARRAY_BUFFER, vbo_);

    glBufferData(
        GL_ARRAY_BUFFER,
        static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)),
        vertices.empty() ? nullptr : vertices.data(),
        GL_DYNAMIC_DRAW
    );

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    return true;
}

void EquilibriumPathMesh::Draw() const
{
    if (vao_ == 0 || drawRanges_.empty()) {
        return;
    }

    glLineWidth(2.5f);

    glBindVertexArray(vao_);

    for (const DrawRange& range : drawRanges_) {
        glDrawArrays(
            GL_LINE_STRIP,
            range.firstVertex,
            range.vertexCount
        );
    }

    glBindVertexArray(0);
}

void EquilibriumPathMesh::Destroy()
{
    if (vao_ != 0) {
        glDeleteVertexArrays(1, &vao_);
        vao_ = 0;
    }

    if (vbo_ != 0) {
        glDeleteBuffers(1, &vbo_);
        vbo_ = 0;
    }

    drawRanges_.clear();
}