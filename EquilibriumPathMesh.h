#pragma once

#include <GL/glew.h>

#include <vector>

#include "Vec2f.h"

class EquilibriumPathMesh final
{
public:
    EquilibriumPathMesh() = default;
    ~EquilibriumPathMesh();

    EquilibriumPathMesh(const EquilibriumPathMesh&) = delete;
    EquilibriumPathMesh& operator=(const EquilibriumPathMesh&) = delete;

    [[nodiscard]] bool Create();

    [[nodiscard]] bool SetPaths(
        const std::vector<std::vector<Vec2f>>& paths,
        float red,
        float green,
        float blue
    );

    void Draw() const;
    void Destroy();

private:
    struct DrawRange
    {
        GLint firstVertex = 0;
        GLsizei vertexCount = 0;
    };

    GLuint vao_ = 0;
    GLuint vbo_ = 0;
    std::vector<DrawRange> drawRanges_;
};