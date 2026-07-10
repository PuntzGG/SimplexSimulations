#pragma once

#include <GL/glew.h>

#include <vector>

#include "Vec2f.h"

class EquilibriumMesh final
{
public:
    EquilibriumMesh() = default;
    ~EquilibriumMesh();

    EquilibriumMesh(const EquilibriumMesh&) = delete;
    EquilibriumMesh& operator=(const EquilibriumMesh&) = delete;

    [[nodiscard]] bool Create();

    [[nodiscard]] bool SetPoints(
        const std::vector<Vec2f>& points,
        float red,
        float green,
        float blue
    );

    void SetSize(float sizePixels);

    void Draw() const;
    void Destroy();

private:
    GLuint vao_ = 0;
    GLuint vbo_ = 0;
    GLsizei pointCount_ = 0;
    float sizePixels_ = 12.0f;
};