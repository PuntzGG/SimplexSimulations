#pragma once

#include <GL/glew.h>

#include <vector>

#include "Vec2f.h"

class TrajectoryMesh final
{
public:
    TrajectoryMesh() = default;
    ~TrajectoryMesh();

    TrajectoryMesh(const TrajectoryMesh&) = delete;
    TrajectoryMesh& operator=(const TrajectoryMesh&) = delete;

    [[nodiscard]] bool Create();

    [[nodiscard]] bool SetPoints(
        const std::vector<Vec2f>& points,
        float red,
        float green,
        float blue
    );

    void Draw() const;
    void Destroy();

private:
    GLuint vao_ = 0;
    GLuint vbo_ = 0;
    GLsizei vertexCount_ = 0;
};