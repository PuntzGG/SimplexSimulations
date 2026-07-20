#pragma once

#include <GL/glew.h>

#include "SimplexMapper.h"
#include "SpeedHeatMapGenerator.h"

class SpeedHeatMapMesh final
{
public:
    SpeedHeatMapMesh() = default;
    ~SpeedHeatMapMesh();

    SpeedHeatMapMesh(const SpeedHeatMapMesh&) = delete;
    SpeedHeatMapMesh& operator=(const SpeedHeatMapMesh&) = delete;

    [[nodiscard]] bool Create();
    [[nodiscard]] bool SetData(
        const SpeedHeatMapResult& heatMap,
        const SimplexMapper& mapper
    );
    void Draw() const;
    void Destroy() noexcept;

private:
    GLuint vao_ = 0;
    GLuint vbo_ = 0;
    GLsizei vertexCount_ = 0;
};
