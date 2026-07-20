#include "SpeedHeatMapMesh.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <vector>

namespace
{
    struct Vertex final
    {
        float x = 0.0F;
        float y = 0.0F;
        float normalizedSpeed = 0.0F;
    };
}

SpeedHeatMapMesh::~SpeedHeatMapMesh()
{
    Destroy();
}

bool SpeedHeatMapMesh::Create()
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
        1,
        GL_FLOAT,
        GL_FALSE,
        sizeof(Vertex),
        reinterpret_cast<void*>(offsetof(Vertex, normalizedSpeed))
    );
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    return true;
}

bool SpeedHeatMapMesh::SetData(
    const SpeedHeatMapResult& heatMap,
    const SimplexMapper& mapper
)
{
    if (vbo_ == 0
        || heatMap.triangleIndices.size() % 3U != 0U
        || heatMap.mixedRegionTriangles.size()
            != heatMap.triangleIndices.size() / 3U) {
        return false;
    }

    std::vector<Vertex> vertices;
    vertices.reserve(heatMap.triangleIndices.size());
    for (std::size_t triangle = 0;
         triangle < heatMap.mixedRegionTriangles.size();
         ++triangle) {
        std::array<const SpeedHeatMapSample*, 3> samples{};
        for (std::size_t corner = 0; corner < 3U; ++corner) {
            const std::uint32_t index = heatMap.triangleIndices[
                triangle * 3U + corner
            ];
            if (index >= heatMap.samples.size()) {
                return false;
            }
            samples[corner] = &heatMap.samples[index];
        }

        const bool flatShadeMixedCell =
            heatMap.mixedRegionTriangles[triangle] != 0U;
        const float flatSpeed = (
            samples[0]->normalizedSpeed
            + samples[1]->normalizedSpeed
            + samples[2]->normalizedSpeed
        ) / 3.0F;
        for (const SpeedHeatMapSample* sample : samples) {
            const Vec2f position = mapper.ToNdcPosition(sample->state);
            vertices.push_back(Vertex{
                position.x,
                position.y,
                flatShadeMixedCell ? flatSpeed : sample->normalizedSpeed
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
    vertexCount_ = static_cast<GLsizei>(vertices.size());
    return true;
}

void SpeedHeatMapMesh::Draw() const
{
    if (vao_ == 0 || vertexCount_ == 0) {
        return;
    }
    glBindVertexArray(vao_);
    glDrawArrays(GL_TRIANGLES, 0, vertexCount_);
    glBindVertexArray(0);
}

void SpeedHeatMapMesh::Destroy() noexcept
{
    if (vao_ != 0) {
        glDeleteVertexArrays(1, &vao_);
        vao_ = 0;
    }
    if (vbo_ != 0) {
        glDeleteBuffers(1, &vbo_);
        vbo_ = 0;
    }
    vertexCount_ = 0;
}
