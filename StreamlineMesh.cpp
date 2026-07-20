#include "StreamlineMesh.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace
{
    struct Vertex final
    {
        float x = 0.0F;
        float y = 0.0F;
        float red = 0.0F;
        float green = 0.0F;
        float blue = 0.0F;
    };

    [[nodiscard]] Vertex MakeVertex(Vec2f point) noexcept
    {
        return Vertex{ point.x, point.y, 0.035F, 0.035F, 0.045F };
    }
}

StreamlineMesh::~StreamlineMesh()
{
    Destroy();
}

bool StreamlineMesh::Create()
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
        reinterpret_cast<void*>(offsetof(Vertex, red))
    );
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    return true;
}

bool StreamlineMesh::SetData(
    const StreamlineFieldResult& field,
    int viewportWidth,
    int viewportHeight,
    float arrowLengthPixels
)
{
    if (vbo_ == 0
        || viewportWidth <= 0
        || viewportHeight <= 0
        || !std::isfinite(arrowLengthPixels)
        || arrowLengthPixels <= 0.0F
        || field.lineSegmentVertices.size() % 2U != 0U) {
        return false;
    }

    std::vector<Vertex> vertices;
    vertices.reserve(
        field.lineSegmentVertices.size() + field.arrows.size() * 3U
    );
    for (const Vec2f point : field.lineSegmentVertices) {
        vertices.push_back(MakeVertex(point));
    }
    lineVertexCount_ = static_cast<GLsizei>(vertices.size());

    const float halfWidthPixels = arrowLengthPixels * 0.42F;
    for (const StreamlineArrow& arrow : field.arrows) {
        float directionX = arrow.direction.x
            * static_cast<float>(viewportWidth) * 0.5F;
        float directionY = arrow.direction.y
            * static_cast<float>(viewportHeight) * 0.5F;
        const float pixelLength = std::sqrt(
            directionX * directionX + directionY * directionY
        );
        if (!std::isfinite(pixelLength) || pixelLength <= 1e-5F) {
            continue;
        }
        directionX /= pixelLength;
        directionY /= pixelLength;
        const float perpendicularX = -directionY;
        const float perpendicularY = directionX;

        const Vec2f base{
            arrow.tip.x - directionX * arrowLengthPixels * 2.0F
                / static_cast<float>(viewportWidth),
            arrow.tip.y - directionY * arrowLengthPixels * 2.0F
                / static_cast<float>(viewportHeight)
        };
        const Vec2f left{
            base.x + perpendicularX * halfWidthPixels * 2.0F
                / static_cast<float>(viewportWidth),
            base.y + perpendicularY * halfWidthPixels * 2.0F
                / static_cast<float>(viewportHeight)
        };
        const Vec2f right{
            base.x - perpendicularX * halfWidthPixels * 2.0F
                / static_cast<float>(viewportWidth),
            base.y - perpendicularY * halfWidthPixels * 2.0F
                / static_cast<float>(viewportHeight)
        };
        vertices.push_back(MakeVertex(arrow.tip));
        vertices.push_back(MakeVertex(left));
        vertices.push_back(MakeVertex(right));
    }
    arrowVertexCount_ = static_cast<GLsizei>(vertices.size())
        - lineVertexCount_;

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

void StreamlineMesh::Draw() const
{
    if (vao_ == 0) {
        return;
    }
    glBindVertexArray(vao_);
    if (lineVertexCount_ > 0) {
        glLineWidth(1.0F);
        glDrawArrays(GL_LINES, 0, lineVertexCount_);
    }
    if (arrowVertexCount_ > 0) {
        glDrawArrays(GL_TRIANGLES, lineVertexCount_, arrowVertexCount_);
    }
    glBindVertexArray(0);
}

void StreamlineMesh::Destroy() noexcept
{
    if (vao_ != 0) {
        glDeleteVertexArrays(1, &vao_);
        vao_ = 0;
    }
    if (vbo_ != 0) {
        glDeleteBuffers(1, &vbo_);
        vbo_ = 0;
    }
    lineVertexCount_ = 0;
    arrowVertexCount_ = 0;
}
