#pragma once

#include <GL/glew.h>

#include "StreamlineFieldGenerator.h"

class StreamlineMesh final
{
public:
    StreamlineMesh() = default;
    ~StreamlineMesh();

    StreamlineMesh(const StreamlineMesh&) = delete;
    StreamlineMesh& operator=(const StreamlineMesh&) = delete;

    [[nodiscard]] bool Create();
    [[nodiscard]] bool SetData(
        const StreamlineFieldResult& field,
        int viewportWidth,
        int viewportHeight,
        float arrowLengthPixels = 8.0F
    );
    void Draw() const;
    void Destroy() noexcept;

private:
    GLuint vao_ = 0;
    GLuint vbo_ = 0;
    GLsizei lineVertexCount_ = 0;
    GLsizei arrowVertexCount_ = 0;
};
