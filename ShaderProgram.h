#pragma once

#include <GL/glew.h>

class ShaderProgram final
{
public:
    ShaderProgram() = default;
    ~ShaderProgram();

    ShaderProgram(const ShaderProgram&) = delete;
    ShaderProgram& operator=(const ShaderProgram&) = delete;

    [[nodiscard]] bool Create(
        const char* vertexSource,
        const char* fragmentSource
    );

    void Use() const noexcept;
    void Destroy() noexcept;

private:
    GLuint id_ = 0;
};
