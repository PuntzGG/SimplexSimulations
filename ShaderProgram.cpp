#include "ShaderProgram.h"

#include <iostream>
#include <string>

namespace
{
    [[nodiscard]] GLuint CompileShader(
        GLenum shaderType,
        const char* source,
        const char* stageName
    )
    {
        if (source == nullptr || source[0] == '\0') {
            std::cerr << "Cannot compile an empty " << stageName
                      << " shader source.\n";
            return 0;
        }

        const GLuint shader = glCreateShader(shaderType);
        if (shader == 0) {
            std::cerr << "glCreateShader failed for the " << stageName
                      << " shader.\n";
            return 0;
        }

        glShaderSource(shader, 1, &source, nullptr);
        glCompileShader(shader);

        GLint success = GL_FALSE;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (success == GL_TRUE) {
            return shader;
        }

        GLint logLength = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLength);
        std::string log(
            logLength > 1 ? static_cast<std::size_t>(logLength) : 1U,
            '\0'
        );
        GLsizei writtenLength = 0;
        glGetShaderInfoLog(
            shader,
            static_cast<GLsizei>(log.size()),
            &writtenLength,
            log.data()
        );
        if (writtenLength >= 0
            && static_cast<std::size_t>(writtenLength) < log.size()) {
            log.resize(static_cast<std::size_t>(writtenLength));
        }

        std::cerr << stageName << " shader compilation failed:\n"
                  << log << '\n';
        glDeleteShader(shader);
        return 0;
    }
}

ShaderProgram::~ShaderProgram()
{
    Destroy();
}

bool ShaderProgram::Create(
    const char* vertexSource,
    const char* fragmentSource
)
{
    Destroy();

    const GLuint vertexShader = CompileShader(
        GL_VERTEX_SHADER,
        vertexSource,
        "Vertex"
    );
    if (vertexShader == 0) {
        return false;
    }

    const GLuint fragmentShader = CompileShader(
        GL_FRAGMENT_SHADER,
        fragmentSource,
        "Fragment"
    );
    if (fragmentShader == 0) {
        glDeleteShader(vertexShader);
        return false;
    }

    const GLuint program = glCreateProgram();
    if (program == 0) {
        std::cerr << "glCreateProgram failed.\n";
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
        return false;
    }

    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);
    glLinkProgram(program);

    GLint success = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (success != GL_TRUE) {
        GLint logLength = 0;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &logLength);
        std::string log(
            logLength > 1 ? static_cast<std::size_t>(logLength) : 1U,
            '\0'
        );
        GLsizei writtenLength = 0;
        glGetProgramInfoLog(
            program,
            static_cast<GLsizei>(log.size()),
            &writtenLength,
            log.data()
        );
        if (writtenLength >= 0
            && static_cast<std::size_t>(writtenLength) < log.size()) {
            log.resize(static_cast<std::size_t>(writtenLength));
        }

        std::cerr << "Shader program linking failed:\n" << log << '\n';
        glDetachShader(program, vertexShader);
        glDetachShader(program, fragmentShader);
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
        glDeleteProgram(program);
        return false;
    }

    glDetachShader(program, vertexShader);
    glDetachShader(program, fragmentShader);
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    id_ = program;
    return true;
}

void ShaderProgram::Use() const noexcept
{
    glUseProgram(id_);
}

void ShaderProgram::Destroy() noexcept
{
    if (id_ != 0) {
        glDeleteProgram(id_);
        id_ = 0;
    }
}
