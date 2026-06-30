#pragma once

#include <GL/glew.h>

class ShaderProgram
{
public:
	ShaderProgram() = default;
	~ShaderProgram();

	ShaderProgram(const ShaderProgram&) = delete;
	ShaderProgram& operator=(const ShaderProgram&) = delete;

	bool Create(const char* vertexSource, const char* fragmentSource);
	void Use() const;
	void Destroy();

private:
	GLuint id_ = 0;
};

