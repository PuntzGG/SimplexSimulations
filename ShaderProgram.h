#pragma once

#include <GL/glew.h>

// Owns one OpenGL shader program and controls its lifetime.
class ShaderProgram
{
public:
	ShaderProgram() = default;
	~ShaderProgram();

	// Prevents two ShaderProgram objects from owning the same OpenGL handle.
	ShaderProgram(const ShaderProgram&) = delete;
	ShaderProgram& operator=(const ShaderProgram&) = delete;

	// Compiles, links, and stores a shader program from source strings.
	bool Create(const char* vertexSource, const char* fragmentSource);

	// Makes this shader program active for subsequent draw calls.
	void Use() const;

	// Deletes the OpenGL program if it exists.
	void Destroy();

private:

	// OpenGL handle for the linked shader program; 0 means "no program".
	GLuint id_ = 0;
};

