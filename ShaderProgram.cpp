#include "ShaderProgram.h"

#include <iostream>
#include <string>


namespace
{
	GLuint CompileShader(GLenum shaderType, const char* source)
	{
		GLuint shader = glCreateShader(shaderType);
		glShaderSource(shader, 1, &source, nullptr);
		glCompileShader(shader);

		GLint success = 0;
		glGetShaderiv(shader, GL_COMPILE_STATUS, &success);

		if (!success) {
			GLint logLength = 0;
			glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLength);

			std::string log(logLength, ' ');
			glGetShaderInfoLog(shader, logLength, nullptr, log.data());

			std::cerr << "Shader compilation failed:\n" << log << "\n";

			glDeleteShader(shader);
			return 0;
		}

		return shader;
	}
}


ShaderProgram::~ShaderProgram()
{
	Destroy();
}

bool ShaderProgram::Create(const char* vertexSource, const char* fragmentSource)
{
	Destroy();

	GLuint vertexShader = CompileShader(GL_VERTEX_SHADER, vertexSource);
	if (vertexShader == 0) {
		return false;
	}

	GLuint fragmentShader = CompileShader(GL_FRAGMENT_SHADER, fragmentSource);
	if (fragmentShader == 0) {
		glDeleteShader(vertexShader);
		return false;
	}

	GLuint program = glCreateProgram();
	glAttachShader(program, vertexShader);
	glAttachShader(program, fragmentShader);
	glLinkProgram(program);

	GLint success = 0;
	glGetProgramiv(program, GL_LINK_STATUS, &success);

	if (!success) {
		GLint logLength = 0;
		glGetProgramiv(program, GL_INFO_LOG_LENGTH, &logLength);

		std::string log(logLength, ' ');
		glGetProgramInfoLog(program, logLength, nullptr, log.data());

		std::cerr << "Shader program linking failed:\n" << log << "\n";

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

void ShaderProgram::Use() const
{
	glUseProgram(id_);
}

void ShaderProgram::Destroy()
{
	if (id_ != 0) {
		glDeleteProgram(id_);
		id_ = 0;
	}
}