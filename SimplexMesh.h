#pragma once

#include <GL/glew.h>

//"final" means no other class is allowed to inherit from SimplexMesh.
class SimplexMesh final
{
public:
	// Use the normal compiler-generated constructor.
	SimplexMesh() = default;
	~SimplexMesh();


	// Forbid copy construction and copy assignment.
	SimplexMesh(const SimplexMesh&) = delete;
	SimplexMesh& operator=(const SimplexMesh&) = delete;

	//[[nodiscard]] means the caller should not ignore the returned bool.
	[[nodiscard]] bool Create();
	void Draw() const;
	void DrawOutline() const;
	void Destroy();

private:
	GLuint vao_ = 0;
	GLuint vbo_ = 0;
	GLsizei vertexCount_ = 0;
};

