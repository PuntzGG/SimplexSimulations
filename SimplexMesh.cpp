#include "SimplexMesh.h"

#include <array>
#include <cstddef>

namespace
{
	struct Vertex
	{
		float x;
		float y;
		float r;
		float g;
		float b;
	};

	constexpr std::array<Vertex, 3> kSimplexVertices = {{
		{  0.0f,   0.75f,  0.7f, 0.0f, 0.99f },
		{ -0.75f, -0.55f,  0.0f, 0.99f, 0.99f },
		{  0.75f, -0.55f,  0.0f, 0.0f, 0.7f }
	}};
}

SimplexMesh::~SimplexMesh()
{
	Destroy();
}

bool SimplexMesh::Create()
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

	glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(kSimplexVertices.size() * sizeof(Vertex)), kSimplexVertices.data(), GL_STATIC_DRAW);

	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, x)));
	glEnableVertexAttribArray(0);

	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, r)));
	glEnableVertexAttribArray(1);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	vertexCount_ = static_cast<GLsizei>(kSimplexVertices.size());
	return true;
}

void SimplexMesh::Draw() const
{
	if (vao_ == 0 || vertexCount_ == 0) {
		return;
	}

	glBindVertexArray(vao_);
	glDrawArrays(GL_TRIANGLES, 0, vertexCount_);
}

void SimplexMesh::Destroy()
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