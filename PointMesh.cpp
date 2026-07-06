#include "PointMesh.h"

#include <algorithm>
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

	Vertex MakeVertex(Vec2f position, float red, float green, float blue)
	{
		return Vertex{
			position.x,
			position.y,
			red,
			green,
			blue
		};
	}
}

PointMesh::~PointMesh()
{
	Destroy();
}

bool PointMesh::Create()
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

	const Vertex vertex = MakeVertex(position_, red_, green_, blue_);

	glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex), &vertex, GL_DYNAMIC_DRAW);

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
		reinterpret_cast<void*>(offsetof(Vertex, r))
	);

	glEnableVertexAttribArray(1);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	return true;
}

void PointMesh::SetPosition(Vec2f position)
{
	position_ = position;
	UploadVertex();
}

void PointMesh::SetColor(float r, float g, float b)
{
	red_ = std::clamp(r, 0.0f, 1.0f);
	green_ = std::clamp(g, 0.0f, 1.0f);
	blue_ = std::clamp(b, 0.0f, 1.0f);

	UploadVertex();
}

void PointMesh::SetSize(float sizePixels)
{
	sizePixels_ = std::max(1.0f, sizePixels);
}

void PointMesh::Draw() const
{
	if (vao_ == 0) {
		return;
	}

	glPointSize(sizePixels_);
	
	glBindVertexArray(vao_);
	glDrawArrays(GL_POINTS, 0, 1);
	glBindVertexArray(0);
}

void PointMesh::Destroy()
{
	if (vao_ != 0) {
		glDeleteVertexArrays(1, &vao_);
		vao_ = 0;
	}

	if (vbo_ != 0) {
		glDeleteBuffers(1, &vbo_);
		vbo_ = 0;
	}
}

void PointMesh::UploadVertex() const
{
	if (vbo_ == 0) {
		return;
	}
	
	const Vertex vertex = MakeVertex(position_, red_, green_, blue_);

	glBindBuffer(GL_ARRAY_BUFFER, vbo_);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Vertex), &vertex);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}