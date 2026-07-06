#pragma once


#include <GL/glew.h>

#include "Vec2f.h"

class PointMesh final
{
public:
	PointMesh() = default;
	~PointMesh();

	PointMesh(const PointMesh&) = delete;
	PointMesh& operator=(const PointMesh&) = delete;

	[[nodiscard]] bool Create();

	void SetPosition(Vec2f position);
	void SetColor(float r, float g, float b);
	void SetSize(float sizePixels);

	void Draw() const;
	void Destroy();

private:
	void UploadVertex() const;

	GLuint vao_ = 0;
	GLuint vbo_ = 0;

	Vec2f position_;
	float red_ = 0.05f;
	float green_ = 0.05f;
	float blue_ = 0.05f;
	float sizePixels_ = 10.0f;
};