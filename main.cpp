#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <GL/glew.h>
#include <iostream>
#include "ShaderProgram.h"
#include "SimplexMesh.h"
#include "SimplexMapper.h"
#include "PointMesh.h"
#include "ShaderSources.h"
#include "SdlSystem.h"
#include "SdlOpenGlWindow.h"


namespace
{
	constexpr int kWindowWidth = 800;
	constexpr int kWindowHeight = 600;
}

int main(int argc, char* argv[])
{
	(void)argc;
	(void)argv;

	SdlSystem sdlSystem;

	if (!sdlSystem.Initialize()) {
		return 1;
	}

	SdlOpenGlWindow window;

	if (!window.Create("Simplex Simulations", kWindowWidth, kWindowHeight)) {
		return 1;
	}

	glewExperimental = GL_TRUE;
	GLenum glewError = glewInit();

	if (glewError != GLEW_OK) {
		std::cerr << "glewInit failed: " << glewGetErrorString(glewError) << "\n";
		return 1;
	}

	ShaderProgram shaderProgram;

	if (!shaderProgram.Create(ShaderSources::kSimplexVertex, ShaderSources::kSimplexFragment)) {
		return 1;
	}

	ShaderProgram pointShaderProgram;

	if (!pointShaderProgram.Create(ShaderSources::kPointVertex, ShaderSources::kPointFragment)) {
		return 1;
	}

	SimplexMesh simplexMesh;
	if (!simplexMesh.Create()) {
		std::cerr << "Failed to create simplex mesh.\n";
		return 1;
	}

	PointMesh statePointMesh;

	if (!statePointMesh.Create()) {
		std::cerr << "Failed to create state point mesh.\n";
		return 1;
	}

	const SimplexMapper simplexMapper(
		Vec2f{ 0.0f, 0.75f },  // Cooperators
		Vec2f{-0.75f, -0.55f}, // Defectors
		Vec2f{0.75f, -0.55f}  // Loners
	);

	const SimplexState currentState = SimplexState::Normalized(
		1.0,
		1.0,
		1.0
	);

	statePointMesh.SetPosition(simplexMapper.ToNdcPosition(currentState));
	statePointMesh.SetColor(1.0f, 0.3f, 0.0f);
	statePointMesh.SetSize(14.0f);



	std::cout << "Window created successfully.\n";

	glViewport(0, 0, kWindowWidth, kWindowHeight);


	bool running = true;

	while (running) {
		SDL_Event event;

		while (SDL_PollEvent(&event)) {
			if (event.type == SDL_EVENT_QUIT) {
				running = false;
			}
		}

		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		shaderProgram.Use();
		simplexMesh.Draw();

		pointShaderProgram.Use();
		statePointMesh.Draw();

		window.SwapBuffers();
	}
	return 0;
}
