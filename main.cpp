#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <GL/glew.h>
#include <iostream>

#include "ShaderProgram.h"
#include "SimplexMesh.h"

const char* vertexShaderSource = R"(
#version 330 core

layout (location = 0) in vec2 aPosition;
layout (location = 1) in vec3 aColor;

out vec3 vColor;

void main()
{
	vColor = aColor;
	gl_Position = vec4(aPosition, 0.0, 1.0);
}
)";

const char* fragmentShaderSource = R"(
#version 330 core

in vec3 vColor;
out vec4 FragColor;

void main()
{
	FragColor = vec4(vColor, 1.0);
}
)";




int main(int argc, char* argv[])
{
	if (!SDL_Init(SDL_INIT_VIDEO)) {
		std::cerr << "SDL_Init failed: " << SDL_GetError() << "\n";
		return 1;
	}

	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

	SDL_Window* window = SDL_CreateWindow("Simplex Simulations", 800, 600, SDL_WINDOW_OPENGL
	);

	if (window == nullptr) {
		std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << "\n";
		SDL_Quit();
		return 1;
	}

	SDL_GLContext context = SDL_GL_CreateContext(window);

	if (context == nullptr) {
		std::cerr << "SDL_GL_CreateContext failed: " << SDL_GetError() << "\n";
		SDL_DestroyWindow(window);
		SDL_Quit();
		return 1;
	}

	glewExperimental = GL_TRUE;
	GLenum glewError = glewInit();

	if (glewError != GLEW_OK) {
		std::cerr << "glewInit failed: " << glewGetErrorString(glewError) << "\n";
		SDL_GL_DestroyContext(context);
		SDL_DestroyWindow(window);
		SDL_Quit();
		return 1;
	}

	ShaderProgram shaderProgram;

	if (!shaderProgram.Create(vertexShaderSource, fragmentShaderSource)) {
		SDL_GL_DestroyContext(context);
		SDL_DestroyWindow(window);
		SDL_Quit();
		return 1;
	}

	SimplexMesh simplexMesh;
	if (!simplexMesh.Create()) {
		std::cerr << "Failed to create simplex mesh.\n";
		shaderProgram.Destroy();
		SDL_GL_DestroyContext(context);
		SDL_DestroyWindow(window);
		SDL_Quit();
		return 1;
	}

	std::cout << "Window created successfully.\n";

	glViewport(0, 0, 800, 600);


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

		SDL_GL_SwapWindow(window);
	}

	simplexMesh.Destroy();
	shaderProgram.Destroy();
	SDL_GL_DestroyContext(context);
	SDL_DestroyWindow(window);
	SDL_Quit();
	return 0;
}