#include "SdlOpenGlWindow.h"

#include <iostream>

SdlOpenGlWindow::~SdlOpenGlWindow()
{
    Destroy();
}

bool SdlOpenGlWindow::Create(const char* title, int width, int height)
{
    Destroy();

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

    window_ = SDL_CreateWindow(
        title,
        width,
        height,
        SDL_WINDOW_OPENGL
    );

    if (window_ == nullptr) {
        std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << "\n";
        return false;
    }

    context_ = SDL_GL_CreateContext(window_);

    if (context_ == nullptr) {
        std::cerr << "SDL_GL_CreateContext failed: " << SDL_GetError() << "\n";
        Destroy();
        return false;
    }

    return true;
}

void SdlOpenGlWindow::SwapBuffers() const
{
    if (window_ != nullptr) {
        SDL_GL_SwapWindow(window_);
    }
}

void SdlOpenGlWindow::Destroy()
{
    if (context_ != nullptr) {
        SDL_GL_DestroyContext(context_);
        context_ = nullptr;
    }

    if (window_ != nullptr) {
        SDL_DestroyWindow(window_);
        window_ = nullptr;
    }
}