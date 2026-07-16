#include "SdlOpenGlWindow.h"

#include <iostream>

namespace
{
    [[nodiscard]] bool SetGlAttribute(SDL_GLAttr attribute, int value)
    {
        if (!SDL_GL_SetAttribute(attribute, value)) {
            std::cerr << "SDL_GL_SetAttribute failed: "
                      << SDL_GetError() << '\n';
            return false;
        }
        return true;
    }
}

SdlOpenGlWindow::~SdlOpenGlWindow()
{
    Destroy();
}

bool SdlOpenGlWindow::Create(const char* title, int width, int height)
{
    Destroy();

    if (title == nullptr || width <= 0 || height <= 0) {
        std::cerr << "Invalid SDL window creation arguments.\n";
        return false;
    }

    if (!SetGlAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3)
        || !SetGlAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3)
        || !SetGlAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE)
        || !SetGlAttribute(SDL_GL_DOUBLEBUFFER, 1)) {
        return false;
    }

    window_ = SDL_CreateWindow(
        title,
        width,
        height,
        SDL_WINDOW_OPENGL
            | SDL_WINDOW_RESIZABLE
            | SDL_WINDOW_HIGH_PIXEL_DENSITY
    );
    if (window_ == nullptr) {
        std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << '\n';
        return false;
    }

    context_ = SDL_GL_CreateContext(window_);
    if (context_ == nullptr) {
        std::cerr << "SDL_GL_CreateContext failed: " << SDL_GetError() << '\n';
        Destroy();
        return false;
    }

    if (!SDL_GL_MakeCurrent(window_, context_)) {
        std::cerr << "SDL_GL_MakeCurrent failed: " << SDL_GetError() << '\n';
        Destroy();
        return false;
    }

    if (!SDL_GL_SetSwapInterval(1)) {
        std::cerr << "Warning: vertical synchronization could not be enabled: "
                  << SDL_GetError() << '\n';
    }

    return true;
}

void SdlOpenGlWindow::SwapBuffers() const
{
    if (window_ != nullptr && !SDL_GL_SwapWindow(window_)) {
        std::cerr << "SDL_GL_SwapWindow failed: " << SDL_GetError() << '\n';
    }
}

bool SdlOpenGlWindow::GetWindowSize(int& width, int& height) const noexcept
{
    width = 0;
    height = 0;
    return window_ != nullptr && SDL_GetWindowSize(window_, &width, &height);
}

bool SdlOpenGlWindow::GetDrawableSize(int& width, int& height) const noexcept
{
    width = 0;
    height = 0;
    return window_ != nullptr
        && SDL_GetWindowSizeInPixels(window_, &width, &height);
}

void SdlOpenGlWindow::Destroy() noexcept
{
    if (context_ != nullptr) {
        if (!SDL_GL_DestroyContext(context_)) {
            std::cerr << "Warning: SDL_GL_DestroyContext failed: "
                      << SDL_GetError() << '\n';
        }
        context_ = nullptr;
    }

    if (window_ != nullptr) {
        SDL_DestroyWindow(window_);
        window_ = nullptr;
    }
}
