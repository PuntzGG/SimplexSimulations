#pragma once

#include <SDL3/SDL.h>

class SdlOpenGlWindow final
{
public:
    SdlOpenGlWindow() = default;
    ~SdlOpenGlWindow();

    SdlOpenGlWindow(const SdlOpenGlWindow&) = delete;
    SdlOpenGlWindow& operator=(const SdlOpenGlWindow&) = delete;

    [[nodiscard]] bool Create(const char* title, int width, int height);
    void SwapBuffers() const;
    void Destroy();

private:
    SDL_Window* window_ = nullptr;
    SDL_GLContext context_ = nullptr;
};