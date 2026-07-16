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

    [[nodiscard]] SDL_Window* NativeWindow() const noexcept { return window_; }
    [[nodiscard]] SDL_GLContext GlContext() const noexcept { return context_; }

    [[nodiscard]] bool GetWindowSize(int& width, int& height) const noexcept;
    [[nodiscard]] bool GetDrawableSize(int& width, int& height) const noexcept;

    void Destroy() noexcept;

private:
    SDL_Window* window_ = nullptr;
    SDL_GLContext context_ = nullptr;
};
