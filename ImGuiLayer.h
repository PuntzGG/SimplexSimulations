#pragma once

#include <SDL3/SDL.h>

class ImGuiLayer final
{
public:
    ImGuiLayer() = default;
    ~ImGuiLayer();

    ImGuiLayer(const ImGuiLayer&) = delete;
    ImGuiLayer& operator=(const ImGuiLayer&) = delete;

    [[nodiscard]] bool Initialize(
        SDL_Window* window,
        SDL_GLContext glContext,
        const char* glslVersion
    );

    void ProcessEvent(const SDL_Event& event);
    void BeginFrame();
    void Render();

    [[nodiscard]] bool WantsMouseInput() const;
    [[nodiscard]] bool WantsKeyboardInput() const;

    void Destroy();

private:
    bool contextCreated_ = false;
    bool sdlBackendInitialized_ = false;
    bool openglBackendInitialized_ = false;
};