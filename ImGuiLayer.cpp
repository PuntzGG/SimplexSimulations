#include "ImGuiLayer.h"

#include <iostream>

#include "imgui.h"
#include "imgui_impl_opengl3.h"
#include "imgui_impl_sdl3.h"

ImGuiLayer::~ImGuiLayer()
{
    Destroy();
}

bool ImGuiLayer::Initialize(
    SDL_Window* window,
    SDL_GLContext glContext,
    const char* glslVersion
)
{
    Destroy();

    if (window == nullptr || glContext == nullptr || glslVersion == nullptr) {
        std::cerr << "ImGui initialization requires a window, OpenGL context, and GLSL version.\n";
        return false;
    }

    IMGUI_CHECKVERSION();

    if (ImGui::CreateContext() == nullptr) {
        std::cerr << "ImGui::CreateContext failed.\n";
        return false;
    }

    contextCreated_ = true;

    ImGui::StyleColorsDark();

    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    if (!ImGui_ImplSDL3_InitForOpenGL(window, glContext)) {
        std::cerr << "ImGui SDL3 backend initialization failed.\n";
        Destroy();
        return false;
    }

    sdlBackendInitialized_ = true;

    if (!ImGui_ImplOpenGL3_Init(glslVersion)) {
        std::cerr << "ImGui OpenGL3 backend initialization failed.\n";
        Destroy();
        return false;
    }

    openglBackendInitialized_ = true;
    return true;
}

void ImGuiLayer::ProcessEvent(const SDL_Event& event)
{
    if (sdlBackendInitialized_) {
        ImGui_ImplSDL3_ProcessEvent(&event);
    }
}

void ImGuiLayer::BeginFrame()
{
    if (!openglBackendInitialized_ || !sdlBackendInitialized_) {
        return;
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();
}

void ImGuiLayer::Render()
{
    if (!openglBackendInitialized_) {
        return;
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

bool ImGuiLayer::WantsMouseInput() const
{
    return contextCreated_ && ImGui::GetIO().WantCaptureMouse;
}

bool ImGuiLayer::WantsKeyboardInput() const
{
    return contextCreated_ && ImGui::GetIO().WantCaptureKeyboard;
}

void ImGuiLayer::Destroy()
{
    if (openglBackendInitialized_) {
        ImGui_ImplOpenGL3_Shutdown();
        openglBackendInitialized_ = false;
    }

    if (sdlBackendInitialized_) {
        ImGui_ImplSDL3_Shutdown();
        sdlBackendInitialized_ = false;
    }

    if (contextCreated_) {
        ImGui::DestroyContext();
        contextCreated_ = false;
    }
}