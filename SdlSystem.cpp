#include "SdlSystem.h"

#include <SDL3/SDL.h>
#include <iostream>

SdlSystem::~SdlSystem()
{
    Destroy();
}

bool SdlSystem::Initialize()
{
    Destroy();

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << "\n";
        return false;
    }

    initialized_ = true;
    return true;
}

void SdlSystem::Destroy()
{
    if (initialized_) {
        SDL_Quit();
        initialized_ = false;
    }
}