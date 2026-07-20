#include <SDL3/SDL_main.h>

#include "SimplexBeastApplication.h"

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    SimplexBeastApplication application;
    return application.Run();
}
