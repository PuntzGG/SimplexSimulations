#pragma once

namespace ShaderSources
{
    inline constexpr const char* kSimplexVertex = R"(
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

    inline constexpr const char* kSimplexFragment = R"(
#version 330 core

in vec3 vColor;
out vec4 FragColor;

void main()
{
    FragColor = vec4(vColor, 1.0);
}
)";

    inline constexpr const char* kSimplexOutlineFragment = R"(
#version 330 core

out vec4 FragColor;

void main()
{
    FragColor = vec4(0.08, 0.08, 0.08, 1.0);
}
)";

    inline constexpr const char* kPointVertex = R"(
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

    inline constexpr const char* kPointFragment = R"(
#version 330 core

in vec3 vColor;
out vec4 FragColor;

void main()
{
    vec2 centeredPointCoordinate = gl_PointCoord - vec2(0.5);
    float distanceFromCenter = length(centeredPointCoordinate);

    if (distanceFromCenter > 0.5) {
        discard;
    }

    FragColor = vec4(vColor, 1.0);
}
)";
}