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

    inline constexpr const char* kRingPointFragment = R"(
#version 330 core

in vec3 vColor;
out vec4 FragColor;

void main()
{
    float distanceFromCenter = length(gl_PointCoord - vec2(0.5));
    if (distanceFromCenter > 0.5 || distanceFromCenter < 0.32) {
        discard;
    }
    FragColor = vec4(vColor, 1.0);
}
)";

    inline constexpr const char* kHeatMapVertex = R"(
#version 330 core

layout (location = 0) in vec2 aPosition;
layout (location = 1) in float aNormalizedSpeed;

out float vNormalizedSpeed;

void main()
{
    vNormalizedSpeed = aNormalizedSpeed;
    gl_Position = vec4(aPosition, 0.0, 1.0);
}
)";

    inline constexpr const char* kHeatMapFragment = R"(
#version 330 core

in float vNormalizedSpeed;
out vec4 FragColor;

vec3 palette(float value)
{
    const vec3 colors[6] = vec3[6](
        vec3(0.02, 0.05, 0.35),
        vec3(0.00, 0.75, 1.00),
        vec3(0.00, 0.75, 0.22),
        vec3(1.00, 0.92, 0.00),
        vec3(1.00, 0.40, 0.00),
        vec3(0.88, 0.00, 0.00)
    );
    float scaled = clamp(value, 0.0, 1.0) * 5.0;
    int first = min(int(floor(scaled)), 4);
    return mix(colors[first], colors[first + 1], scaled - float(first));
}

void main()
{
    FragColor = vec4(palette(vNormalizedSpeed), 1.0);
}
)";
}
