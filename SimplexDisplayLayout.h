#pragma once

#include "Vec2f.h"

namespace SimplexDisplayLayout
{
    // Reserve the left side of the default window for scientific controls
    // while keeping one authoritative triangle geometry for mapping and GL.
    inline constexpr Vec2f kCooperatorsVertex{ 0.32F, 0.80F };
    inline constexpr Vec2f kDefectorsVertex{ 0.94F, -0.62F };
    inline constexpr Vec2f kLonersVertex{ -0.30F, -0.62F };
}
