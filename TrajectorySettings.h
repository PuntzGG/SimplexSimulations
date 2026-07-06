#pragma once

#include <cmath>

struct TrajectorySettings final
{
    double totalTime = 10.0;
    double timeStep = 0.02;
    int maxSteps = 10000;

    [[nodiscard]] bool IsComputable() const
    {
        return std::isfinite(totalTime)
            && std::isfinite(timeStep)
            && totalTime > 0.0
            && timeStep > 0.0
            && maxSteps > 0;
    }
};