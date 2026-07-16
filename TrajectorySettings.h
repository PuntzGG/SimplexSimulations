#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

struct TrajectorySettings final
{
    double totalTime = 10.0;
    double timeStep = 0.02;
    int maxSteps = 10000;

    [[nodiscard]] std::optional<int> RequestedStepCount() const noexcept
    {
        if (!std::isfinite(totalTime)
            || !std::isfinite(timeStep)
            || totalTime <= 0.0
            || timeStep <= 0.0
            || maxSteps <= 0) {
            return std::nullopt;
        }

        const double rawStepCount = totalTime / timeStep;
        if (!std::isfinite(rawStepCount) || rawStepCount <= 0.0) {
            return std::nullopt;
        }

        // Ratios that are mathematically integral can land a few ulps above an
        // integer (for example 0.3 / 0.1). Treat those as exact instead of
        // accidentally requesting one extra, zero-length final step.
        const double nearestInteger = std::round(rawStepCount);
        const double scale = std::max(1.0, std::abs(rawStepCount));
        const double integralTolerance =
            32.0 * std::numeric_limits<double>::epsilon() * scale;

        const double resolvedStepCount =
            std::abs(rawStepCount - nearestInteger) <= integralTolerance
                ? nearestInteger
                : std::ceil(rawStepCount);

        if (!std::isfinite(resolvedStepCount)
            || resolvedStepCount < 1.0
            || resolvedStepCount
                > static_cast<double>(std::numeric_limits<int>::max())) {
            return std::nullopt;
        }

        const int requestedSteps = static_cast<int>(resolvedStepCount);
        if (requestedSteps > maxSteps) {
            return std::nullopt;
        }

        return requestedSteps;
    }

    [[nodiscard]] bool IsComputable() const noexcept
    {
        return RequestedStepCount().has_value();
    }
};
