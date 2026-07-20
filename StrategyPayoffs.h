#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

struct StrategyPayoffs final
{
    double cooperators = 0.0;
    double defectors = 0.0;
    double loners = 0.0;

    [[nodiscard]] bool IsFinite() const noexcept
    {
        return std::isfinite(cooperators)
            && std::isfinite(defectors)
            && std::isfinite(loners);
    }

    [[nodiscard]] std::array<double, 3> AsArray() const noexcept
    {
        return { cooperators, defectors, loners };
    }

    [[nodiscard]] double Maximum() const noexcept
    {
        return std::max({ cooperators, defectors, loners });
    }

    [[nodiscard]] double MaximumMagnitude() const noexcept
    {
        return std::max({
            std::abs(cooperators),
            std::abs(defectors),
            std::abs(loners)
        });
    }
};
