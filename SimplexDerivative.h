#pragma once

#include <cmath>

struct SimplexDerivative final
{
    double dx = 0.0;
    double dy = 0.0;
    double dz = 0.0;

    [[nodiscard]] bool IsFinite() const noexcept
    {
        return std::isfinite(dx) && std::isfinite(dy) && std::isfinite(dz);
    }

    [[nodiscard]] bool IsTangent(double epsilon = 1e-10) const noexcept
    {
        if (!IsFinite() || !std::isfinite(epsilon) || epsilon < 0.0) {
            return false;
        }

        const double scale = 1.0 + std::abs(dx) + std::abs(dy) + std::abs(dz);
        return std::abs(dx + dy + dz) <= epsilon * scale;
    }
};
