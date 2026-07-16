#pragma once

#include <cmath>

struct OpggParameters final
{
    int groupSize = 5;
    double multiplicationFactor = 3.0;   // r
    double lonerPayoffMultiplier = 1.0;  // sigma
    double contributionCost = 1.0;       // c
    double punishmentFraction = 0.25;    // d (v in the Python notebook)
    double logitNoise = 0.08;            // eta

    [[nodiscard]] bool IsNumericallyComputable() const noexcept
    {
        return groupSize >= 2
            && std::isfinite(multiplicationFactor)
            && std::isfinite(lonerPayoffMultiplier)
            && std::isfinite(contributionCost)
            && std::isfinite(punishmentFraction)
            && std::isfinite(logitNoise)
            && contributionCost > 0.0
            && logitNoise > 0.0;
    }

    [[nodiscard]] bool IsValidForModel() const noexcept
    {
        if (!IsNumericallyComputable()) {
            return false;
        }

        const double n = static_cast<double>(groupSize);
        return multiplicationFactor > 1.0
            && multiplicationFactor < n
            && lonerPayoffMultiplier > 0.0
            && lonerPayoffMultiplier < multiplicationFactor - 1.0
            && punishmentFraction >= 0.0
            && punishmentFraction <= 1.0;
    }

    [[nodiscard]] bool IsComputable() const noexcept
    {
        return IsValidForModel();
    }
};
