#pragma once

#include <cmath>

struct OpggParameters final
{
    int groupSize = 5;
    double multiplicationFactor = 3.0;   // r
    double lonerPayoffMultiplier = 1.0;  // sigma
    double contributionCost = 1.0;       // c
    double punishmentFraction = 0.25;    // v in the Python source by Jossias
    double logitNoise = 0.08;            // eta

    [[nodiscard]] bool IsComputable() const
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
};