#pragma once

#include <optional>
#include <vector>

#include "SimplexDynamicModel.h"
#include "SimplexState.h"

struct SimplexEquilibrium final
{
    SimplexState state;
    double residual;
};

struct SimplexEquilibriumSearchSettings final
{
    int latticeResolution = 32;
    int maximumNewtonIterations = 64;

    double finiteDifferenceStep = 1e-6;
    double residualTolerance = 1e-9;
    double duplicateDistance = 1e-6;

    [[nodiscard]] bool IsComputable() const;
};

class SimplexEquilibriumFinder final
{
public:
    [[nodiscard]] std::optional<std::vector<SimplexEquilibrium>> Find(
        const SimplexDynamicModel& dynamics,
        const SimplexEquilibriumSearchSettings& settings = {}
    ) const;
};