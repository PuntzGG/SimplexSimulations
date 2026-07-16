#pragma once

#include <optional>
#include <vector>

#include "SimplexDynamicModel.h"
#include "SimplexState.h"

struct SimplexEquilibrium final
{
    SimplexState state;
    double residual = 0.0;
};

struct SimplexEquilibriumSearchSettings final
{
    int latticeResolution = 32;
    int maximumNewtonIterations = 64;
    int maximumBacktrackingSteps = 24;
    double finiteDifferenceStep = 1e-6;
    double residualTolerance = 1e-9;
    double duplicateDistance = 1e-6;
    double interiorMargin = 1e-12;
    double minimumJacobianDeterminant = 1e-14;

    [[nodiscard]] bool IsComputable() const noexcept;
};

// Finds verified interior rest points. Boundary equilibria require a separate
// face-by-face search and are intentionally not reported by this class.
class SimplexEquilibriumFinder final
{
public:
    [[nodiscard]] std::optional<std::vector<SimplexEquilibrium>> Find(
        const SimplexDynamicModel& dynamics,
        const SimplexEquilibriumSearchSettings& settings = {}
    ) const;
};
