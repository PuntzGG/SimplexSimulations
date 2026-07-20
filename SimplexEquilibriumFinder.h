#pragma once

#include <optional>
#include <vector>

#include "SimplexDynamicModel.h"
#include "SimplexState.h"

enum class SimplexEquilibriumLocation
{
    Interior,
    EdgeXY,
    EdgeXZ,
    EdgeYZ,
    VertexX,
    VertexY,
    VertexZ
};

struct SimplexEquilibrium final
{
    SimplexState state;
    double residual = 0.0;
    SimplexEquilibriumLocation location =
        SimplexEquilibriumLocation::Interior;
    bool isIsolated = true;
};

[[nodiscard]] const char* SimplexEquilibriumLocationName(
    SimplexEquilibriumLocation location
) noexcept;

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

// Finds residual-verified rest points using model-aware interior, edge, and
// vertex searches. Logit remains interior-only; Replicator includes invariant
// faces; equal-split Best Response checks its finite set of admissible targets.
class SimplexEquilibriumFinder final
{
public:
    [[nodiscard]] std::optional<std::vector<SimplexEquilibrium>> Find(
        const SimplexDynamicModel& dynamics,
        const SimplexEquilibriumSearchSettings& settings = {}
    ) const;
};
