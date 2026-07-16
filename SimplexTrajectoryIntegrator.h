#pragma once

#include <optional>
#include <vector>

#include "SimplexDerivative.h"
#include "SimplexDynamicModel.h"
#include "SimplexState.h"
#include "TrajectorySettings.h"

class SimplexTrajectoryIntegrator final
{
public:
    [[nodiscard]] std::optional<std::vector<SimplexState>> Integrate(
        const SimplexDynamicModel& dynamics,
        const SimplexState& initialState,
        const TrajectorySettings& settings
    ) const;

private:
    [[nodiscard]] static std::optional<SimplexDerivative> EvaluateChecked(
        const SimplexDynamicModel& dynamics,
        const SimplexState& state
    );

    [[nodiscard]] static std::optional<SimplexState> AddScaledDerivative(
        const SimplexState& state,
        const SimplexDerivative& derivative,
        double scale
    );

    [[nodiscard]] static std::optional<SimplexState> TryRk4Step(
        const SimplexDynamicModel& dynamics,
        const SimplexState& state,
        double timeStep
    );

    [[nodiscard]] static std::optional<SimplexState> AdvanceWithSubdivision(
        const SimplexDynamicModel& dynamics,
        const SimplexState& state,
        double timeStep,
        int remainingSubdivisionDepth
    );
};
