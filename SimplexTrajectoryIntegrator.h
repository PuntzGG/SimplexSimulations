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
    [[nodiscard]] static SimplexState AddScaledDerivative(
        const SimplexState& state,
        const SimplexDerivative& derivative,
        double scale
    );

    [[nodiscard]] static SimplexDerivative CombineRk4Derivatives(
        const SimplexDerivative& k1,
        const SimplexDerivative& k2,
        const SimplexDerivative& k3,
        const SimplexDerivative& k4
    );
};