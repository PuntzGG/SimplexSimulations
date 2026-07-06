#include "SimplexTrajectoryIntegrator.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

std::optional<std::vector<SimplexState>> SimplexTrajectoryIntegrator::Integrate(
    const SimplexDynamicModel& dynamics,
    const SimplexState& initialState,
    const TrajectorySettings& settings
) const
{
    if (!settings.IsComputable()) {
        return std::nullopt;
    }

    const int requestedSteps = static_cast<int>(
        std::ceil(settings.totalTime / settings.timeStep)
        );

    const int stepCount = std::min(requestedSteps, settings.maxSteps);

    std::vector<SimplexState> states;
    states.reserve(static_cast<std::size_t>(stepCount) + 1);
    states.push_back(initialState);

    SimplexState currentState = initialState;

    for (int step = 0; step < stepCount; ++step) {
        const auto k1 = dynamics.Evaluate(currentState);
        if (!k1.has_value()) {
            return std::nullopt;
        }

        const SimplexState k2State = AddScaledDerivative(
            currentState,
            *k1,
            settings.timeStep * 0.5
        );

        const auto k2 = dynamics.Evaluate(k2State);
        if (!k2.has_value()) {
            return std::nullopt;
        }

        const SimplexState k3State = AddScaledDerivative(
            currentState,
            *k2,
            settings.timeStep * 0.5
        );

        const auto k3 = dynamics.Evaluate(k3State);
        if (!k3.has_value()) {
            return std::nullopt;
        }

        const SimplexState k4State = AddScaledDerivative(
            currentState,
            *k3,
            settings.timeStep
        );

        const auto k4 = dynamics.Evaluate(k4State);
        if (!k4.has_value()) {
            return std::nullopt;
        }

        const SimplexDerivative combinedDerivative = CombineRk4Derivatives(
            *k1,
            *k2,
            *k3,
            *k4
        );

        currentState = AddScaledDerivative(
            currentState,
            combinedDerivative,
            settings.timeStep
        );

        states.push_back(currentState);
    }

    return states;
}

SimplexState SimplexTrajectoryIntegrator::AddScaledDerivative(
    const SimplexState& state,
    const SimplexDerivative& derivative,
    double scale
)
{
    return SimplexState::Normalized(
        state.X() + scale * derivative.dx,
        state.Y() + scale * derivative.dy,
        state.Z() + scale * derivative.dz
    );
}

SimplexDerivative SimplexTrajectoryIntegrator::CombineRk4Derivatives(
    const SimplexDerivative& k1,
    const SimplexDerivative& k2,
    const SimplexDerivative& k3,
    const SimplexDerivative& k4
)
{
    return SimplexDerivative{
        (k1.dx + 2.0 * k2.dx + 2.0 * k3.dx + k4.dx) / 6.0,
        (k1.dy + 2.0 * k2.dy + 2.0 * k3.dy + k4.dy) / 6.0,
        (k1.dz + 2.0 * k2.dz + 2.0 * k3.dz + k4.dz) / 6.0
    };
}