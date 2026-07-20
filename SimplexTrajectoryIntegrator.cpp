#include "SimplexTrajectoryIntegrator.h"

#include <algorithm>
#include <cmath>

namespace
{
    constexpr double kStageEpsilon = 1e-10;
    constexpr double kMinimumInternalStep = 1e-12;
    constexpr int kMaximumSubdivisionDepth = 20;
}

std::optional<std::vector<SimplexState>>
SimplexTrajectoryIntegrator::Integrate(
    const SimplexDynamicModel& dynamics,
    const SimplexState& initialState,
    const TrajectorySettings& settings
) const
{
    const auto requestedSteps = settings.RequestedStepCount();
    if (!requestedSteps.has_value() || !initialState.IsValid()) {
        return std::nullopt;
    }

    std::vector<SimplexState> states;
    states.reserve(static_cast<std::size_t>(*requestedSteps) + 1U);
    states.push_back(initialState);

    SimplexState currentState = initialState;
    double elapsedTime = 0.0;

    for (int step = 0; step < *requestedSteps; ++step) {
        const double remainingTime = settings.totalTime - elapsedTime;
        const double stepSize = std::min(settings.timeStep, remainingTime);
        if (!std::isfinite(stepSize) || stepSize <= 0.0) {
            return std::nullopt;
        }

        const auto nextState = AdvanceOneStep(
            dynamics,
            currentState,
            stepSize
        );
        if (!nextState.has_value()) {
            return std::nullopt;
        }

        currentState = *nextState;
        elapsedTime += stepSize;
        states.push_back(currentState);
    }

    return states;
}

std::optional<SimplexState> SimplexTrajectoryIntegrator::AdvanceOneStep(
    const SimplexDynamicModel& dynamics,
    const SimplexState& state,
    double timeStep
) const
{
    if (!state.IsValid() || !std::isfinite(timeStep) || timeStep <= 0.0) {
        return std::nullopt;
    }

    return AdvanceWithSubdivision(
        dynamics,
        state,
        timeStep,
        kMaximumSubdivisionDepth
    );
}

std::optional<SimplexDerivative>
SimplexTrajectoryIntegrator::EvaluateChecked(
    const SimplexDynamicModel& dynamics,
    const SimplexState& state
)
{
    const auto derivative = dynamics.Evaluate(state);
    if (!derivative.has_value()
        || !derivative->IsFinite()
        || !derivative->IsTangent()) {
        return std::nullopt;
    }

    return derivative;
}

std::optional<SimplexState>
SimplexTrajectoryIntegrator::AddScaledDerivative(
    const SimplexState& state,
    const SimplexDerivative& derivative,
    double scale
)
{
    if (!std::isfinite(scale) || !derivative.IsTangent()) {
        return std::nullopt;
    }

    const double x = state.X() + scale * derivative.dx;
    const double y = state.Y() + scale * derivative.dy;
    const double z = 1.0 - x - y;

    return SimplexState::TryCreate(x, y, z, kStageEpsilon);
}

std::optional<SimplexState> SimplexTrajectoryIntegrator::TryRk4Step(
    const SimplexDynamicModel& dynamics,
    const SimplexState& state,
    double timeStep
)
{
    const auto k1 = EvaluateChecked(dynamics, state);
    if (!k1.has_value()) {
        return std::nullopt;
    }

    const auto k2State = AddScaledDerivative(state, *k1, timeStep * 0.5);
    if (!k2State.has_value()) {
        return std::nullopt;
    }

    const auto k2 = EvaluateChecked(dynamics, *k2State);
    if (!k2.has_value()) {
        return std::nullopt;
    }

    const auto k3State = AddScaledDerivative(state, *k2, timeStep * 0.5);
    if (!k3State.has_value()) {
        return std::nullopt;
    }

    const auto k3 = EvaluateChecked(dynamics, *k3State);
    if (!k3.has_value()) {
        return std::nullopt;
    }

    const auto k4State = AddScaledDerivative(state, *k3, timeStep);
    if (!k4State.has_value()) {
        return std::nullopt;
    }

    const auto k4 = EvaluateChecked(dynamics, *k4State);
    if (!k4.has_value()) {
        return std::nullopt;
    }

    const SimplexDerivative combined{
        (k1->dx + 2.0 * k2->dx + 2.0 * k3->dx + k4->dx) / 6.0,
        (k1->dy + 2.0 * k2->dy + 2.0 * k3->dy + k4->dy) / 6.0,
        (k1->dz + 2.0 * k2->dz + 2.0 * k3->dz + k4->dz) / 6.0
    };

    return AddScaledDerivative(state, combined, timeStep);
}

std::optional<SimplexState>
SimplexTrajectoryIntegrator::AdvanceWithSubdivision(
    const SimplexDynamicModel& dynamics,
    const SimplexState& state,
    double timeStep,
    int remainingSubdivisionDepth
)
{
    const auto fullStep = TryRk4Step(dynamics, state, timeStep);
    if (fullStep.has_value()) {
        return fullStep;
    }

    if (remainingSubdivisionDepth <= 0
        || timeStep * 0.5 < kMinimumInternalStep) {
        return std::nullopt;
    }

    const double halfStep = timeStep * 0.5;
    const auto midpoint = AdvanceWithSubdivision(
        dynamics,
        state,
        halfStep,
        remainingSubdivisionDepth - 1
    );
    if (!midpoint.has_value()) {
        return std::nullopt;
    }

    return AdvanceWithSubdivision(
        dynamics,
        *midpoint,
        halfStep,
        remainingSubdivisionDepth - 1
    );
}
