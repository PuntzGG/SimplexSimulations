#include "ReplicatorDynamics.h"

#include <cmath>

ReplicatorDynamics::ReplicatorDynamics(OpggParameters parameters)
    : payoffEvaluator_(parameters)
{
}

std::optional<SimplexDerivative> ReplicatorDynamics::Evaluate(
    const SimplexState& state
) const
{
    const auto payoffs = payoffEvaluator_.Evaluate(state);
    if (!payoffs.has_value()) {
        return std::nullopt;
    }

    const double averagePayoff =
        state.X() * payoffs->cooperators
        + state.Y() * payoffs->defectors
        + state.Z() * payoffs->loners;
    if (!std::isfinite(averagePayoff)) {
        return std::nullopt;
    }

    const SimplexDerivative derivative{
        state.X() * (payoffs->cooperators - averagePayoff),
        state.Y() * (payoffs->defectors - averagePayoff),
        state.Z() * (payoffs->loners - averagePayoff)
    };

    return derivative.IsTangent()
        ? std::optional<SimplexDerivative>{ derivative }
        : std::nullopt;
}

DynamicsKind ReplicatorDynamics::Kind() const noexcept
{
    return DynamicsKind::Replicator;
}

std::optional<StrategyPayoffs> ReplicatorDynamics::Payoffs(
    const SimplexState& state
) const
{
    return payoffEvaluator_.Evaluate(state);
}
