#include "LogitDynamics.h"

#include <algorithm>
#include <array>
#include <cmath>

LogitDynamics::LogitDynamics(OpggParameters parameters)
    : parameters_(parameters),
      payoffEvaluator_(parameters)
{
}

std::optional<SimplexDerivative> LogitDynamics::Evaluate(
    const SimplexState& state
) const
{
    const auto target = ResponseTarget(state);
    if (!target.has_value()) {
        return std::nullopt;
    }

    const SimplexDerivative derivative{
        target->X() - state.X(),
        target->Y() - state.Y(),
        target->Z() - state.Z()
    };

    return derivative.IsTangent()
        ? std::optional<SimplexDerivative>{ derivative }
        : std::nullopt;
}

DynamicsKind LogitDynamics::Kind() const noexcept
{
    return DynamicsKind::Logit;
}

std::optional<StrategyPayoffs> LogitDynamics::Payoffs(
    const SimplexState& state
) const
{
    return payoffEvaluator_.Evaluate(state);
}

std::optional<SimplexState> LogitDynamics::ResponseTarget(
    const SimplexState& state
) const
{
    if (!parameters_.IsComputable() || !state.IsValid()) {
        return std::nullopt;
    }

    const auto payoffs = payoffEvaluator_.Evaluate(state);
    if (!payoffs.has_value()) {
        return std::nullopt;
    }

    const double maximumPayoff = payoffs->Maximum();

    // Subtract before dividing by eta. All exponents are non-positive, so
    // overflow is impossible even for very small positive eta.
    const std::array<double, 3> weights{
        std::exp((payoffs->cooperators - maximumPayoff) / parameters_.logitNoise),
        std::exp((payoffs->defectors - maximumPayoff) / parameters_.logitNoise),
        std::exp((payoffs->loners - maximumPayoff) / parameters_.logitNoise)
    };

    const double totalWeight = weights[0] + weights[1] + weights[2];
    if (!std::isfinite(totalWeight) || totalWeight <= 0.0) {
        return std::nullopt;
    }

    return SimplexState::Normalized(weights[0], weights[1], weights[2]);
}
