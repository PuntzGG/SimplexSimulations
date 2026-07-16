#include "LogitDynamics.h"

#include <algorithm>
#include <array>
#include <cmath>

LogitDynamics::LogitDynamics(OpggParameters parameters)
    : parameters_(parameters)
{
}

bool LogitDynamics::Payoffs::IsFinite() const noexcept
{
    return std::isfinite(cooperators)
        && std::isfinite(defectors)
        && std::isfinite(loners);
}

std::optional<SimplexDerivative> LogitDynamics::Evaluate(
    const SimplexState& state
) const
{
    if (!parameters_.IsComputable() || !state.IsValid()) {
        return std::nullopt;
    }

    const auto payoffs = EvaluatePayoffs(state);
    if (!payoffs.has_value()) {
        return std::nullopt;
    }

    const double maximumPayoff = std::max({
        payoffs->cooperators,
        payoffs->defectors,
        payoffs->loners
    });

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

    const double targetX = weights[0] / totalWeight;
    const double targetY = weights[1] / totalWeight;
    const double targetZ = 1.0 - targetX - targetY;

    const SimplexDerivative derivative{
        targetX - state.X(),
        targetY - state.Y(),
        targetZ - state.Z()
    };

    if (!derivative.IsTangent()) {
        return std::nullopt;
    }

    return derivative;
}

std::optional<LogitDynamics::Payoffs> LogitDynamics::EvaluatePayoffs(
    const SimplexState& state
) const
{
    const double x = state.X();
    const double y = state.Y();
    const double z = state.Z();

    const double c = parameters_.contributionCost;
    const double r = parameters_.multiplicationFactor;
    const double sigma = parameters_.lonerPayoffMultiplier;
    const double d = parameters_.punishmentFraction;
    const int n = parameters_.groupSize;

    const double zToNMinusOne = std::pow(z, n - 1);
    const double participationTerm = ComputeParticipationTerm(z);

    const Payoffs result{
        c * sigma * zToNMinusOne
            + (r - 1.0) * c * (1.0 - zToNMinusOne)
            - r * c * y * participationTerm,
        c * sigma * zToNMinusOne
            + (1.0 - d) * r * c * x * participationTerm,
        c * sigma
    };

    if (!result.IsFinite()) {
        return std::nullopt;
    }

    return result;
}

double LogitDynamics::ComputeParticipationTerm(
    double lonerFrequency
) const noexcept
{
    // Algebraically equivalent to
    // [1 - (1/n)(1-z^n)/(1-z)]/(1-z), but evaluated as a polynomial:
    // (1/n) * sum_{j=0}^{n-2} (n-1-j) z^j.
    // This form is stable at z=1 and gives the exact limit (n-1)/2.
    const int n = parameters_.groupSize;
    double polynomial = 1.0;

    for (int coefficient = 2; coefficient <= n - 1; ++coefficient) {
        polynomial = polynomial * lonerFrequency
            + static_cast<double>(coefficient);
    }

    return polynomial / static_cast<double>(n);
}
