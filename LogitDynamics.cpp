#include "LogitDynamics.h"

#include <algorithm>
#include <cmath>

LogitDynamics::LogitDynamics(OpggParameters parameters)
    : parameters_(parameters)
{
}

std::optional<SimplexDerivative> LogitDynamics::Evaluate(
    const SimplexState& state
) const
{
    if (!parameters_.IsComputable()) {
        return std::nullopt;
    }

    const Payoffs payoffs = EvaluatePayoffs(state);

    const double scaledCooperatorPayoff = payoffs.cooperators / parameters_.logitNoise;
    const double scaledDefectorPayoff = payoffs.defectors / parameters_.logitNoise;
    const double scaledLonerPayoff = payoffs.loners / parameters_.logitNoise;

    const double maxScaledPayoff = std::max({
        scaledCooperatorPayoff,
        scaledDefectorPayoff,
        scaledLonerPayoff
        });

    const double cooperatorWeight = std::exp(scaledCooperatorPayoff - maxScaledPayoff);
    const double defectorWeight = std::exp(scaledDefectorPayoff - maxScaledPayoff);
    const double lonerWeight = std::exp(scaledLonerPayoff - maxScaledPayoff);

    const double totalWeight = cooperatorWeight + defectorWeight + lonerWeight;

    if (totalWeight <= 0.0 || !std::isfinite(totalWeight)) {
        return std::nullopt;
    }

    const double targetCooperators = cooperatorWeight / totalWeight;
    const double targetDefectors = defectorWeight / totalWeight;
    const double targetLoners = lonerWeight / totalWeight;

    return SimplexDerivative{
        targetCooperators - state.X(),
        targetDefectors - state.Y(),
        targetLoners - state.Z()
    };
}

LogitDynamics::Payoffs LogitDynamics::EvaluatePayoffs(
    const SimplexState& state
) const
{
    const double x = state.X();
    const double y = state.Y();
    const double z = state.Z();

    const double c = parameters_.contributionCost;
    const double r = parameters_.multiplicationFactor;
    const double sigma = parameters_.lonerPayoffMultiplier;
    const double v = parameters_.punishmentFraction;
    const int n = parameters_.groupSize;

    const double zToNMinusOne = std::pow(z, n - 1);
    const double participationTerm = ComputeParticipationTerm(z);

    const double cooperatorPayoff =
        c * sigma * zToNMinusOne
        + (r - 1.0) * c * (1.0 - zToNMinusOne)
        - r * c * y * participationTerm;

    const double defectorPayoff =
        c * sigma * zToNMinusOne
        + (1.0 - v) * r * c * x * participationTerm;

    const double lonerPayoff = c * sigma;

    return Payoffs{
        cooperatorPayoff,
        defectorPayoff,
        lonerPayoff
    };
}

double LogitDynamics::ComputeParticipationTerm(double lonerFrequency) const
{
    constexpr double kNearPureLonerEpsilon = 1e-8;

    const double z = lonerFrequency;
    const int n = parameters_.groupSize;
    const double oneMinusZ = 1.0 - z;

    if (std::abs(oneMinusZ) <= kNearPureLonerEpsilon) {
        return 0.5 * static_cast<double>(n - 1);
    }

    const double geometricSum = (1.0 - std::pow(z, n)) / oneMinusZ;

    return (1.0 - (geometricSum / static_cast<double>(n))) / oneMinusZ;
}