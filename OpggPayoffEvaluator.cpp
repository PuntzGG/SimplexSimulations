#include "OpggPayoffEvaluator.h"

#include <cmath>

OpggPayoffEvaluator::OpggPayoffEvaluator(OpggParameters parameters)
    : parameters_(parameters)
{
}

std::optional<StrategyPayoffs> OpggPayoffEvaluator::Evaluate(
    const SimplexState& state
) const
{
    if (!parameters_.IsComputable() || !state.IsValid()) {
        return std::nullopt;
    }

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

    const StrategyPayoffs result{
        c * sigma * zToNMinusOne
            + (r - 1.0) * c * (1.0 - zToNMinusOne)
            - r * c * y * participationTerm,
        c * sigma * zToNMinusOne
            + (1.0 - d) * r * c * x * participationTerm,
        c * sigma
    };

    return result.IsFinite()
        ? std::optional<StrategyPayoffs>{ result }
        : std::nullopt;
}

const OpggParameters& OpggPayoffEvaluator::Parameters() const noexcept
{
    return parameters_;
}

double OpggPayoffEvaluator::ComputeParticipationTerm(
    double lonerFrequency
) const noexcept
{
    // Algebraically equivalent to
    // [1 - (1/n)(1-z^n)/(1-z)]/(1-z), evaluated as the stable polynomial
    // (1/n) * sum_{j=0}^{n-2} (n-1-j) z^j. At z=1 this gives the exact
    // limiting value (n-1)/2 without a removable singularity.
    const int n = parameters_.groupSize;
    double polynomial = 1.0;

    for (int coefficient = 2; coefficient <= n - 1; ++coefficient) {
        polynomial = polynomial * lonerFrequency
            + static_cast<double>(coefficient);
    }

    return polynomial / static_cast<double>(n);
}
