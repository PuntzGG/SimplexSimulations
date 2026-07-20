#include "BestResponseDynamics.h"

#include <algorithm>
#include <array>
#include <cmath>

bool BestResponseSettings::IsComputable() const noexcept
{
    return std::isfinite(absoluteTieTolerance)
        && std::isfinite(relativeTieTolerance)
        && absoluteTieTolerance >= 0.0
        && relativeTieTolerance >= 0.0;
}

BestResponseDynamics::BestResponseDynamics(
    OpggParameters parameters,
    BestResponseSettings settings
)
    : payoffEvaluator_(parameters),
      settings_(settings)
{
}

std::optional<SimplexDerivative> BestResponseDynamics::Evaluate(
    const SimplexState& state
) const
{
    const auto selection = Select(state);
    if (!selection.has_value()) {
        return std::nullopt;
    }

    const SimplexDerivative derivative{
        selection->target.X() - state.X(),
        selection->target.Y() - state.Y(),
        selection->target.Z() - state.Z()
    };

    return derivative.IsTangent()
        ? std::optional<SimplexDerivative>{ derivative }
        : std::nullopt;
}

DynamicsKind BestResponseDynamics::Kind() const noexcept
{
    return DynamicsKind::EqualSplitBestResponse;
}

std::optional<StrategyPayoffs> BestResponseDynamics::Payoffs(
    const SimplexState& state
) const
{
    return payoffEvaluator_.Evaluate(state);
}

std::optional<SimplexState> BestResponseDynamics::ResponseTarget(
    const SimplexState& state
) const
{
    const auto selection = Select(state);
    return selection.has_value()
        ? std::optional<SimplexState>{ selection->target }
        : std::nullopt;
}

bool BestResponseDynamics::IsClassicallyDifferentiableAt(
    const SimplexState& state
) const noexcept
{
    const auto selection = Select(state);
    return selection.has_value() && selection->supportSize == 1;
}

std::uint32_t BestResponseDynamics::RegionIdentifier(
    const SimplexState& state
) const noexcept
{
    const auto selection = Select(state);
    return selection.has_value()
        ? static_cast<std::uint32_t>(selection->supportMask)
        : 0U;
}

std::optional<BestResponseSelection> BestResponseDynamics::Select(
    const SimplexState& state
) const
{
    const auto payoffs = payoffEvaluator_.Evaluate(state);
    if (!payoffs.has_value()) {
        return std::nullopt;
    }

    return SelectFromPayoffs(*payoffs, settings_);
}

std::optional<BestResponseSelection>
BestResponseDynamics::SelectFromPayoffs(
    const StrategyPayoffs& payoffs,
    const BestResponseSettings& settings
)
{
    if (!payoffs.IsFinite() || !settings.IsComputable()) {
        return std::nullopt;
    }

    const std::array<double, 3> values = payoffs.AsArray();
    const double maximum = payoffs.Maximum();
    const double tolerance = settings.absoluteTieTolerance
        + settings.relativeTieTolerance
            * std::max(1.0, payoffs.MaximumMagnitude());

    std::uint8_t supportMask = 0U;
    int supportSize = 0;
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (maximum - values[index] <= tolerance) {
            supportMask |= static_cast<std::uint8_t>(1U << index);
            ++supportSize;
        }
    }

    if (supportSize <= 0) {
        return std::nullopt;
    }

    const double mass = 1.0 / static_cast<double>(supportSize);
    const double x = (supportMask & 0x1U) != 0U ? mass : 0.0;
    const double y = (supportMask & 0x2U) != 0U ? mass : 0.0;
    const double z = (supportMask & 0x4U) != 0U ? mass : 0.0;
    const auto target = SimplexState::TryCreate(x, y, z, 1e-12);
    if (!target.has_value()) {
        return std::nullopt;
    }

    return BestResponseSelection{ *target, supportMask, supportSize };
}
