#pragma once

#include <cstdint>
#include <optional>

#include "DynamicsKind.h"
#include "SimplexDerivative.h"
#include "SimplexState.h"
#include "StrategyPayoffs.h"

class SimplexDynamicModel
{
public:
    virtual ~SimplexDynamicModel() = default;

    [[nodiscard]] virtual std::optional<SimplexDerivative> Evaluate(
        const SimplexState& state
    ) const = 0;

    [[nodiscard]] virtual DynamicsKind Kind() const noexcept
    {
        return DynamicsKind::Custom;
    }

    [[nodiscard]] virtual std::optional<StrategyPayoffs> Payoffs(
        const SimplexState&
    ) const
    {
        return std::nullopt;
    }

    [[nodiscard]] virtual std::optional<SimplexState> ResponseTarget(
        const SimplexState&
    ) const
    {
        return std::nullopt;
    }

    [[nodiscard]] virtual bool IsClassicallyDifferentiableAt(
        const SimplexState&
    ) const noexcept
    {
        return CapabilitiesFor(Kind()).hasClassicalJacobian;
    }

    [[nodiscard]] virtual std::uint32_t RegionIdentifier(
        const SimplexState&
    ) const noexcept
    {
        return 0U;
    }
};
