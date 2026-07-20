#pragma once

#include <cstdint>
#include <optional>

#include "OpggParameters.h"
#include "OpggPayoffEvaluator.h"
#include "SimplexDynamicModel.h"

struct BestResponseSettings final
{
    double absoluteTieTolerance = 1e-12;
    double relativeTieTolerance = 1e-10;

    [[nodiscard]] bool IsComputable() const noexcept;
};

struct BestResponseSelection final
{
    SimplexState target;
    std::uint8_t supportMask = 0U;
    int supportSize = 0;
};

class BestResponseDynamics final : public SimplexDynamicModel
{
public:
    explicit BestResponseDynamics(
        OpggParameters parameters = {},
        BestResponseSettings settings = {}
    );

    [[nodiscard]] std::optional<SimplexDerivative> Evaluate(
        const SimplexState& state
    ) const override;

    [[nodiscard]] DynamicsKind Kind() const noexcept override;

    [[nodiscard]] std::optional<StrategyPayoffs> Payoffs(
        const SimplexState& state
    ) const override;

    [[nodiscard]] std::optional<SimplexState> ResponseTarget(
        const SimplexState& state
    ) const override;

    [[nodiscard]] bool IsClassicallyDifferentiableAt(
        const SimplexState& state
    ) const noexcept override;

    [[nodiscard]] std::uint32_t RegionIdentifier(
        const SimplexState& state
    ) const noexcept override;

    [[nodiscard]] std::optional<BestResponseSelection> Select(
        const SimplexState& state
    ) const;

    [[nodiscard]] static std::optional<BestResponseSelection> SelectFromPayoffs(
        const StrategyPayoffs& payoffs,
        const BestResponseSettings& settings = {}
    );

private:
    OpggPayoffEvaluator payoffEvaluator_;
    BestResponseSettings settings_;
};
