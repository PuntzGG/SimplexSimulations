#pragma once

#include <optional>

#include "OpggParameters.h"
#include "OpggPayoffEvaluator.h"
#include "SimplexDynamicModel.h"

class ReplicatorDynamics final : public SimplexDynamicModel
{
public:
    explicit ReplicatorDynamics(OpggParameters parameters = {});

    [[nodiscard]] std::optional<SimplexDerivative> Evaluate(
        const SimplexState& state
    ) const override;

    [[nodiscard]] DynamicsKind Kind() const noexcept override;

    [[nodiscard]] std::optional<StrategyPayoffs> Payoffs(
        const SimplexState& state
    ) const override;

private:
    OpggPayoffEvaluator payoffEvaluator_;
};
