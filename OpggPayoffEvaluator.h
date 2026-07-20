#pragma once

#include <optional>

#include "OpggParameters.h"
#include "SimplexState.h"
#include "StrategyPayoffs.h"

class OpggPayoffEvaluator final
{
public:
    explicit OpggPayoffEvaluator(OpggParameters parameters = {});

    [[nodiscard]] std::optional<StrategyPayoffs> Evaluate(
        const SimplexState& state
    ) const;

    [[nodiscard]] const OpggParameters& Parameters() const noexcept;

private:
    [[nodiscard]] double ComputeParticipationTerm(
        double lonerFrequency
    ) const noexcept;

    OpggParameters parameters_;
};
