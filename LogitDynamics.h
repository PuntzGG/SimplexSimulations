#pragma once

#include <optional>

#include "OpggParameters.h"
#include "SimplexDerivative.h"
#include "SimplexDynamicModel.h"
#include "SimplexState.h"

class LogitDynamics final : public SimplexDynamicModel
{
public:
    explicit LogitDynamics(OpggParameters parameters = {});

    [[nodiscard]] std::optional<SimplexDerivative> Evaluate(
        const SimplexState& state
    ) const override;

private:
    struct Payoffs final
    {
        double cooperators = 0.0;
        double defectors = 0.0;
        double loners = 0.0;

        [[nodiscard]] bool IsFinite() const noexcept;
    };

    [[nodiscard]] std::optional<Payoffs> EvaluatePayoffs(
        const SimplexState& state
    ) const;

    [[nodiscard]] double ComputeParticipationTerm(
        double lonerFrequency
    ) const noexcept;

    OpggParameters parameters_;
};
