#pragma once

#include <optional>

#include "OpggParameters.h"
#include "SimplexDerivative.h"
#include "SimplexState.h"
#include "SimplexDynamicModel.h"

class LogitDynamics final : public SimplexDynamicModel
{
public:
    explicit LogitDynamics(OpggParameters parameters = {});

    [[nodiscard]] std::optional<SimplexDerivative> Evaluate(
        const SimplexState& state
    ) const override;

private:
    struct Payoffs
    {
        double cooperators = 0.0;
        double defectors = 0.0;
        double loners = 0.0;
    };

    [[nodiscard]] Payoffs EvaluatePayoffs(const SimplexState& state) const;
    [[nodiscard]] double ComputeParticipationTerm(double lonerFrequency) const;

    OpggParameters parameters_;
};