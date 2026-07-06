#pragma once

#include <optional>

#include "SimplexDerivative.h"
#include "SimplexState.h"

class SimplexDynamicModel
{
public:
    virtual ~SimplexDynamicModel() = default;

    [[nodiscard]] virtual std::optional<SimplexDerivative> Evaluate(
        const SimplexState& state
    ) const = 0;
};