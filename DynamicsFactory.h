#pragma once

#include <memory>

#include "BestResponseDynamics.h"
#include "DynamicsKind.h"
#include "OpggParameters.h"
#include "SimplexDynamicModel.h"

[[nodiscard]] std::unique_ptr<SimplexDynamicModel> CreateDynamics(
    DynamicsKind kind,
    const OpggParameters& parameters,
    const BestResponseSettings& bestResponseSettings = {}
);
