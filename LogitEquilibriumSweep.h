#pragma once

#include <optional>
#include <vector>

#include "OpggParameters.h"
#include "SimplexEquilibriumFinder.h"

enum class LogitEquilibriumSweepParameter
{
    LogitNoise,
    PunishmentFraction
};

struct LogitEquilibriumSweepSettings final
{
    LogitEquilibriumSweepParameter parameter =
        LogitEquilibriumSweepParameter::LogitNoise;

    double minimumParameter = 0.001;
    double maximumParameter = 1.0;
    int sampleCount = 81;

    double maximumBranchStepDistance = 0.1;

    SimplexEquilibriumSearchSettings equilibriumSearchSettings{};

    [[nodiscard]] bool IsComputable() const;
};

struct LogitEquilibriumSweepSample final
{
    double parameterValue;
    SimplexEquilibrium equilibrium;
};

struct LogitEquilibriumBranch final
{
    std::vector<LogitEquilibriumSweepSample> samples;
};

struct LogitEquilibriumSweepResult final
{
    LogitEquilibriumSweepParameter parameter;
    OpggParameters baselineParameters;

    double minimumParameter;
    double maximumParameter;

    std::vector<LogitEquilibriumBranch> branches;
};

class LogitEquilibriumSweep final
{
public:
    [[nodiscard]] std::optional<LogitEquilibriumSweepResult> Generate(
        const OpggParameters& baselineParameters,
        const LogitEquilibriumSweepSettings& settings
    ) const;

private:
    SimplexEquilibriumFinder equilibriumFinder_;
};