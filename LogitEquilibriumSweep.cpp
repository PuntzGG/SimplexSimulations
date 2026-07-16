#include "LogitEquilibriumSweep.h"

#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

#include "LogitDynamics.h"

namespace
{
    struct ParameterSample final
    {
        double parameterValue = 0.0;
        std::vector<SimplexEquilibrium> equilibria;
    };

    constexpr std::size_t kNoBranch =
        std::numeric_limits<std::size_t>::max();

    [[nodiscard]] double SquaredSimplexDistance(
        const SimplexState& left,
        const SimplexState& right
    ) noexcept
    {
        const double dx = left.X() - right.X();
        const double dy = left.Y() - right.Y();
        const double dz = left.Z() - right.Z();
        return dx * dx + dy * dy + dz * dz;
    }

    [[nodiscard]] double SampleParameterValue(
        const LogitEquilibriumSweepSettings& settings,
        int sampleIndex
    )
    {
        if (sampleIndex == 0) {
            return settings.minimumParameter;
        }
        if (sampleIndex == settings.sampleCount - 1) {
            return settings.maximumParameter;
        }

        const double progress =
            static_cast<double>(sampleIndex)
            / static_cast<double>(settings.sampleCount - 1);

        if (settings.parameter
            == LogitEquilibriumSweepParameter::LogitNoise) {
            const double minimumLog = std::log(settings.minimumParameter);
            const double maximumLog = std::log(settings.maximumParameter);
            return std::exp(
                minimumLog + progress * (maximumLog - minimumLog)
            );
        }

        return settings.minimumParameter
            + progress
                * (settings.maximumParameter - settings.minimumParameter);
    }

    void ApplySweptParameter(
        OpggParameters& parameters,
        LogitEquilibriumSweepParameter parameter,
        double value
    ) noexcept
    {
        switch (parameter) {
        case LogitEquilibriumSweepParameter::LogitNoise:
            parameters.logitNoise = value;
            return;
        case LogitEquilibriumSweepParameter::PunishmentFraction:
            parameters.punishmentFraction = value;
            return;
        }
    }

    [[nodiscard]] std::vector<int> FindClosestIndices(
        const std::vector<SimplexEquilibrium>& origins,
        const std::vector<SimplexEquilibrium>& targets
    )
    {
        std::vector<int> closestIndices(origins.size(), -1);

        for (std::size_t originIndex = 0;
             originIndex < origins.size();
             ++originIndex) {
            double smallestDistanceSquared =
                std::numeric_limits<double>::infinity();

            for (std::size_t targetIndex = 0;
                 targetIndex < targets.size();
                 ++targetIndex) {
                const double distanceSquared = SquaredSimplexDistance(
                    origins[originIndex].state,
                    targets[targetIndex].state
                );
                if (distanceSquared < smallestDistanceSquared) {
                    smallestDistanceSquared = distanceSquared;
                    closestIndices[originIndex] =
                        static_cast<int>(targetIndex);
                }
            }
        }

        return closestIndices;
    }

    [[nodiscard]] std::size_t StartBranch(
        LogitEquilibriumSweepResult& result,
        double parameterValue,
        const SimplexEquilibrium& equilibrium
    )
    {
        result.branches.push_back(LogitEquilibriumBranch{});
        result.branches.back().samples.push_back(
            LogitEquilibriumSweepSample{ parameterValue, equilibrium }
        );
        return result.branches.size() - 1U;
    }

    void BuildBranches(
        const std::vector<ParameterSample>& parameterSamples,
        double maximumBranchStepDistance,
        LogitEquilibriumSweepResult& result
    )
    {
        if (parameterSamples.empty()) {
            return;
        }

        const ParameterSample& firstSample = parameterSamples.front();
        std::vector<std::size_t> previousBranchIndices;
        previousBranchIndices.reserve(firstSample.equilibria.size());

        for (const SimplexEquilibrium& equilibrium : firstSample.equilibria) {
            previousBranchIndices.push_back(StartBranch(
                result,
                firstSample.parameterValue,
                equilibrium
            ));
        }

        const double maximumDistanceSquared =
            maximumBranchStepDistance * maximumBranchStepDistance;

        for (std::size_t sampleIndex = 1;
             sampleIndex < parameterSamples.size();
             ++sampleIndex) {
            const ParameterSample& previousSample =
                parameterSamples[sampleIndex - 1U];
            const ParameterSample& currentSample =
                parameterSamples[sampleIndex];

            const std::vector<int> closestCurrentForPrevious =
                FindClosestIndices(
                    previousSample.equilibria,
                    currentSample.equilibria
                );
            const std::vector<int> closestPreviousForCurrent =
                FindClosestIndices(
                    currentSample.equilibria,
                    previousSample.equilibria
                );

            std::vector<std::size_t> currentBranchIndices(
                currentSample.equilibria.size(),
                kNoBranch
            );

            for (std::size_t previousIndex = 0;
                 previousIndex < previousSample.equilibria.size();
                 ++previousIndex) {
                const int currentIndex =
                    closestCurrentForPrevious[previousIndex];
                if (currentIndex < 0) {
                    continue;
                }

                const std::size_t currentRootIndex =
                    static_cast<std::size_t>(currentIndex);
                if (currentRootIndex >= closestPreviousForCurrent.size()
                    || closestPreviousForCurrent[currentRootIndex]
                        != static_cast<int>(previousIndex)) {
                    continue;
                }

                const double distanceSquared = SquaredSimplexDistance(
                    previousSample.equilibria[previousIndex].state,
                    currentSample.equilibria[currentRootIndex].state
                );
                if (!std::isfinite(distanceSquared)
                    || distanceSquared > maximumDistanceSquared) {
                    continue;
                }

                const std::size_t branchIndex =
                    previousBranchIndices[previousIndex];
                result.branches[branchIndex].samples.push_back(
                    LogitEquilibriumSweepSample{
                        currentSample.parameterValue,
                        currentSample.equilibria[currentRootIndex]
                    }
                );
                currentBranchIndices[currentRootIndex] = branchIndex;
            }

            for (std::size_t currentIndex = 0;
                 currentIndex < currentSample.equilibria.size();
                 ++currentIndex) {
                if (currentBranchIndices[currentIndex] != kNoBranch) {
                    continue;
                }

                currentBranchIndices[currentIndex] = StartBranch(
                    result,
                    currentSample.parameterValue,
                    currentSample.equilibria[currentIndex]
                );
            }

            previousBranchIndices = std::move(currentBranchIndices);
        }
    }
}

bool LogitEquilibriumSweepSettings::IsComputable() const noexcept
{
    if (!std::isfinite(minimumParameter)
        || !std::isfinite(maximumParameter)
        || !std::isfinite(maximumBranchStepDistance)
        || sampleCount < 2
        || minimumParameter >= maximumParameter
        || maximumBranchStepDistance <= 0.0
        || !equilibriumSearchSettings.IsComputable()) {
        return false;
    }

    switch (parameter) {
    case LogitEquilibriumSweepParameter::LogitNoise:
        return minimumParameter > 0.0;
    case LogitEquilibriumSweepParameter::PunishmentFraction:
        return minimumParameter >= 0.0 && maximumParameter <= 1.0;
    }

    return false;
}

std::optional<LogitEquilibriumSweepResult> LogitEquilibriumSweep::Generate(
    const OpggParameters& baselineParameters,
    const LogitEquilibriumSweepSettings& settings
) const
{
    if (!baselineParameters.IsComputable() || !settings.IsComputable()) {
        return std::nullopt;
    }

    std::vector<ParameterSample> parameterSamples;
    parameterSamples.reserve(static_cast<std::size_t>(settings.sampleCount));

    for (int sampleIndex = 0;
         sampleIndex < settings.sampleCount;
         ++sampleIndex) {
        const double parameterValue = SampleParameterValue(
            settings,
            sampleIndex
        );
        if (!std::isfinite(parameterValue)) {
            return std::nullopt;
        }

        OpggParameters sampledParameters = baselineParameters;
        ApplySweptParameter(
            sampledParameters,
            settings.parameter,
            parameterValue
        );
        if (!sampledParameters.IsComputable()) {
            return std::nullopt;
        }

        const LogitDynamics dynamics(sampledParameters);
        auto equilibria = equilibriumFinder_.Find(
            dynamics,
            settings.equilibriumSearchSettings
        );
        if (!equilibria.has_value()) {
            return std::nullopt;
        }

        parameterSamples.push_back(ParameterSample{
            parameterValue,
            std::move(*equilibria)
        });
    }

    LogitEquilibriumSweepResult result{
        settings.parameter,
        baselineParameters,
        settings.minimumParameter,
        settings.maximumParameter,
        {}
    };
    BuildBranches(
        parameterSamples,
        settings.maximumBranchStepDistance,
        result
    );

    return result;
}
