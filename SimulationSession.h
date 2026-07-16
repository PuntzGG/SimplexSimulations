#pragma once

#include <optional>
#include <vector>

#include "LogitDynamics.h"
#include "LogitEquilibriumSweep.h"
#include "OpggParameters.h"
#include "SimplexEquilibriumFinder.h"
#include "SimplexState.h"
#include "SimplexTrajectoryIntegrator.h"
#include "TrajectorySettings.h"

class SimulationSession final
{
public:
    SimulationSession();

    [[nodiscard]] bool Initialize();
    [[nodiscard]] bool SetCurrentState(const SimplexState& state);
    [[nodiscard]] bool SetParameters(const OpggParameters& parameters);
    [[nodiscard]] bool SetTrajectorySettings(
        const TrajectorySettings& settings
    );

    [[nodiscard]] const SimplexState& CurrentState() const noexcept;
    [[nodiscard]] const OpggParameters& Parameters() const noexcept;
    [[nodiscard]] const TrajectorySettings& Settings() const noexcept;
    [[nodiscard]] const std::vector<SimplexState>& Trajectory() const noexcept;

    [[nodiscard]] std::optional<std::vector<SimplexEquilibrium>>
    FindEquilibria(
        const SimplexEquilibriumSearchSettings& settings = {}
    ) const;

    [[nodiscard]] std::optional<LogitEquilibriumSweepResult>
    GenerateEquilibriumSweep(
        const LogitEquilibriumSweepSettings& settings
    ) const;

private:
    [[nodiscard]] bool RebuildTrajectory();

    SimplexState currentState_;
    OpggParameters parameters_;
    TrajectorySettings trajectorySettings_;
    SimplexEquilibriumFinder equilibriumFinder_;
    LogitEquilibriumSweep equilibriumSweep_;
    LogitDynamics dynamics_;
    SimplexTrajectoryIntegrator trajectoryIntegrator_;
    std::vector<SimplexState> trajectory_;
};
