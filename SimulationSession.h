#pragma once

#include <vector>
#include <optional>

#include "SimplexEquilibriumFinder.h"
#include "LogitDynamics.h"
#include "OpggParameters.h"
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

    [[nodiscard]] const SimplexState& CurrentState() const;
    [[nodiscard]] const OpggParameters& Parameters() const;
    [[nodiscard]] const TrajectorySettings& Settings() const;
    [[nodiscard]] const std::vector<SimplexState>& Trajectory() const;
    [[nodiscard]] std::optional<std::vector<SimplexEquilibrium>> FindEquilibria(
        const SimplexEquilibriumSearchSettings& settings = {}
    ) const;

private:
    [[nodiscard]] bool RebuildTrajectory();

    SimplexState currentState_;
    OpggParameters parameters_;
    TrajectorySettings trajectorySettings_;
    SimplexEquilibriumFinder equilibriumFinder_;
    LogitDynamics dynamics_;
    SimplexTrajectoryIntegrator trajectoryIntegrator_;
    std::vector<SimplexState> trajectory_;
};