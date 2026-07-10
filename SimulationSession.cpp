#include "SimulationSession.h"

#include <utility>

SimulationSession::SimulationSession()
    : currentState_(SimplexState::Normalized(1.0, 1.0, 1.0))
    , parameters_()
    , trajectorySettings_()
    , dynamics_(parameters_)
{
}

bool SimulationSession::Initialize()
{
    return RebuildTrajectory();
}

bool SimulationSession::SetCurrentState(const SimplexState& state)
{
    if (!state.IsValid()) {
        return false;
    }

    const auto trajectory = trajectoryIntegrator_.Integrate(
        dynamics_,
        state,
        trajectorySettings_
    );

    if (!trajectory.has_value()) {
        return false;
    }

    currentState_ = state;
    trajectory_ = std::move(*trajectory);
    return true;
}

bool SimulationSession::SetParameters(const OpggParameters& parameters)
{
    if (!parameters.IsComputable()) {
        return false;
    }

    LogitDynamics candidateDynamics(parameters);

    const auto trajectory = trajectoryIntegrator_.Integrate(
        candidateDynamics,
        currentState_,
        trajectorySettings_
    );

    if (!trajectory.has_value()) {
        return false;
    }

    parameters_ = parameters;
    dynamics_ = std::move(candidateDynamics);
    trajectory_ = std::move(*trajectory);
    return true;
}

bool SimulationSession::SetTrajectorySettings(
    const TrajectorySettings& settings
)
{
    if (!settings.IsComputable()) {
        return false;
    }

    const auto trajectory = trajectoryIntegrator_.Integrate(
        dynamics_,
        currentState_,
        settings
    );

    if (!trajectory.has_value()) {
        return false;
    }

    trajectorySettings_ = settings;
    trajectory_ = std::move(*trajectory);
    return true;
}

const SimplexState& SimulationSession::CurrentState() const
{
    return currentState_;
}

const OpggParameters& SimulationSession::Parameters() const
{
    return parameters_;
}

const TrajectorySettings& SimulationSession::Settings() const
{
    return trajectorySettings_;
}

const std::vector<SimplexState>& SimulationSession::Trajectory() const
{
    return trajectory_;
}

bool SimulationSession::RebuildTrajectory()
{
    const auto trajectory = trajectoryIntegrator_.Integrate(
        dynamics_,
        currentState_,
        trajectorySettings_
    );

    if (!trajectory.has_value()) {
        return false;
    }

    trajectory_ = std::move(*trajectory);
    return true;
}