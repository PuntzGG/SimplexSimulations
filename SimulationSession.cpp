#include "SimulationSession.h"

#include <utility>

#include "DynamicsFactory.h"

SimulationSession::SimulationSession()
    : currentState_(SimplexState::Normalized(1.0, 1.0, 1.0)),
      parameters_(),
      trajectorySettings_(),
      dynamics_(CreateDynamics(
          dynamicsKind_,
          parameters_,
          bestResponseSettings_
      ))
{
}

bool SimulationSession::Initialize()
{
    if (!parameters_.IsComputable()
        || !trajectorySettings_.IsComputable()
        || dynamics_ == nullptr) {
        return false;
    }
    return RebuildTrajectory();
}

bool SimulationSession::SetCurrentState(const SimplexState& state)
{
    if (!state.IsValid()) {
        return false;
    }

    auto candidateTrajectory = trajectoryIntegrator_.Integrate(
        *dynamics_,
        state,
        trajectorySettings_
    );
    if (!candidateTrajectory.has_value()) {
        return false;
    }

    currentState_ = state;
    trajectory_ = std::move(*candidateTrajectory);
    return true;
}

bool SimulationSession::SetParameters(const OpggParameters& parameters)
{
    if (!parameters.IsComputable()) {
        return false;
    }

    auto candidateDynamics = CreateDynamics(
        dynamicsKind_,
        parameters,
        bestResponseSettings_
    );
    if (candidateDynamics == nullptr) {
        return false;
    }

    auto candidateTrajectory = trajectoryIntegrator_.Integrate(
        *candidateDynamics,
        currentState_,
        trajectorySettings_
    );
    if (!candidateTrajectory.has_value()) {
        return false;
    }

    parameters_ = parameters;
    dynamics_ = std::move(candidateDynamics);
    trajectory_ = std::move(*candidateTrajectory);
    return true;
}

bool SimulationSession::SetDynamicsKind(DynamicsKind kind)
{
    if (kind == DynamicsKind::Custom || kind == dynamicsKind_) {
        return kind == dynamicsKind_;
    }

    auto candidateDynamics = CreateDynamics(
        kind,
        parameters_,
        bestResponseSettings_
    );
    if (candidateDynamics == nullptr) {
        return false;
    }

    auto candidateTrajectory = trajectoryIntegrator_.Integrate(
        *candidateDynamics,
        currentState_,
        trajectorySettings_
    );
    if (!candidateTrajectory.has_value()) {
        return false;
    }

    dynamicsKind_ = kind;
    dynamicsCapabilities_ = CapabilitiesFor(kind);
    dynamics_ = std::move(candidateDynamics);
    trajectory_ = std::move(*candidateTrajectory);
    return true;
}

bool SimulationSession::SetBestResponseSettings(
    const BestResponseSettings& settings
)
{
    if (!settings.IsComputable()) {
        return false;
    }

    auto candidateDynamics = CreateDynamics(
        dynamicsKind_,
        parameters_,
        settings
    );
    if (candidateDynamics == nullptr) {
        return false;
    }

    auto candidateTrajectory = trajectoryIntegrator_.Integrate(
        *candidateDynamics,
        currentState_,
        trajectorySettings_
    );
    if (!candidateTrajectory.has_value()) {
        return false;
    }

    bestResponseSettings_ = settings;
    dynamics_ = std::move(candidateDynamics);
    trajectory_ = std::move(*candidateTrajectory);
    return true;
}

bool SimulationSession::SetTrajectorySettings(
    const TrajectorySettings& settings
)
{
    if (!settings.IsComputable()) {
        return false;
    }

    auto candidateTrajectory = trajectoryIntegrator_.Integrate(
        *dynamics_,
        currentState_,
        settings
    );
    if (!candidateTrajectory.has_value()) {
        return false;
    }

    trajectorySettings_ = settings;
    trajectory_ = std::move(*candidateTrajectory);
    return true;
}

const SimplexState& SimulationSession::CurrentState() const noexcept
{
    return currentState_;
}

const OpggParameters& SimulationSession::Parameters() const noexcept
{
    return parameters_;
}

DynamicsKind SimulationSession::ActiveDynamicsKind() const noexcept
{
    return dynamicsKind_;
}

const DynamicsCapabilities&
SimulationSession::ActiveCapabilities() const noexcept
{
    return dynamicsCapabilities_;
}

const BestResponseSettings&
SimulationSession::BestResponseOptions() const noexcept
{
    return bestResponseSettings_;
}

const SimplexDynamicModel&
SimulationSession::ActiveDynamics() const noexcept
{
    return *dynamics_;
}

const TrajectorySettings& SimulationSession::Settings() const noexcept
{
    return trajectorySettings_;
}

const std::vector<SimplexState>&
SimulationSession::Trajectory() const noexcept
{
    return trajectory_;
}

std::optional<std::vector<SimplexEquilibrium>>
SimulationSession::FindEquilibria(
    const SimplexEquilibriumSearchSettings& settings
) const
{
    return equilibriumFinder_.Find(*dynamics_, settings);
}

std::optional<LogitEquilibriumSweepResult>
SimulationSession::GenerateEquilibriumSweep(
    const LogitEquilibriumSweepSettings& settings
) const
{
    if (dynamicsKind_ != DynamicsKind::Logit) {
        return std::nullopt;
    }

    return equilibriumSweep_.Generate(parameters_, settings);
}

bool SimulationSession::RebuildTrajectory()
{
    auto candidateTrajectory = trajectoryIntegrator_.Integrate(
        *dynamics_,
        currentState_,
        trajectorySettings_
    );
    if (!candidateTrajectory.has_value()) {
        return false;
    }

    trajectory_ = std::move(*candidateTrajectory);
    return true;
}
