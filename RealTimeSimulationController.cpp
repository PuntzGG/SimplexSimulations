#include "RealTimeSimulationController.h"

#include <algorithm>
#include <cmath>

bool RealTimeSimulationSettings::IsComputable() const noexcept
{
    return std::isfinite(fixedIntegrationStep)
        && std::isfinite(playbackSpeed)
        && std::isfinite(suspensionThresholdSeconds)
        && fixedIntegrationStep > 0.0
        && playbackSpeed > 0.0
        && maximumSubstepsPerFrame > 0
        && suspensionThresholdSeconds > 0.0;
}

bool RealTimeSimulationController::Enter(
    const SimplexState& seed,
    TimePoint now
)
{
    if (!seed.IsValid() || !settings_.IsComputable()) {
        return false;
    }

    liveState_ = seed;
    seedState_ = seed;
    accumulatedSimulationTime_ = 0.0;
    elapsedSimulationTime_ = 0.0;
    latestWallTime_ = now;
    active_ = true;
    running_ = false;
    status_ = RealTimeSimulationStatus::Paused;
    return true;
}

void RealTimeSimulationController::Leave() noexcept
{
    active_ = false;
    running_ = false;
    accumulatedSimulationTime_ = 0.0;
    latestWallTime_.reset();
    status_ = RealTimeSimulationStatus::Paused;
}

bool RealTimeSimulationController::Start(TimePoint now) noexcept
{
    if (!active_ || status_ == RealTimeSimulationStatus::NumericalError) {
        return false;
    }

    running_ = true;
    latestWallTime_ = now;
    status_ = RealTimeSimulationStatus::Running;
    return true;
}

void RealTimeSimulationController::Pause(TimePoint now) noexcept
{
    running_ = false;
    latestWallTime_ = now;
    if (active_) {
        status_ = RealTimeSimulationStatus::Paused;
    }
}

void RealTimeSimulationController::Reset(TimePoint now) noexcept
{
    if (!active_) {
        return;
    }

    liveState_ = seedState_;
    accumulatedSimulationTime_ = 0.0;
    elapsedSimulationTime_ = 0.0;
    running_ = false;
    latestWallTime_ = now;
    status_ = RealTimeSimulationStatus::Paused;
}

bool RealTimeSimulationController::Reseed(
    const SimplexState& seed,
    TimePoint now
)
{
    if (!active_ || !seed.IsValid()) {
        return false;
    }

    liveState_ = seed;
    seedState_ = seed;
    accumulatedSimulationTime_ = 0.0;
    elapsedSimulationTime_ = 0.0;
    latestWallTime_ = now;
    status_ = running_
        ? RealTimeSimulationStatus::Running
        : RealTimeSimulationStatus::Paused;
    return true;
}

void RealTimeSimulationController::NotifyDynamicsChanged(TimePoint now) noexcept
{
    if (!active_) {
        return;
    }

    accumulatedSimulationTime_ = 0.0;
    running_ = false;
    latestWallTime_ = now;
    status_ = RealTimeSimulationStatus::Paused;
}

void RealTimeSimulationController::ResetClockBaseline(TimePoint now) noexcept
{
    latestWallTime_ = active_ ? std::optional<TimePoint>{ now } : std::nullopt;
}

bool RealTimeSimulationController::SetSettings(
    const RealTimeSimulationSettings& settings,
    TimePoint now
) noexcept
{
    if (!settings.IsComputable()) {
        return false;
    }

    settings_ = settings;
    accumulatedSimulationTime_ = 0.0;
    running_ = false;
    latestWallTime_ = active_ ? std::optional<TimePoint>{ now } : std::nullopt;
    status_ = RealTimeSimulationStatus::Paused;
    return true;
}

bool RealTimeSimulationController::SetPlaybackSpeed(
    double playbackSpeed,
    TimePoint now
) noexcept
{
    if (!std::isfinite(playbackSpeed) || playbackSpeed <= 0.0) {
        return false;
    }

    settings_.playbackSpeed = playbackSpeed;
    latestWallTime_ = active_ ? std::optional<TimePoint>{ now } : std::nullopt;
    return true;
}

RealTimeUpdateResult RealTimeSimulationController::Update(
    const SimplexDynamicModel& dynamics,
    bool pointerDragging,
    TimePoint now
)
{
    if (!active_) {
        return RealTimeUpdateResult{};
    }

    if (!latestWallTime_.has_value()) {
        latestWallTime_ = now;
        return RealTimeUpdateResult{};
    }

    const std::chrono::duration<double> elapsed = now - *latestWallTime_;
    latestWallTime_ = now;
    return ConsumeWallDuration(dynamics, elapsed.count(), pointerDragging);
}

RealTimeUpdateResult RealTimeSimulationController::UpdateForWallDuration(
    const SimplexDynamicModel& dynamics,
    double wallDurationSeconds,
    bool pointerDragging
)
{
    return ConsumeWallDuration(
        dynamics,
        wallDurationSeconds,
        pointerDragging
    );
}

bool RealTimeSimulationController::IsActive() const noexcept
{
    return active_;
}

bool RealTimeSimulationController::IsRunning() const noexcept
{
    return active_ && running_;
}

const SimplexState& RealTimeSimulationController::LiveState() const noexcept
{
    return liveState_;
}

const SimplexState& RealTimeSimulationController::SeedState() const noexcept
{
    return seedState_;
}

double RealTimeSimulationController::ElapsedSimulationTime() const noexcept
{
    return elapsedSimulationTime_;
}

double RealTimeSimulationController::AccumulatedSimulationTime() const noexcept
{
    return accumulatedSimulationTime_;
}

const RealTimeSimulationSettings&
RealTimeSimulationController::Settings() const noexcept
{
    return settings_;
}

RealTimeSimulationStatus RealTimeSimulationController::Status() const noexcept
{
    return status_;
}

std::string_view RealTimeSimulationController::StatusText() const noexcept
{
    switch (status_) {
    case RealTimeSimulationStatus::Paused:
        return "Paused";
    case RealTimeSimulationStatus::Running:
        return "Running";
    case RealTimeSimulationStatus::BehindRealTime:
        return "Playback cannot keep pace";
    case RealTimeSimulationStatus::NumericalError:
        return "Numerical integration failed; playback paused";
    }

    return "Unknown";
}

RealTimeUpdateResult RealTimeSimulationController::ConsumeWallDuration(
    const SimplexDynamicModel& dynamics,
    double wallDurationSeconds,
    bool pointerDragging
)
{
    RealTimeUpdateResult result;
    if (!active_
        || !running_
        || pointerDragging
        || !std::isfinite(wallDurationSeconds)
        || wallDurationSeconds <= 0.0) {
        return result;
    }

    if (wallDurationSeconds > settings_.suspensionThresholdSeconds) {
        status_ = RealTimeSimulationStatus::BehindRealTime;
        result.playbackBehind = true;
        return result;
    }

    const double requestedSimulationTime =
        wallDurationSeconds * settings_.playbackSpeed;
    const double frameCapacity = settings_.fixedIntegrationStep
        * static_cast<double>(settings_.maximumSubstepsPerFrame);
    const double availableCapacity = std::max(
        0.0,
        frameCapacity - accumulatedSimulationTime_
    );
    const double acceptedSimulationTime = std::min(
        requestedSimulationTime,
        availableCapacity
    );
    result.playbackBehind = requestedSimulationTime > availableCapacity;
    accumulatedSimulationTime_ += acceptedSimulationTime;

    const double comparisonTolerance =
        settings_.fixedIntegrationStep * 1e-12;
    while (result.acceptedSteps < settings_.maximumSubstepsPerFrame
           && accumulatedSimulationTime_ + comparisonTolerance
               >= settings_.fixedIntegrationStep) {
        const auto nextState = integrator_.AdvanceOneStep(
            dynamics,
            liveState_,
            settings_.fixedIntegrationStep
        );
        if (!nextState.has_value()) {
            running_ = false;
            status_ = RealTimeSimulationStatus::NumericalError;
            result.numericalError = true;
            return result;
        }

        liveState_ = *nextState;
        accumulatedSimulationTime_ = std::max(
            0.0,
            accumulatedSimulationTime_ - settings_.fixedIntegrationStep
        );
        elapsedSimulationTime_ += settings_.fixedIntegrationStep;
        ++result.acceptedSteps;
        result.stateChanged = true;
    }

    status_ = result.playbackBehind
        ? RealTimeSimulationStatus::BehindRealTime
        : RealTimeSimulationStatus::Running;
    return result;
}
