#pragma once

#include <chrono>
#include <optional>
#include <string_view>

#include "SimplexDynamicModel.h"
#include "SimplexState.h"
#include "SimplexTrajectoryIntegrator.h"

struct RealTimeSimulationSettings final
{
    double fixedIntegrationStep = 0.01;
    double playbackSpeed = 1.0;
    int maximumSubstepsPerFrame = 120;
    double suspensionThresholdSeconds = 0.5;

    [[nodiscard]] bool IsComputable() const noexcept;
};

enum class RealTimeSimulationStatus
{
    Paused,
    Running,
    BehindRealTime,
    NumericalError
};

struct RealTimeUpdateResult final
{
    int acceptedSteps = 0;
    bool stateChanged = false;
    bool playbackBehind = false;
    bool numericalError = false;
};

class RealTimeSimulationController final
{
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    [[nodiscard]] bool Enter(
        const SimplexState& seed,
        TimePoint now = Clock::now()
    );
    void Leave() noexcept;

    [[nodiscard]] bool Start(TimePoint now = Clock::now()) noexcept;
    void Pause(TimePoint now = Clock::now()) noexcept;
    void Reset(TimePoint now = Clock::now()) noexcept;
    [[nodiscard]] bool Reseed(
        const SimplexState& seed,
        TimePoint now = Clock::now()
    );
    void NotifyDynamicsChanged(TimePoint now = Clock::now()) noexcept;
    void ResetClockBaseline(TimePoint now = Clock::now()) noexcept;

    [[nodiscard]] bool SetSettings(
        const RealTimeSimulationSettings& settings,
        TimePoint now = Clock::now()
    ) noexcept;
    [[nodiscard]] bool SetPlaybackSpeed(
        double playbackSpeed,
        TimePoint now = Clock::now()
    ) noexcept;

    [[nodiscard]] RealTimeUpdateResult Update(
        const SimplexDynamicModel& dynamics,
        bool pointerDragging,
        TimePoint now = Clock::now()
    );
    [[nodiscard]] RealTimeUpdateResult UpdateForWallDuration(
        const SimplexDynamicModel& dynamics,
        double wallDurationSeconds,
        bool pointerDragging = false
    );

    [[nodiscard]] bool IsActive() const noexcept;
    [[nodiscard]] bool IsRunning() const noexcept;
    [[nodiscard]] const SimplexState& LiveState() const noexcept;
    [[nodiscard]] const SimplexState& SeedState() const noexcept;
    [[nodiscard]] double ElapsedSimulationTime() const noexcept;
    [[nodiscard]] double AccumulatedSimulationTime() const noexcept;
    [[nodiscard]] const RealTimeSimulationSettings& Settings() const noexcept;
    [[nodiscard]] RealTimeSimulationStatus Status() const noexcept;
    [[nodiscard]] std::string_view StatusText() const noexcept;

private:
    [[nodiscard]] RealTimeUpdateResult ConsumeWallDuration(
        const SimplexDynamicModel& dynamics,
        double wallDurationSeconds,
        bool pointerDragging
    );

    SimplexState liveState_ = SimplexState::Normalized(1.0, 1.0, 1.0);
    SimplexState seedState_ = SimplexState::Normalized(1.0, 1.0, 1.0);
    RealTimeSimulationSettings settings_;
    SimplexTrajectoryIntegrator integrator_;
    std::optional<TimePoint> latestWallTime_;
    double accumulatedSimulationTime_ = 0.0;
    double elapsedSimulationTime_ = 0.0;
    bool active_ = false;
    bool running_ = false;
    RealTimeSimulationStatus status_ = RealTimeSimulationStatus::Paused;
};
