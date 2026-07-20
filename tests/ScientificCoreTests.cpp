#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <cstddef>
#include <optional>
#include <array>
#include <algorithm>
#include <string_view>

#include "BestResponseDynamics.h"
#include "DynamicsKind.h"
#include "LogitDynamics.h"
#include "LogitEquilibriumSweep.h"
#include "OpggPayoffEvaluator.h"
#include "RealTimeSimulationController.h"
#include "ReplicatorDynamics.h"
#include "SimulationSession.h"
#include "OpggParameters.h"
#include "SimplexDynamicModel.h"
#include "SimplexEquilibriumFinder.h"
#include "SimplexJacobianAnalyzer.h"
#include "SimplexMapper.h"
#include "SimplexState.h"
#include "SimplexTrajectoryIntegrator.h"
#include "SpeedHeatMapGenerator.h"
#include "StreamlineFieldGenerator.h"
#include "TrajectorySettings.h"

namespace
{
    int failures = 0;

    void Check(bool condition, std::string_view message)
    {
        if (!condition) {
            ++failures;
            std::cerr << "FAIL: " << message << '\n';
        }
    }

    void CheckNear(double actual, double expected, double tolerance,
                   std::string_view message)
    {
        Check(std::abs(actual - expected) <= tolerance, message);
        if (std::abs(actual - expected) > tolerance) {
            std::cerr << "  expected " << expected << ", got " << actual << '\n';
        }
    }

    class ConstantTangentDynamic final : public SimplexDynamicModel
    {
    public:
        [[nodiscard]] std::optional<SimplexDerivative> Evaluate(
            const SimplexState&
        ) const override
        {
            return SimplexDerivative{ -0.1, 0.05, 0.05 };
        }
    };

    class NonTangentDynamic final : public SimplexDynamicModel
    {
    public:
        [[nodiscard]] std::optional<SimplexDerivative> Evaluate(
            const SimplexState&
        ) const override
        {
            return SimplexDerivative{ 1.0, 0.0, 0.0 };
        }
    };

    class LinearReducedDynamic final : public SimplexDynamicModel
    {
    public:
        explicit LinearReducedDynamic(ReducedJacobian jacobian)
            : jacobian_(jacobian)
        {
        }

        [[nodiscard]] std::optional<SimplexDerivative> Evaluate(
            const SimplexState& state
        ) const override
        {
            const double offsetX = state.X() - 1.0 / 3.0;
            const double offsetY = state.Y() - 1.0 / 3.0;
            const double dx = jacobian_.dxByDx * offsetX
                + jacobian_.dxByDy * offsetY;
            const double dy = jacobian_.dyByDx * offsetX
                + jacobian_.dyByDy * offsetY;
            return SimplexDerivative{ dx, dy, -dx - dy };
        }

        [[nodiscard]] bool IsClassicallyDifferentiableAt(
            const SimplexState&
        ) const noexcept override
        {
            return true;
        }

    private:
        ReducedJacobian jacobian_;
    };

    class NonsmoothRestPointDynamic final : public SimplexDynamicModel
    {
    public:
        [[nodiscard]] std::optional<SimplexDerivative> Evaluate(
            const SimplexState&
        ) const override
        {
            return SimplexDerivative{};
        }

        [[nodiscard]] DynamicsKind Kind() const noexcept override
        {
            return DynamicsKind::EqualSplitBestResponse;
        }

        [[nodiscard]] bool IsClassicallyDifferentiableAt(
            const SimplexState&
        ) const noexcept override
        {
            return false;
        }
    };

    [[nodiscard]] SimplexDerivative ReferenceLogitDerivative(
        const SimplexState& state,
        const OpggParameters& parameters
    )
    {
        const double x = state.X();
        const double y = state.Y();
        const double z = state.Z();
        const double n = static_cast<double>(parameters.groupSize);
        const double participation =
            (1.0 - (1.0 / n)
                * ((1.0 - std::pow(z, parameters.groupSize)) / (1.0 - z)))
            / (1.0 - z);
        const double zToNMinusOne = std::pow(z, parameters.groupSize - 1);

        const std::array<double, 3> payoffs{
            parameters.contributionCost
                * parameters.lonerPayoffMultiplier
                * zToNMinusOne
                + (parameters.multiplicationFactor - 1.0)
                    * parameters.contributionCost
                    * (1.0 - zToNMinusOne)
                - parameters.multiplicationFactor
                    * parameters.contributionCost
                    * y
                    * participation,
            parameters.contributionCost
                * parameters.lonerPayoffMultiplier
                * zToNMinusOne
                + (1.0 - parameters.punishmentFraction)
                    * parameters.multiplicationFactor
                    * parameters.contributionCost
                    * x
                    * participation,
            parameters.contributionCost * parameters.lonerPayoffMultiplier
        };

        const double maximumPayoff = *std::max_element(
            payoffs.begin(),
            payoffs.end()
        );
        const std::array<double, 3> weights{
            std::exp((payoffs[0] - maximumPayoff) / parameters.logitNoise),
            std::exp((payoffs[1] - maximumPayoff) / parameters.logitNoise),
            std::exp((payoffs[2] - maximumPayoff) / parameters.logitNoise)
        };
        const double totalWeight = weights[0] + weights[1] + weights[2];
        const double targetX = weights[0] / totalWeight;
        const double targetY = weights[1] / totalWeight;
        const double targetZ = 1.0 - targetX - targetY;

        return SimplexDerivative{
            targetX - x,
            targetY - y,
            targetZ - z
        };
    }

    void TestSimplexState()
    {
        const auto state = SimplexState::TryCreate(0.2, 0.3, 0.5);
        Check(state.has_value(), "valid simplex state should be created");
        Check(state.has_value() && state->IsValid(), "created state should be valid");
        Check(!SimplexState::TryCreate(-0.1, 0.4, 0.7).has_value(),
              "materially negative coordinate should be rejected");
        Check(!SimplexState::TryCreate(0.2, 0.3, 0.6).has_value(),
              "non-unit sum should be rejected");
        Check(!SimplexState::TryCreate(NAN, 0.5, 0.5).has_value(),
              "non-finite coordinate should be rejected");

        const SimplexState normalized = SimplexState::Normalized(-1.0, 2.0, 2.0);
        CheckNear(normalized.X(), 0.0, 1e-15, "normalization clamps negative x");
        CheckNear(normalized.Y(), 0.5, 1e-15, "normalization rescales y");
        CheckNear(normalized.Z(), 0.5, 1e-15, "normalization rescales z");
    }

    void TestParameterValidation()
    {
        OpggParameters parameters;
        Check(parameters.IsComputable(), "default OPGG parameters should be valid");

        parameters.punishmentFraction = 1.1;
        Check(!parameters.IsComputable(), "punishment fraction above one is invalid");

        parameters = {};
        parameters.multiplicationFactor = 5.0;
        Check(!parameters.IsComputable(), "r must be strictly below group size");

        parameters = {};
        parameters.lonerPayoffMultiplier = 2.0;
        Check(!parameters.IsComputable(), "sigma must be strictly below r - 1");
    }

    void TestMapper()
    {
        const SimplexMapper mapper(
            Vec2f{ 0.0f, 0.75f },
            Vec2f{ -0.75f, -0.55f },
            Vec2f{ 0.75f, -0.55f }
        );
        const auto original = SimplexState::TryCreate(0.2, 0.3, 0.5);
        Check(original.has_value(), "mapper test state should exist");
        if (!original.has_value()) {
            return;
        }

        const Vec2f position = mapper.ToNdcPosition(*original);
        const auto roundTrip = mapper.FromNdcPosition(position);
        Check(roundTrip.has_value(), "mapped interior point should map back");
        if (roundTrip.has_value()) {
            CheckNear(roundTrip->X(), original->X(), 1e-6, "mapper round-trip x");
            CheckNear(roundTrip->Y(), original->Y(), 1e-6, "mapper round-trip y");
            CheckNear(roundTrip->Z(), original->Z(), 1e-6, "mapper round-trip z");
        }

        const auto clamped = mapper.FromNdcPositionClamped(Vec2f{ 2.0f, -2.0f });
        Check(clamped.has_value() && clamped->IsValid(),
              "outside point should clamp to a valid simplex state");
    }

    void TestLogitReferenceDerivative()
    {
        const LogitDynamics dynamics(OpggParameters{});
        const SimplexState center = SimplexState::Normalized(1.0, 1.0, 1.0);
        const auto centerPayoffs = dynamics.Payoffs(center);
        Check(centerPayoffs.has_value(),
              "shared OPGG payoffs should be computable at center");
        const auto centerTarget = dynamics.ResponseTarget(center);
        Check(centerTarget.has_value(),
              "logit response target should be computable at center");
        const auto derivative = dynamics.Evaluate(center);
        Check(derivative.has_value(), "logit derivative should be computable at center");
        if (!derivative.has_value()) {
            return;
        }

        Check(derivative->IsTangent(), "logit derivative should be tangent to simplex");
        CheckNear(derivative->dx, -0.0406812, 2e-6,
                  "reference center derivative dx");
        CheckNear(derivative->dy, -0.278905, 2e-6,
                  "reference center derivative dy");
        CheckNear(derivative->dz, 0.319586, 2e-6,
                  "reference center derivative dz");

        const std::array<double, 4> lonerFrequencies{ 0.0, 0.2, 0.8, 0.99 };
        for (const double z : lonerFrequencies) {
            const double participantMass = 1.0 - z;
            const auto state = SimplexState::TryCreate(
                0.4 * participantMass,
                0.6 * participantMass,
                z
            );
            Check(state.has_value(), "reference-comparison state should be valid");
            if (!state.has_value()) {
                continue;
            }

            const auto actual = dynamics.Evaluate(*state);
            const SimplexDerivative expected = ReferenceLogitDerivative(
                *state,
                OpggParameters{}
            );
            Check(actual.has_value(), "logit derivative should match reference domain");
            if (actual.has_value()) {
                CheckNear(actual->dx, expected.dx, 2e-12,
                          "stable participation formula dx equivalence");
                CheckNear(actual->dy, expected.dy, 2e-12,
                          "stable participation formula dy equivalence");
                CheckNear(actual->dz, expected.dz, 2e-12,
                          "stable participation formula dz equivalence");
            }
        }

        for (int xIndex = 0; xIndex <= 10; ++xIndex) {
            for (int yIndex = 0; xIndex + yIndex <= 10; ++yIndex) {
                const double x = static_cast<double>(xIndex) / 10.0;
                const double y = static_cast<double>(yIndex) / 10.0;
                const double z = 1.0 - x - y;
                const auto state = SimplexState::TryCreate(x, y, z);
                Check(state.has_value(), "grid simplex state should be valid");
                if (!state.has_value()) {
                    continue;
                }

                const auto gridDerivative = dynamics.Evaluate(*state);
                Check(gridDerivative.has_value()
                          && gridDerivative->IsFinite()
                          && gridDerivative->IsTangent(),
                      "logit field should be finite and tangent over simplex grid");
            }
        }

        const SimplexState lonerCorner = SimplexState::Normalized(0.0, 0.0, 1.0);
        const auto cornerDerivative = dynamics.Evaluate(lonerCorner);
        Check(cornerDerivative.has_value() && cornerDerivative->IsFinite(),
              "pure-loner limit should remain finite");
        if (cornerDerivative.has_value()) {
            CheckNear(cornerDerivative->dx, 1.0 / 3.0, 1e-12,
                      "pure-loner limit dx");
            CheckNear(cornerDerivative->dy, 1.0 / 3.0, 1e-12,
                      "pure-loner limit dy");
            CheckNear(cornerDerivative->dz, -2.0 / 3.0, 1e-12,
                      "pure-loner limit dz");
        }
    }

    void TestSharedPayoffsAndDynamics()
    {
        const OpggParameters parameters{};
        const OpggPayoffEvaluator payoffEvaluator(parameters);
        const LogitDynamics logit(parameters);
        const BestResponseDynamics bestResponse(parameters);
        const ReplicatorDynamics replicator(parameters);
        const SimplexState center = SimplexState::Normalized(1.0, 1.0, 1.0);

        const auto sharedPayoffs = payoffEvaluator.Evaluate(center);
        const auto logitPayoffs = logit.Payoffs(center);
        Check(sharedPayoffs.has_value() && logitPayoffs.has_value(),
              "shared and Logit payoff inspection should both succeed");
        if (sharedPayoffs.has_value() && logitPayoffs.has_value()) {
            CheckNear(logitPayoffs->cooperators, sharedPayoffs->cooperators,
                      0.0, "Logit should use shared cooperator payoff");
            CheckNear(logitPayoffs->defectors, sharedPayoffs->defectors,
                      0.0, "Logit should use shared defector payoff");
            CheckNear(logitPayoffs->loners, sharedPayoffs->loners,
                      0.0, "Logit should use shared loner payoff");
        }

        const std::array<const SimplexDynamicModel*, 3> dynamics{
            &logit,
            &bestResponse,
            &replicator
        };
        for (int xIndex = 0; xIndex <= 20; ++xIndex) {
            for (int yIndex = 0; xIndex + yIndex <= 20; ++yIndex) {
                const double x = static_cast<double>(xIndex) / 20.0;
                const double y = static_cast<double>(yIndex) / 20.0;
                const auto state = SimplexState::TryCreate(x, y, 1.0 - x - y);
                Check(state.has_value(), "dynamics lattice state should be valid");
                if (!state.has_value()) {
                    continue;
                }

                for (const SimplexDynamicModel* model : dynamics) {
                    const auto derivative = model->Evaluate(*state);
                    Check(derivative.has_value()
                              && derivative->IsFinite()
                              && derivative->IsTangent(),
                          "every built-in dynamic should be finite and tangent");
                }

                const auto replicatorDerivative = replicator.Evaluate(*state);
                if (replicatorDerivative.has_value()) {
                    if (x == 0.0) {
                        CheckNear(replicatorDerivative->dx, 0.0, 1e-15,
                                  "Replicator should preserve x=0 face");
                    }
                    if (y == 0.0) {
                        CheckNear(replicatorDerivative->dy, 0.0, 1e-15,
                                  "Replicator should preserve y=0 face");
                    }
                    if (state->Z() == 0.0) {
                        CheckNear(replicatorDerivative->dz, 0.0, 1e-15,
                                  "Replicator should preserve z=0 face");
                    }
                }
            }
        }

        Check(logit.ResponseTarget(center).has_value(),
              "Logit should expose a response target");
        Check(bestResponse.ResponseTarget(center).has_value(),
              "Best Response should expose a response target");
        Check(!replicator.ResponseTarget(center).has_value(),
              "Replicator should not fabricate a response target");

        const auto uniqueWinner = BestResponseDynamics::SelectFromPayoffs(
            StrategyPayoffs{ 3.0, 2.0, 1.0 }
        );
        Check(uniqueWinner.has_value() && uniqueWinner->supportMask == 0x1U,
              "Best Response should select a unique cooperator winner");
        if (uniqueWinner.has_value()) {
            CheckNear(uniqueWinner->target.X(), 1.0, 0.0,
                      "unique best response should place all target mass on winner");
        }

        const auto twoWayTie = BestResponseDynamics::SelectFromPayoffs(
            StrategyPayoffs{ 5.0, 5.0 + 1e-11, 1.0 }
        );
        Check(twoWayTie.has_value() && twoWayTie->supportMask == 0x3U,
              "Best Response should resolve near-equal top payoffs as a tie");
        if (twoWayTie.has_value()) {
            CheckNear(twoWayTie->target.X(), 0.5, 1e-15,
                      "two-way tie should split target mass equally");
            CheckNear(twoWayTie->target.Y(), 0.5, 1e-15,
                      "two-way tie should split target mass equally");
        }

        const auto threeWayTie = BestResponseDynamics::SelectFromPayoffs(
            StrategyPayoffs{ 2.0, 2.0, 2.0 }
        );
        Check(threeWayTie.has_value() && threeWayTie->supportMask == 0x7U,
              "Best Response should detect a three-way tie");
        if (threeWayTie.has_value()) {
            CheckNear(threeWayTie->target.X(), 1.0 / 3.0, 1e-15,
                      "three-way tie should split target mass equally");
        }

        const std::array<SimplexState, 3> vertices{
            SimplexState::Normalized(1.0, 0.0, 0.0),
            SimplexState::Normalized(0.0, 1.0, 0.0),
            SimplexState::Normalized(0.0, 0.0, 1.0)
        };
        for (const SimplexState& vertex : vertices) {
            const auto derivative = replicator.Evaluate(vertex);
            Check(derivative.has_value(),
                  "Replicator derivative should exist at every vertex");
            if (derivative.has_value()) {
                CheckNear(derivative->dx, 0.0, 1e-15,
                          "Replicator vertex dx should vanish");
                CheckNear(derivative->dy, 0.0, 1e-15,
                          "Replicator vertex dy should vanish");
                CheckNear(derivative->dz, 0.0, 1e-15,
                          "Replicator vertex dz should vanish");
            }
        }
    }

    void TestIntegrator()
    {
        const ConstantTangentDynamic dynamic;
        const SimplexTrajectoryIntegrator integrator;
        const auto initial = SimplexState::TryCreate(0.6, 0.3, 0.1);
        Check(initial.has_value(), "integrator initial state should exist");
        if (!initial.has_value()) {
            return;
        }

        TrajectorySettings settings;
        settings.totalTime = 1.0;
        settings.timeStep = 0.1;
        settings.maxSteps = 10;
        const auto trajectory = integrator.Integrate(dynamic, *initial, settings);
        Check(trajectory.has_value(), "constant tangent trajectory should integrate");
        if (trajectory.has_value()) {
            Check(trajectory->size() == 11, "trajectory should contain initial plus 10 states");
            const SimplexState& final = trajectory->back();
            CheckNear(final.X(), 0.5, 1e-12, "constant-field final x");
            CheckNear(final.Y(), 0.35, 1e-12, "constant-field final y");
            CheckNear(final.Z(), 0.15, 1e-12, "constant-field final z");
            for (const SimplexState& state : *trajectory) {
                Check(state.IsValid(), "every integrated state should be valid");
            }
        }

        settings.maxSteps = 9;
        Check(!settings.IsComputable(),
              "settings should reject silent truncation by maxSteps");

        TrajectorySettings decimalRatio;
        decimalRatio.totalTime = 0.3;
        decimalRatio.timeStep = 0.1;
        decimalRatio.maxSteps = 3;
        const auto decimalSteps = decimalRatio.RequestedStepCount();
        Check(decimalSteps.has_value() && *decimalSteps == 3,
              "near-integral floating ratios should not add a spurious step");

        const NonTangentDynamic invalidDynamic;
        TrajectorySettings invalidSettings;
        invalidSettings.totalTime = 0.1;
        invalidSettings.timeStep = 0.1;
        invalidSettings.maxSteps = 1;
        Check(!integrator.Integrate(
                    invalidDynamic,
                    *initial,
                    invalidSettings
                ).has_value(),
              "integrator should reject a non-tangent vector field");
    }

    void TestRealTimeSimulation()
    {
        const LogitDynamics dynamics(OpggParameters{});
        const SimplexState initial = SimplexState::Normalized(1.0, 1.0, 1.0);
        RealTimeSimulationController controller;

        Check(controller.Enter(initial),
              "Real Time mode should accept a valid seed");
        Check(!controller.IsRunning(),
              "Real Time mode should enter paused");
        const auto pausedUpdate = controller.UpdateForWallDuration(
            dynamics,
            0.1
        );
        Check(!pausedUpdate.stateChanged,
              "paused Real Time mode should not advance");
        CheckNear(controller.ElapsedSimulationTime(), 0.0, 0.0,
                  "paused Real Time elapsed time should remain zero");

        Check(controller.Start(), "Real Time Start should succeed");
        const auto draggingUpdate = controller.UpdateForWallDuration(
            dynamics,
            0.1,
            true
        );
        Check(!draggingUpdate.stateChanged,
              "Real Time playback should not advance while dragging");

        const auto update = controller.UpdateForWallDuration(dynamics, 0.1);
        Check(update.acceptedSteps == 10,
              "0.1 wall seconds at 1x should accept ten fixed steps");
        CheckNear(controller.ElapsedSimulationTime(), 0.1, 1e-14,
                  "Real Time elapsed simulation time should track fixed steps");

        TrajectorySettings batchSettings;
        batchSettings.totalTime = 0.1;
        batchSettings.timeStep = controller.Settings().fixedIntegrationStep;
        batchSettings.maxSteps = 10;
        const SimplexTrajectoryIntegrator integrator;
        const auto batch = integrator.Integrate(
            dynamics,
            initial,
            batchSettings
        );
        Check(batch.has_value(),
              "batch comparison trajectory should integrate");
        if (batch.has_value()) {
            CheckNear(controller.LiveState().X(), batch->back().X(), 1e-14,
                      "Real Time and batch integration should agree in x");
            CheckNear(controller.LiveState().Y(), batch->back().Y(), 1e-14,
                      "Real Time and batch integration should agree in y");
            CheckNear(controller.LiveState().Z(), batch->back().Z(), 1e-14,
                      "Real Time and batch integration should agree in z");
        }

        RealTimeSimulationController partitioned;
        Check(partitioned.Enter(initial) && partitioned.Start(),
              "partitioned Real Time controller should start");
        for (int frame = 0; frame < 10; ++frame) {
            (void)partitioned.UpdateForWallDuration(dynamics, 0.01);
        }
        CheckNear(partitioned.LiveState().X(), controller.LiveState().X(),
                  1e-14, "frame partition should not change Real Time x");
        CheckNear(partitioned.LiveState().Y(), controller.LiveState().Y(),
                  1e-14, "frame partition should not change Real Time y");

        RealTimeSimulationController accelerated;
        Check(accelerated.Enter(initial),
              "accelerated Real Time controller should enter");
        Check(accelerated.SetPlaybackSpeed(2.0),
              "positive playback speed should be accepted");
        Check(accelerated.Start(),
              "accelerated Real Time controller should start");
        (void)accelerated.UpdateForWallDuration(dynamics, 0.05);
        CheckNear(accelerated.LiveState().X(), controller.LiveState().X(),
                  1e-14, "speed should not change path at equal simulated time");
        CheckNear(accelerated.LiveState().Y(), controller.LiveState().Y(),
                  1e-14, "speed should not change path at equal simulated time");

        accelerated.Reset();
        Check(!accelerated.IsRunning(), "Reset should pause playback");
        CheckNear(accelerated.LiveState().X(), initial.X(), 0.0,
                  "Reset should restore the seed state");
        CheckNear(accelerated.ElapsedSimulationTime(), 0.0, 0.0,
                  "Reset should clear simulated time");

        Check(accelerated.Start(), "controller should restart after Reset");
        const auto stalled = accelerated.UpdateForWallDuration(dynamics, 1.0);
        Check(stalled.playbackBehind && !stalled.stateChanged,
              "long suspension should report behind without catch-up jump");

        const auto reseed = SimplexState::TryCreate(0.2, 0.5, 0.3);
        Check(reseed.has_value() && accelerated.Reseed(*reseed),
              "Real Time reseed should accept a valid simplex point");
        Check(accelerated.IsRunning(),
              "reseed should preserve the running choice");
        CheckNear(accelerated.LiveState().Y(), 0.5, 1e-15,
                  "reseed should immediately replace the live state");

        RealTimeSimulationController failing;
        Check(failing.Enter(initial) && failing.Start(),
              "failure-path controller should start");
        const NonTangentDynamic invalidDynamic;
        const auto failed = failing.UpdateForWallDuration(
            invalidDynamic,
            0.01
        );
        Check(failed.numericalError && !failing.IsRunning(),
              "failed validated step should pause with explicit error");
        CheckNear(failing.LiveState().X(), initial.X(), 0.0,
                  "failed step should retain the last valid live state");
    }


    void TestSweepAndSession()
    {
        LogitEquilibriumSweepSettings sweepSettings;
        sweepSettings.parameter =
            LogitEquilibriumSweepParameter::PunishmentFraction;
        sweepSettings.minimumParameter = 0.1;
        sweepSettings.maximumParameter = 0.3;
        sweepSettings.sampleCount = 7;
        sweepSettings.maximumBranchStepDistance = 0.2;
        sweepSettings.equilibriumSearchSettings.latticeResolution = 10;

        const LogitEquilibriumSweep sweep{};
        const auto result = sweep.Generate(OpggParameters{}, sweepSettings);
        Check(result.has_value(), "equilibrium sweep should complete");
        Check(result.has_value() && !result->branches.empty(),
              "equilibrium sweep should contain verified samples");

        if (result.has_value()) {
            std::size_t verifiedSampleCount = 0;
            for (const LogitEquilibriumBranch& branch : result->branches) {
                double previousParameter =
                    -std::numeric_limits<double>::infinity();
                for (const LogitEquilibriumSweepSample& sample : branch.samples) {
                    ++verifiedSampleCount;
                    Check(sample.parameterValue >= sweepSettings.minimumParameter
                              && sample.parameterValue
                                  <= sweepSettings.maximumParameter,
                          "sweep parameter should stay in its requested range");
                    Check(sample.parameterValue > previousParameter,
                          "branch parameters should be strictly increasing");
                    Check(sample.equilibrium.state.IsValid(),
                          "sweep equilibrium state should be valid");
                    Check(sample.equilibrium.residual
                              <= sweepSettings
                                     .equilibriumSearchSettings
                                     .residualTolerance,
                          "sweep equilibrium should satisfy residual tolerance");
                    previousParameter = sample.parameterValue;
                }
            }
            Check(verifiedSampleCount >=
                      static_cast<std::size_t>(sweepSettings.sampleCount),
                  "sweep should retain at least one root per sampled parameter");
        }

        SimulationSession session;
        Check(session.Initialize(), "simulation session should initialize");
        Check(session.Trajectory().size() == 501,
              "session should expose the default verified trajectory");

        const OpggParameters originalParameters = session.Parameters();
        OpggParameters invalidParameters = originalParameters;
        invalidParameters.punishmentFraction = 2.0;
        Check(!session.SetParameters(invalidParameters),
              "session should reject invalid parameters");
        CheckNear(
            session.Parameters().punishmentFraction,
            originalParameters.punishmentFraction,
            0.0,
            "failed parameter update should leave session unchanged"
        );

        TrajectorySettings invalidSettings = session.Settings();
        invalidSettings.maxSteps = 1;
        Check(!session.SetTrajectorySettings(invalidSettings),
              "session should reject a truncated trajectory request");
        Check(session.Settings().maxSteps
                  == TrajectorySettings{}.maxSteps,
              "failed trajectory-setting update should be transactional");

        Check(session.SetDynamicsKind(DynamicsKind::Replicator),
              "session should switch transactionally to Replicator");
        Check(session.ActiveDynamicsKind() == DynamicsKind::Replicator,
              "session should expose the active Replicator kind");
        Check(!session.ActiveDynamics().ResponseTarget(
                   session.CurrentState()).has_value(),
              "active Replicator session should not expose a target");
        Check(!session.SetDynamicsKind(DynamicsKind::Custom),
              "session should reject unsupported custom dynamics");
        Check(session.ActiveDynamicsKind() == DynamicsKind::Replicator,
              "failed dynamic switch should leave active kind unchanged");
        Check(session.SetDynamicsKind(DynamicsKind::EqualSplitBestResponse),
              "session should switch to equal-split Best Response");
        Check(session.ActiveDynamics().ResponseTarget(
                  session.CurrentState()).has_value(),
              "active Best Response session should expose a target");
        Check(session.SetDynamicsKind(DynamicsKind::Logit),
              "session should switch back to Logit");
    }

    void TestLogitTrajectoryAndEquilibrium()
    {
        const LogitDynamics dynamics(OpggParameters{});
        const SimplexTrajectoryIntegrator integrator;
        const SimplexState center = SimplexState::Normalized(1.0, 1.0, 1.0);

        TrajectorySettings coarse;
        coarse.totalTime = 10.0;
        coarse.timeStep = 0.02;
        coarse.maxSteps = 10000;
        const auto coarseTrajectory = integrator.Integrate(dynamics, center, coarse);
        Check(coarseTrajectory.has_value(), "default logit trajectory should integrate");
        Check(coarseTrajectory.has_value() && coarseTrajectory->size() == 501,
              "default trajectory should contain 501 states");

        TrajectorySettings fine = coarse;
        fine.timeStep = 0.01;
        const auto fineTrajectory = integrator.Integrate(dynamics, center, fine);
        Check(fineTrajectory.has_value(), "fine logit trajectory should integrate");

        if (coarseTrajectory.has_value() && fineTrajectory.has_value()) {
            const SimplexState& coarseFinal = coarseTrajectory->back();
            const SimplexState& fineFinal = fineTrajectory->back();
            const double distance = std::sqrt(
                std::pow(coarseFinal.X() - fineFinal.X(), 2.0)
                + std::pow(coarseFinal.Y() - fineFinal.Y(), 2.0)
                + std::pow(coarseFinal.Z() - fineFinal.Z(), 2.0)
            );
            Check(distance < 1e-5,
                  "halving time step should give a converged final logit state");
        }

        SimplexEquilibriumFinder finder;
        SimplexEquilibriumSearchSettings search;
        search.latticeResolution = 16;
        const auto equilibria = finder.Find(dynamics, search);
        Check(equilibria.has_value(), "equilibrium search should complete");
        Check(equilibria.has_value() && !equilibria->empty(),
              "default logit model should have an interior rest point");
        if (equilibria.has_value()) {
            for (const SimplexEquilibrium& equilibrium : *equilibria) {
                Check(equilibrium.state.IsValid(), "equilibrium state should be valid");
                Check(equilibrium.residual <= search.residualTolerance,
                      "equilibrium should satisfy residual tolerance");
            }
        }
    }

    void TestModelAwareEquilibriaAndJacobian()
    {
        const OpggParameters parameters{};
        const ReplicatorDynamics replicator(parameters);
        const SimplexEquilibriumFinder finder;
        SimplexEquilibriumSearchSettings search;
        search.latticeResolution = 16;
        const auto replicatorEquilibria = finder.Find(replicator, search);
        Check(replicatorEquilibria.has_value(),
              "Replicator equilibrium search should complete");
        if (replicatorEquilibria.has_value()) {
            Check(replicatorEquilibria->size() <= 12U,
                  "Replicator search should not misclassify near-boundary slow states as equilibria");
            bool hasXVertex = false;
            bool hasYVertex = false;
            bool hasZVertex = false;
            for (const SimplexEquilibrium& equilibrium
                 : *replicatorEquilibria) {
                const auto derivative = replicator.Evaluate(equilibrium.state);
                Check(derivative.has_value(),
                      "reported Replicator equilibrium should evaluate");
                Check(equilibrium.residual <= search.residualTolerance,
                      "reported Replicator equilibrium should be verified");
                if (equilibrium.location
                    == SimplexEquilibriumLocation::Interior) {
                    const auto payoffs = replicator.Payoffs(equilibrium.state);
                    Check(payoffs.has_value(),
                          "interior Replicator equilibrium should expose payoffs");
                    if (payoffs.has_value()) {
                        const double payoffSpread = std::max({
                            std::abs(payoffs->cooperators - payoffs->defectors),
                            std::abs(payoffs->cooperators - payoffs->loners),
                            std::abs(payoffs->defectors - payoffs->loners)
                        });
                        Check(payoffSpread <= search.residualTolerance,
                              "interior Replicator equilibrium should equalize all strategy payoffs");
                    }
                }
                hasXVertex = hasXVertex
                    || equilibrium.location
                        == SimplexEquilibriumLocation::VertexX;
                hasYVertex = hasYVertex
                    || equilibrium.location
                        == SimplexEquilibriumLocation::VertexY;
                hasZVertex = hasZVertex
                    || equilibrium.location
                        == SimplexEquilibriumLocation::VertexZ;
            }
            Check(hasXVertex && hasYVertex && hasZVertex,
                  "Replicator search should include all invariant vertices");
        }

        const BestResponseDynamics bestResponse(parameters);
        const auto bestResponseEquilibria = finder.Find(bestResponse, search);
        Check(bestResponseEquilibria.has_value(),
              "Best Response equilibrium search should complete without Newton");
        if (bestResponseEquilibria.has_value()) {
            for (const SimplexEquilibrium& equilibrium
                 : *bestResponseEquilibria) {
                Check(equilibrium.residual <= search.residualTolerance,
                      "Best Response equilibrium should be residual verified");
            }
        }

        const std::array<ReducedJacobian, 6> matrices{
            ReducedJacobian{ -1.0, 0.0, 0.0, -2.0 },
            ReducedJacobian{ 1.0, 0.0, 0.0, 2.0 },
            ReducedJacobian{ 1.0, 0.0, 0.0, -1.0 },
            ReducedJacobian{ -1.0, -2.0, 2.0, -1.0 },
            ReducedJacobian{ 0.0, -1.0, 1.0, 0.0 },
            ReducedJacobian{ 1.0, 1.0, 0.0, 1.0 }
        };
        const std::array<StabilityClassification, 6> expected{
            StabilityClassification::AttractingNode,
            StabilityClassification::RepellingNode,
            StabilityClassification::Saddle,
            StabilityClassification::SpiralSink,
            StabilityClassification::CenterLike,
            StabilityClassification::RepellingNode
        };

        for (std::size_t matrixIndex = 0;
             matrixIndex < matrices.size();
             ++matrixIndex) {
            const auto analysis =
                SimplexJacobianAnalyzer::AnalyzeMatrix(matrices[matrixIndex]);
            Check(analysis.status == JacobianAnalysisStatus::Available,
                  "known matrix eigenanalysis should be available");
            Check(analysis.classification == expected[matrixIndex],
                  "known matrix should receive expected stability class");
            for (const SimplexEigenpair& eigenpair : analysis.eigenpairs) {
                if (!eigenpair.hasEigenvector) {
                    continue;
                }
                const auto& v = eigenpair.reducedEigenvector;
                const std::complex<double> residualX =
                    matrices[matrixIndex].dxByDx * v[0]
                    + matrices[matrixIndex].dxByDy * v[1]
                    - eigenpair.eigenvalue * v[0];
                const std::complex<double> residualY =
                    matrices[matrixIndex].dyByDx * v[0]
                    + matrices[matrixIndex].dyByDy * v[1]
                    - eigenpair.eigenvalue * v[1];
                Check(std::abs(residualX) <= 1e-9
                          && std::abs(residualY) <= 1e-9,
                      "reported eigenpair should satisfy Jv = lambda v");
                Check(std::abs(
                          eigenpair.simplexEigenvector[0]
                          + eigenpair.simplexEigenvector[1]
                          + eigenpair.simplexEigenvector[2]) <= 1e-12,
                      "reported full eigenvector should be tangent to simplex");
            }
        }

        const auto defective =
            SimplexJacobianAnalyzer::AnalyzeMatrix(matrices.back());
        Check(defective.isRepeatedEigenvalue && defective.isDefective,
              "Jordan block should be marked repeated and defective");

        const ReducedJacobian expectedJacobian{ -1.0, 0.25, 0.5, -2.0 };
        const LinearReducedDynamic linear(expectedJacobian);
        const SimplexJacobianAnalyzer analyzer;
        const auto finiteDifference = analyzer.Analyze(
            linear,
            SimplexState::Normalized(1.0, 1.0, 1.0)
        );
        Check(finiteDifference.jacobian.has_value(),
              "interior finite-difference Jacobian should be available");
        if (finiteDifference.jacobian.has_value()) {
            CheckNear(finiteDifference.jacobian->dxByDx,
                      expectedJacobian.dxByDx, 1e-9,
                      "finite-difference J00 should match linear field");
            CheckNear(finiteDifference.jacobian->dxByDy,
                      expectedJacobian.dxByDy, 1e-9,
                      "finite-difference J01 should match linear field");
            CheckNear(finiteDifference.jacobian->dyByDx,
                      expectedJacobian.dyByDx, 1e-9,
                      "finite-difference J10 should match linear field");
            CheckNear(finiteDifference.jacobian->dyByDy,
                      expectedJacobian.dyByDy, 1e-9,
                      "finite-difference J11 should match linear field");
        }

        const auto boundaryAnalysis = analyzer.Analyze(
            replicator,
            SimplexState::Normalized(1.0, 0.0, 0.0)
        );
        Check(boundaryAnalysis.jacobian.has_value()
                  && boundaryAnalysis.isBoundaryConstrained,
              "smooth boundary equilibrium should use constrained stencil");

        const NonsmoothRestPointDynamic nonsmooth;
        const auto nonsmoothAnalysis = analyzer.Analyze(
            nonsmooth,
            SimplexState::Normalized(1.0, 1.0, 1.0)
        );
        Check(nonsmoothAnalysis.status
                  == JacobianAnalysisStatus::NonsmoothSwitchingSurface,
              "nonsmooth switching point should reject classical Jacobian");
        Check(!nonsmoothAnalysis.jacobian.has_value(),
              "nonsmooth switching point should not fabricate a Jacobian");
    }

    void TestHeatMapAndStreamlines()
    {
        const OpggParameters parameters{};
        const LogitDynamics logit(parameters);
        const SpeedHeatMapGenerator heatMapGenerator;
        SpeedHeatMapSettings heatMapSettings;
        heatMapSettings.resolution = 8;
        const auto heatMap = heatMapGenerator.Generate(logit, heatMapSettings);
        Check(heatMap.has_value(), "Logit speed heat map should generate");
        if (heatMap.has_value()) {
            Check(heatMap->samples.size() == 45U,
                  "resolution 8 heat map should have 45 samples");
            Check(heatMap->triangleIndices.size() == 64U * 3U,
                  "resolution 8 heat map should have 64 triangles");
            Check(heatMap->mixedRegionTriangles.size() == 64U,
                  "heat map should classify every triangle region");
            bool hasMinimumColor = false;
            bool hasMaximumColor = false;
            bool hasXVertex = false;
            bool hasYVertex = false;
            bool hasZVertex = false;
            for (const SpeedHeatMapSample& sample : heatMap->samples) {
                Check(std::isfinite(sample.speed)
                          && sample.normalizedSpeed >= 0.0F
                          && sample.normalizedSpeed <= 1.0F,
                      "heat-map scalar should be finite and normalized");
                hasMinimumColor = hasMinimumColor
                    || sample.normalizedSpeed <= 1e-6F;
                hasMaximumColor = hasMaximumColor
                    || sample.normalizedSpeed >= 1.0F - 1e-6F;
                hasXVertex = hasXVertex || sample.state.X() == 1.0;
                hasYVertex = hasYVertex || sample.state.Y() == 1.0;
                hasZVertex = hasZVertex || sample.state.Z() == 1.0;
            }
            Check(hasMinimumColor && hasMaximumColor,
                  "relative heat map should span the full palette");
            Check(hasXVertex && hasYVertex && hasZVertex,
                  "heat map should include all simplex vertices");
        }

        const HeatMapColor slow = SpeedHeatMapGenerator::Palette(0.0F);
        const HeatMapColor middle = SpeedHeatMapGenerator::Palette(0.4F);
        const HeatMapColor fast = SpeedHeatMapGenerator::Palette(1.0F);
        Check(slow.blue > slow.red && slow.blue > slow.green,
              "slow palette endpoint should be dark blue");
        Check(middle.green > middle.red && middle.green > middle.blue,
              "middle palette should pass through green");
        Check(fast.red > fast.green && fast.red > fast.blue,
              "fast palette endpoint should be red");

        SpeedHeatMapSettings locked = heatMapSettings;
        locked.normalization = HeatMapNormalizationMode::LockedRange;
        locked.lockedMinimumSpeed = 0.0;
        locked.lockedMaximumSpeed = 10.0;
        const auto lockedMap = heatMapGenerator.Generate(logit, locked);
        Check(lockedMap.has_value()
                  && lockedMap->displayedMaximumSpeed == 10.0,
              "locked heat map should retain its declared numeric range");

        const BestResponseDynamics bestResponse(parameters);
        const auto bestResponseMap = heatMapGenerator.Generate(
            bestResponse,
            heatMapSettings
        );
        Check(bestResponseMap.has_value(),
              "Best Response heat map should generate");
        if (bestResponseMap.has_value()) {
            bool hasRegionMetadata = false;
            bool hasMixedTriangle = false;
            for (const SpeedHeatMapSample& sample
                 : bestResponseMap->samples) {
                hasRegionMetadata = hasRegionMetadata
                    || sample.regionIdentifier != 0U;
            }
            for (const std::uint8_t mixed
                 : bestResponseMap->mixedRegionTriangles) {
                hasMixedTriangle = hasMixedTriangle || mixed != 0U;
            }
            Check(hasRegionMetadata,
                  "Best Response heat map should carry region metadata");
            Check(hasMixedTriangle,
                  "Best Response grid should identify switching-boundary cells");
        }

        const SimplexMapper mapper(
            Vec2f{ 0.0F, 0.75F },
            Vec2f{ -0.75F, -0.55F },
            Vec2f{ 0.75F, -0.55F }
        );
        StreamlineFieldSettings fieldSettings;
        fieldSettings.density = 7;
        fieldSettings.integrationTime = 0.6;
        fieldSettings.integrationTimeStep = 0.02;
        fieldSettings.maximumStepsPerPath = 100;
        const StreamlineFieldGenerator fieldGenerator;
        const auto field = fieldGenerator.Generate(
            logit,
            mapper,
            fieldSettings
        );
        Check(field.has_value(), "Logit streamline field should generate");
        if (field.has_value()) {
            Check(field->candidateSeedCount == 36,
                  "density 7 should create 36 deterministic candidates");
            Check(field->acceptedSeedCount > 0,
                  "streamline occupancy should retain visible paths");
            Check(field->lineSegmentVertices.size() % 2U == 0U,
                  "streamline line geometry should consist of GL_LINES pairs");
            Check(field->arrows.size() == field->paths.size(),
                  "every nondegenerate streamline should have one arrow");
            for (const StreamlinePath& path : field->paths) {
                for (const SimplexState& state : path.states) {
                    Check(state.IsValid(),
                          "every generated streamline state should be valid");
                }
            }
            for (const StreamlineArrow& arrow : field->arrows) {
                const double directionLength = std::sqrt(
                    static_cast<double>(arrow.direction.x * arrow.direction.x)
                    + static_cast<double>(arrow.direction.y * arrow.direction.y)
                );
                CheckNear(directionLength, 1.0, 1e-5,
                          "streamline arrow direction should be normalized");
            }

            const auto repeated = fieldGenerator.Generate(
                logit,
                mapper,
                fieldSettings
            );
            Check(repeated.has_value()
                      && repeated->acceptedSeedCount == field->acceptedSeedCount
                      && repeated->lineSegmentVertices.size()
                          == field->lineSegmentVertices.size(),
                  "identical streamline generation should be deterministic");
            if (repeated.has_value()
                && !field->lineSegmentVertices.empty()
                && !repeated->lineSegmentVertices.empty()) {
                CheckNear(repeated->lineSegmentVertices.front().x,
                          field->lineSegmentVertices.front().x, 0.0,
                          "deterministic streamline x geometry should match");
                CheckNear(repeated->lineSegmentVertices.front().y,
                          field->lineSegmentVertices.front().y, 0.0,
                          "deterministic streamline y geometry should match");
            }
        }

        const ReplicatorDynamics replicator(parameters);
        const auto replicatorField = fieldGenerator.Generate(
            replicator,
            mapper,
            fieldSettings
        );
        Check(replicatorField.has_value(),
              "Replicator streamline field should generate");
        if (replicatorField.has_value()) {
            bool verifiedBoundaryPath = false;
            for (const StreamlinePath& path : replicatorField->paths) {
                if (!path.startsOnBoundary || path.states.empty()) {
                    continue;
                }
                const SimplexState& seed = path.states.front();
                const bool zeroX = seed.X() <= 1e-12;
                const bool zeroY = seed.Y() <= 1e-12;
                const bool zeroZ = seed.Z() <= 1e-12;
                for (const SimplexState& state : path.states) {
                    if (zeroX) {
                        CheckNear(state.X(), 0.0, 1e-9,
                                  "Replicator streamline should preserve x=0 edge");
                    }
                    if (zeroY) {
                        CheckNear(state.Y(), 0.0, 1e-9,
                                  "Replicator streamline should preserve y=0 edge");
                    }
                    if (zeroZ) {
                        CheckNear(state.Z(), 0.0, 1e-9,
                                  "Replicator streamline should preserve z=0 edge");
                    }
                }
                verifiedBoundaryPath = true;
            }
            Check(verifiedBoundaryPath,
                  "Replicator field should retain at least one boundary path");
        }
    }
}

int main()
{
    TestSimplexState();
    TestParameterValidation();
    TestMapper();
    TestLogitReferenceDerivative();
    TestSharedPayoffsAndDynamics();
    TestIntegrator();
    TestRealTimeSimulation();
    TestLogitTrajectoryAndEquilibrium();
    TestModelAwareEquilibriaAndJacobian();
    TestHeatMapAndStreamlines();
    TestSweepAndSession();

    if (failures != 0) {
        std::cerr << failures << " scientific core test(s) failed.\n";
        return EXIT_FAILURE;
    }

    std::cout << "All scientific core tests passed.\n";
    return EXIT_SUCCESS;
}
