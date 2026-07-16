#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <cstddef>
#include <optional>
#include <array>
#include <algorithm>
#include <string_view>

#include "LogitDynamics.h"
#include "LogitEquilibriumSweep.h"
#include "SimulationSession.h"
#include "OpggParameters.h"
#include "SimplexDynamicModel.h"
#include "SimplexEquilibriumFinder.h"
#include "SimplexMapper.h"
#include "SimplexState.h"
#include "SimplexTrajectoryIntegrator.h"
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

        const LogitEquilibriumSweep sweep;
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
}

int main()
{
    TestSimplexState();
    TestParameterValidation();
    TestMapper();
    TestLogitReferenceDerivative();
    TestIntegrator();
    TestLogitTrajectoryAndEquilibrium();
    TestSweepAndSession();

    if (failures != 0) {
        std::cerr << failures << " scientific core test(s) failed.\n";
        return EXIT_FAILURE;
    }

    std::cout << "All scientific core tests passed.\n";
    return EXIT_SUCCESS;
}
