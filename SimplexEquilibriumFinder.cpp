#include "SimplexEquilibriumFinder.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace
{
    struct Coordinates final
    {
        double x = 0.0;
        double y = 0.0;
    };

    struct VectorFieldValue final
    {
        double dx = 0.0;
        double dy = 0.0;
        double residual = 0.0;
    };

    struct Jacobian final
    {
        double dxByDx = 0.0;
        double dxByDy = 0.0;
        double dyByDx = 0.0;
        double dyByDy = 0.0;
    };

    [[nodiscard]] std::optional<SimplexState> MakeInteriorState(
        const Coordinates& coordinates,
        double margin
    )
    {
        const double z = 1.0 - coordinates.x - coordinates.y;
        if (!std::isfinite(coordinates.x)
            || !std::isfinite(coordinates.y)
            || !std::isfinite(z)
            || coordinates.x <= margin
            || coordinates.y <= margin
            || z <= margin) {
            return std::nullopt;
        }

        return SimplexState::TryCreate(
            coordinates.x,
            coordinates.y,
            z,
            std::max(SimplexState::kDefaultEpsilon, margin)
        );
    }

    [[nodiscard]] std::optional<VectorFieldValue> EvaluateRootFunction(
        const SimplexDynamicModel& dynamics,
        const Coordinates& coordinates,
        const SimplexEquilibriumSearchSettings& settings
    )
    {
        const auto state = MakeInteriorState(coordinates, settings.interiorMargin);
        if (!state.has_value()) {
            return std::nullopt;
        }

        if (dynamics.Kind() == DynamicsKind::Replicator) {
            // In the open simplex, Replicator rest points require equal
            // payoffs. Solving x_i(pi_i - average) = 0 directly creates false
            // roots arbitrarily close to a boundary because x_i is tiny.
            // Payoff differences remove that degeneracy; invariant faces are
            // handled separately by the explicit boundary search below.
            const auto payoffs = dynamics.Payoffs(*state);
            if (!payoffs.has_value() || !payoffs->IsFinite()) {
                return std::nullopt;
            }
            const double xDifference =
                payoffs->cooperators - payoffs->loners;
            const double yDifference =
                payoffs->defectors - payoffs->loners;
            return VectorFieldValue{
                xDifference,
                yDifference,
                std::max({
                    std::abs(xDifference),
                    std::abs(yDifference),
                    std::abs(payoffs->cooperators - payoffs->defectors)
                })
            };
        }

        const auto derivative = dynamics.Evaluate(*state);
        if (!derivative.has_value()
            || !derivative->IsFinite()
            || !derivative->IsTangent()) {
            return std::nullopt;
        }

        return VectorFieldValue{
            derivative->dx,
            derivative->dy,
            std::max({
                std::abs(derivative->dx),
                std::abs(derivative->dy),
                std::abs(derivative->dz)
            })
        };
    }

    [[nodiscard]] std::optional<Jacobian> EstimateJacobian(
        const SimplexDynamicModel& dynamics,
        const Coordinates& coordinates,
        const SimplexEquilibriumSearchSettings& settings
    )
    {
        const double z = 1.0 - coordinates.x - coordinates.y;
        const double xStep = std::min({
            settings.finiteDifferenceStep,
            (coordinates.x - settings.interiorMargin) * 0.5,
            (z - settings.interiorMargin) * 0.5
        });
        const double yStep = std::min({
            settings.finiteDifferenceStep,
            (coordinates.y - settings.interiorMargin) * 0.5,
            (z - settings.interiorMargin) * 0.5
        });

        if (!std::isfinite(xStep)
            || !std::isfinite(yStep)
            || xStep <= 0.0
            || yStep <= 0.0) {
            return std::nullopt;
        }

        const auto positiveX = EvaluateRootFunction(
            dynamics,
            Coordinates{ coordinates.x + xStep, coordinates.y },
            settings
        );
        const auto negativeX = EvaluateRootFunction(
            dynamics,
            Coordinates{ coordinates.x - xStep, coordinates.y },
            settings
        );
        const auto positiveY = EvaluateRootFunction(
            dynamics,
            Coordinates{ coordinates.x, coordinates.y + yStep },
            settings
        );
        const auto negativeY = EvaluateRootFunction(
            dynamics,
            Coordinates{ coordinates.x, coordinates.y - yStep },
            settings
        );

        if (!positiveX.has_value()
            || !negativeX.has_value()
            || !positiveY.has_value()
            || !negativeY.has_value()) {
            return std::nullopt;
        }

        return Jacobian{
            (positiveX->dx - negativeX->dx) / (2.0 * xStep),
            (positiveY->dx - negativeY->dx) / (2.0 * yStep),
            (positiveX->dy - negativeX->dy) / (2.0 * xStep),
            (positiveY->dy - negativeY->dy) / (2.0 * yStep)
        };
    }

    [[nodiscard]] std::optional<SimplexEquilibrium> RefineEquilibrium(
        const SimplexDynamicModel& dynamics,
        Coordinates coordinates,
        const SimplexEquilibriumSearchSettings& settings
    )
    {
        auto value = EvaluateRootFunction(dynamics, coordinates, settings);
        if (!value.has_value()) {
            return std::nullopt;
        }

        for (int iteration = 0;
             iteration < settings.maximumNewtonIterations;
             ++iteration) {
            if (value->residual <= settings.residualTolerance) {
                const auto state = MakeInteriorState(
                    coordinates,
                    settings.interiorMargin
                );
                if (!state.has_value()) {
                    return std::nullopt;
                }

                const auto verification = dynamics.Evaluate(*state);
                if (!verification.has_value()
                    || !verification->IsFinite()
                    || !verification->IsTangent()) {
                    return std::nullopt;
                }

                const double verifiedResidual = std::max({
                    std::abs(verification->dx),
                    std::abs(verification->dy),
                    std::abs(verification->dz)
                });
                if (verifiedResidual > settings.residualTolerance) {
                    return std::nullopt;
                }

                const auto rootVerification = EvaluateRootFunction(
                    dynamics,
                    Coordinates{ state->X(), state->Y() },
                    settings
                );
                if (!rootVerification.has_value()
                    || rootVerification->residual > settings.residualTolerance) {
                    return std::nullopt;
                }

                return SimplexEquilibrium{ *state, verifiedResidual };
            }

            const auto jacobian = EstimateJacobian(
                dynamics,
                coordinates,
                settings
            );
            if (!jacobian.has_value()) {
                return std::nullopt;
            }

            const double determinant =
                jacobian->dxByDx * jacobian->dyByDy
                - jacobian->dxByDy * jacobian->dyByDx;
            if (!std::isfinite(determinant)
                || std::abs(determinant)
                    <= settings.minimumJacobianDeterminant) {
                return std::nullopt;
            }

            const double stepX =
                (jacobian->dxByDy * value->dy
                    - jacobian->dyByDy * value->dx)
                / determinant;
            const double stepY =
                (jacobian->dyByDx * value->dx
                    - jacobian->dxByDx * value->dy)
                / determinant;
            if (!std::isfinite(stepX) || !std::isfinite(stepY)) {
                return std::nullopt;
            }

            bool accepted = false;
            double scale = 1.0;
            for (int attempt = 0;
                 attempt < settings.maximumBacktrackingSteps;
                 ++attempt) {
                const Coordinates candidate{
                    coordinates.x + scale * stepX,
                    coordinates.y + scale * stepY
                };
                const auto candidateValue = EvaluateRootFunction(
                    dynamics,
                    candidate,
                    settings
                );

                if (candidateValue.has_value()
                    && candidateValue->residual < value->residual) {
                    coordinates = candidate;
                    value = candidateValue;
                    accepted = true;
                    break;
                }

                scale *= 0.5;
            }

            if (!accepted) {
                return std::nullopt;
            }
        }

        return std::nullopt;
    }

    [[nodiscard]] bool IsDuplicate(
        const SimplexEquilibrium& candidate,
        const std::vector<SimplexEquilibrium>& equilibria,
        double duplicateDistance
    )
    {
        const double thresholdSquared = duplicateDistance * duplicateDistance;
        for (const SimplexEquilibrium& existing : equilibria) {
            const double dx = candidate.state.X() - existing.state.X();
            const double dy = candidate.state.Y() - existing.state.Y();
            const double dz = candidate.state.Z() - existing.state.Z();
            if (dx * dx + dy * dy + dz * dz <= thresholdSquared) {
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] SimplexEquilibriumLocation ClassifyLocation(
        const SimplexState& state,
        double tolerance
    ) noexcept
    {
        const bool zeroX = state.X() <= tolerance;
        const bool zeroY = state.Y() <= tolerance;
        const bool zeroZ = state.Z() <= tolerance;

        if (zeroY && zeroZ) {
            return SimplexEquilibriumLocation::VertexX;
        }
        if (zeroX && zeroZ) {
            return SimplexEquilibriumLocation::VertexY;
        }
        if (zeroX && zeroY) {
            return SimplexEquilibriumLocation::VertexZ;
        }
        if (zeroZ) {
            return SimplexEquilibriumLocation::EdgeXY;
        }
        if (zeroY) {
            return SimplexEquilibriumLocation::EdgeXZ;
        }
        if (zeroX) {
            return SimplexEquilibriumLocation::EdgeYZ;
        }
        return SimplexEquilibriumLocation::Interior;
    }

    [[nodiscard]] std::optional<SimplexEquilibrium> VerifyState(
        const SimplexDynamicModel& dynamics,
        const SimplexState& state,
        const SimplexEquilibriumSearchSettings& settings,
        bool isIsolated = true
    )
    {
        const auto derivative = dynamics.Evaluate(state);
        if (!derivative.has_value()
            || !derivative->IsFinite()
            || !derivative->IsTangent()) {
            return std::nullopt;
        }

        const double residual = std::max({
            std::abs(derivative->dx),
            std::abs(derivative->dy),
            std::abs(derivative->dz)
        });
        if (residual > settings.residualTolerance) {
            return std::nullopt;
        }

        return SimplexEquilibrium{
            state,
            residual,
            ClassifyLocation(state, settings.duplicateDistance),
            isIsolated
        };
    }

    void AddIfUnique(
        const SimplexEquilibrium& candidate,
        std::vector<SimplexEquilibrium>& equilibria,
        double duplicateDistance
    )
    {
        if (!IsDuplicate(candidate, equilibria, duplicateDistance)) {
            equilibria.push_back(candidate);
        }
    }

    [[nodiscard]] SimplexState MakeEdgeState(int edge, double parameter)
    {
        switch (edge) {
        case 0:
            return SimplexState::Normalized(parameter, 1.0 - parameter, 0.0);
        case 1:
            return SimplexState::Normalized(parameter, 0.0, 1.0 - parameter);
        default:
            return SimplexState::Normalized(0.0, parameter, 1.0 - parameter);
        }
    }

    [[nodiscard]] std::optional<double> EvaluateEdgeComponent(
        const SimplexDynamicModel& dynamics,
        int edge,
        double parameter
    )
    {
        const auto derivative = dynamics.Evaluate(
            MakeEdgeState(edge, parameter)
        );
        if (!derivative.has_value()
            || !derivative->IsFinite()
            || !derivative->IsTangent()) {
            return std::nullopt;
        }

        return edge == 2 ? derivative->dy : derivative->dx;
    }

    [[nodiscard]] std::optional<double> BisectEdgeRoot(
        const SimplexDynamicModel& dynamics,
        int edge,
        double left,
        double right,
        const SimplexEquilibriumSearchSettings& settings
    )
    {
        auto leftValue = EvaluateEdgeComponent(dynamics, edge, left);
        auto rightValue = EvaluateEdgeComponent(dynamics, edge, right);
        if (!leftValue.has_value() || !rightValue.has_value()) {
            return std::nullopt;
        }

        for (int iteration = 0; iteration < 80; ++iteration) {
            const double midpoint = 0.5 * (left + right);
            const auto middleValue = EvaluateEdgeComponent(
                dynamics,
                edge,
                midpoint
            );
            if (!middleValue.has_value()) {
                return std::nullopt;
            }

            if (std::abs(*middleValue) <= settings.residualTolerance
                || right - left <= settings.duplicateDistance * 0.25) {
                return midpoint;
            }

            if ((*leftValue < 0.0 && *middleValue > 0.0)
                || (*leftValue > 0.0 && *middleValue < 0.0)) {
                right = midpoint;
                rightValue = middleValue;
            }
            else {
                left = midpoint;
                leftValue = middleValue;
            }
        }

        return 0.5 * (left + right);
    }

    void AddReplicatorBoundaryEquilibria(
        const SimplexDynamicModel& dynamics,
        const SimplexEquilibriumSearchSettings& settings,
        std::vector<SimplexEquilibrium>& equilibria
    )
    {
        const std::array<SimplexState, 3> vertices{
            SimplexState::Normalized(1.0, 0.0, 0.0),
            SimplexState::Normalized(0.0, 1.0, 0.0),
            SimplexState::Normalized(0.0, 0.0, 1.0)
        };
        for (const SimplexState& vertex : vertices) {
            const auto equilibrium = VerifyState(dynamics, vertex, settings);
            if (equilibrium.has_value()) {
                AddIfUnique(
                    *equilibrium,
                    equilibria,
                    settings.duplicateDistance
                );
            }
        }

        const int edgeResolution = std::max(64, settings.latticeResolution * 4);
        for (int edge = 0; edge < 3; ++edge) {
            bool edgeAppearsDegenerate = true;
            std::vector<double> values;
            values.reserve(static_cast<std::size_t>(edgeResolution - 1));
            for (int index = 1; index < edgeResolution; ++index) {
                const auto value = EvaluateEdgeComponent(
                    dynamics,
                    edge,
                    static_cast<double>(index) / edgeResolution
                );
                if (!value.has_value()) {
                    values.clear();
                    break;
                }
                edgeAppearsDegenerate = edgeAppearsDegenerate
                    && std::abs(*value) <= settings.residualTolerance;
                values.push_back(*value);
            }

            if (values.empty()) {
                continue;
            }

            if (edgeAppearsDegenerate) {
                const auto representative = VerifyState(
                    dynamics,
                    MakeEdgeState(edge, 0.5),
                    settings,
                    false
                );
                if (representative.has_value()) {
                    AddIfUnique(
                        *representative,
                        equilibria,
                        settings.duplicateDistance
                    );
                }
                continue;
            }

            if (std::abs(values.front()) <= settings.residualTolerance) {
                const auto equilibrium = VerifyState(
                    dynamics,
                    MakeEdgeState(edge, 1.0 / edgeResolution),
                    settings
                );
                if (equilibrium.has_value()) {
                    AddIfUnique(
                        *equilibrium,
                        equilibria,
                        settings.duplicateDistance
                    );
                }
            }

            for (int index = 2; index < edgeResolution; ++index) {
                const double previousParameter =
                    static_cast<double>(index - 1) / edgeResolution;
                const double parameter =
                    static_cast<double>(index) / edgeResolution;
                const double previousValue = values[
                    static_cast<std::size_t>(index - 2)
                ];
                const double value = values[
                    static_cast<std::size_t>(index - 1)
                ];
                const bool crosses =
                    (previousValue < 0.0 && value > 0.0)
                    || (previousValue > 0.0 && value < 0.0);
                const bool sampleIsRoot =
                    std::abs(value) <= settings.residualTolerance;
                if (crosses || sampleIsRoot) {
                    const auto root = sampleIsRoot
                        ? std::optional<double>{ parameter }
                        : BisectEdgeRoot(
                            dynamics,
                            edge,
                            previousParameter,
                            parameter,
                            settings
                        );
                    if (root.has_value()) {
                        const auto equilibrium = VerifyState(
                            dynamics,
                            MakeEdgeState(edge, *root),
                            settings
                        );
                        if (equilibrium.has_value()) {
                            AddIfUnique(
                                *equilibrium,
                                equilibria,
                                settings.duplicateDistance
                            );
                        }
                    }
                }
            }
        }
    }

    void AddBestResponseEquilibria(
        const SimplexDynamicModel& dynamics,
        const SimplexEquilibriumSearchSettings& settings,
        std::vector<SimplexEquilibrium>& equilibria
    )
    {
        // B(s) is uniform on a non-empty support. Therefore G(s)=B(s)-s can
        // vanish only at one of these seven support-uniform states.
        const std::array<SimplexState, 7> candidates{
            SimplexState::Normalized(1.0, 0.0, 0.0),
            SimplexState::Normalized(0.0, 1.0, 0.0),
            SimplexState::Normalized(0.0, 0.0, 1.0),
            SimplexState::Normalized(1.0, 1.0, 0.0),
            SimplexState::Normalized(1.0, 0.0, 1.0),
            SimplexState::Normalized(0.0, 1.0, 1.0),
            SimplexState::Normalized(1.0, 1.0, 1.0)
        };

        for (const SimplexState& candidate : candidates) {
            const auto equilibrium = VerifyState(dynamics, candidate, settings);
            if (equilibrium.has_value()) {
                AddIfUnique(
                    *equilibrium,
                    equilibria,
                    settings.duplicateDistance
                );
            }
        }
    }
}

const char* SimplexEquilibriumLocationName(
    SimplexEquilibriumLocation location
) noexcept
{
    switch (location) {
    case SimplexEquilibriumLocation::Interior:
        return "Interior";
    case SimplexEquilibriumLocation::EdgeXY:
        return "Cooperator-Defector edge";
    case SimplexEquilibriumLocation::EdgeXZ:
        return "Cooperator-Loner edge";
    case SimplexEquilibriumLocation::EdgeYZ:
        return "Defector-Loner edge";
    case SimplexEquilibriumLocation::VertexX:
        return "Cooperator vertex";
    case SimplexEquilibriumLocation::VertexY:
        return "Defector vertex";
    case SimplexEquilibriumLocation::VertexZ:
        return "Loner vertex";
    }

    return "Unknown";
}

bool SimplexEquilibriumSearchSettings::IsComputable() const noexcept
{
    return latticeResolution >= 3
        && maximumNewtonIterations > 0
        && maximumBacktrackingSteps > 0
        && std::isfinite(finiteDifferenceStep)
        && std::isfinite(residualTolerance)
        && std::isfinite(duplicateDistance)
        && std::isfinite(interiorMargin)
        && std::isfinite(minimumJacobianDeterminant)
        && finiteDifferenceStep > 0.0
        && residualTolerance > 0.0
        && duplicateDistance > 0.0
        && interiorMargin >= 0.0
        && interiorMargin < 1.0 / 3.0
        && minimumJacobianDeterminant > 0.0;
}

std::optional<std::vector<SimplexEquilibrium>>
SimplexEquilibriumFinder::Find(
    const SimplexDynamicModel& dynamics,
    const SimplexEquilibriumSearchSettings& settings
) const
{
    if (!settings.IsComputable()) {
        return std::nullopt;
    }

    if (!EvaluateRootFunction(
            dynamics,
            Coordinates{ 1.0 / 3.0, 1.0 / 3.0 },
            settings
        ).has_value()) {
        return std::nullopt;
    }

    std::vector<SimplexEquilibrium> equilibria;

    if (dynamics.Kind() == DynamicsKind::EqualSplitBestResponse) {
        AddBestResponseEquilibria(dynamics, settings, equilibria);
        return equilibria;
    }

    const auto tryAdd = [&](const Coordinates& seed) {
        const auto equilibrium = RefineEquilibrium(dynamics, seed, settings);
        if (equilibrium.has_value()
            && !IsDuplicate(
                *equilibrium,
                equilibria,
                settings.duplicateDistance
            )) {
            equilibria.push_back(*equilibrium);
        }
    };

    const double resolution = static_cast<double>(settings.latticeResolution);
    for (int xIndex = 1;
         xIndex < settings.latticeResolution - 1;
         ++xIndex) {
        for (int yIndex = 1;
             xIndex + yIndex < settings.latticeResolution;
             ++yIndex) {
            tryAdd(Coordinates{
                static_cast<double>(xIndex) / resolution,
                static_cast<double>(yIndex) / resolution
            });
        }
    }

    const double nearEdge = std::max(1e-3, settings.interiorMargin * 10.0);
    tryAdd(Coordinates{ 1.0 / 3.0, 1.0 / 3.0 });
    tryAdd(Coordinates{ 1.0 - 2.0 * nearEdge, nearEdge });
    tryAdd(Coordinates{ nearEdge, 1.0 - 2.0 * nearEdge });
    tryAdd(Coordinates{ nearEdge, nearEdge });

    for (SimplexEquilibrium& equilibrium : equilibria) {
        equilibrium.location = ClassifyLocation(
            equilibrium.state,
            settings.duplicateDistance
        );
    }

    if (dynamics.Kind() == DynamicsKind::Replicator) {
        AddReplicatorBoundaryEquilibria(dynamics, settings, equilibria);
    }

    std::sort(
        equilibria.begin(),
        equilibria.end(),
        [](const SimplexEquilibrium& left,
           const SimplexEquilibrium& right) {
            if (left.state.X() != right.state.X()) {
                return left.state.X() < right.state.X();
            }
            return left.state.Y() < right.state.Y();
        }
    );

    return equilibria;
}
