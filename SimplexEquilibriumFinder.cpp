#include "SimplexEquilibriumFinder.h"

#include <algorithm>
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

    [[nodiscard]] std::optional<VectorFieldValue> EvaluateVectorField(
        const SimplexDynamicModel& dynamics,
        const Coordinates& coordinates,
        const SimplexEquilibriumSearchSettings& settings
    )
    {
        const auto state = MakeInteriorState(coordinates, settings.interiorMargin);
        if (!state.has_value()) {
            return std::nullopt;
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

        const auto positiveX = EvaluateVectorField(
            dynamics,
            Coordinates{ coordinates.x + xStep, coordinates.y },
            settings
        );
        const auto negativeX = EvaluateVectorField(
            dynamics,
            Coordinates{ coordinates.x - xStep, coordinates.y },
            settings
        );
        const auto positiveY = EvaluateVectorField(
            dynamics,
            Coordinates{ coordinates.x, coordinates.y + yStep },
            settings
        );
        const auto negativeY = EvaluateVectorField(
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
        auto value = EvaluateVectorField(dynamics, coordinates, settings);
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
                const auto candidateValue = EvaluateVectorField(
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

    if (!EvaluateVectorField(
            dynamics,
            Coordinates{ 1.0 / 3.0, 1.0 / 3.0 },
            settings
        ).has_value()) {
        return std::nullopt;
    }

    std::vector<SimplexEquilibrium> equilibria;

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
