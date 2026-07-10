#include "SimplexEquilibriumFinder.h"

#include <algorithm>
#include <cmath>

namespace
{
    constexpr double kInteriorMargin = 1e-12;
    constexpr int kMaximumBacktrackingSteps = 24;

    struct Coordinates
    {
        double x;
        double y;
    };

    struct VectorFieldValue
    {
        double dx;
        double dy;
        double dz;
        double residual;
    };

    struct Jacobian
    {
        double dxByDx;
        double dxByDy;
        double dyByDx;
        double dyByDy;
    };

    [[nodiscard]] std::optional<SimplexState> MakeInteriorState(
        const Coordinates& coordinates
    )
    {
        const double z = 1.0 - coordinates.x - coordinates.y;

        if (!std::isfinite(coordinates.x)
            || !std::isfinite(coordinates.y)
            || !std::isfinite(z)
            || coordinates.x <= kInteriorMargin
            || coordinates.y <= kInteriorMargin
            || z <= kInteriorMargin) {
            return std::nullopt;
        }

        return SimplexState::TryCreate(
            coordinates.x,
            coordinates.y,
            z
        );
    }

    [[nodiscard]] std::optional<VectorFieldValue> EvaluateVectorField(
        const SimplexDynamicModel& dynamics,
        const Coordinates& coordinates
    )
    {
        const auto state = MakeInteriorState(coordinates);

        if (!state.has_value()) {
            return std::nullopt;
        }

        const auto derivative = dynamics.Evaluate(*state);

        if (!derivative.has_value()
            || !std::isfinite(derivative->dx)
            || !std::isfinite(derivative->dy)
            || !std::isfinite(derivative->dz)) {
            return std::nullopt;
        }

        return VectorFieldValue{
            derivative->dx,
            derivative->dy,
            derivative->dz,
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
        const double availableX = std::min(
            coordinates.x - kInteriorMargin,
            1.0 - kInteriorMargin - coordinates.x - coordinates.y
        );

        const double availableY = std::min(
            coordinates.y - kInteriorMargin,
            1.0 - kInteriorMargin - coordinates.x - coordinates.y
        );

        const double xStep = std::min(
            settings.finiteDifferenceStep,
            availableX * 0.5
        );

        const double yStep = std::min(
            settings.finiteDifferenceStep,
            availableY * 0.5
        );

        if (xStep <= 0.0 || yStep <= 0.0) {
            return std::nullopt;
        }

        const auto positiveX = EvaluateVectorField(
            dynamics,
            Coordinates{ coordinates.x + xStep, coordinates.y }
        );

        const auto negativeX = EvaluateVectorField(
            dynamics,
            Coordinates{ coordinates.x - xStep, coordinates.y }
        );

        const auto positiveY = EvaluateVectorField(
            dynamics,
            Coordinates{ coordinates.x, coordinates.y + yStep }
        );

        const auto negativeY = EvaluateVectorField(
            dynamics,
            Coordinates{ coordinates.x, coordinates.y - yStep }
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
        auto value = EvaluateVectorField(dynamics, coordinates);

        if (!value.has_value()) {
            return std::nullopt;
        }

        for (int iteration = 0;
            iteration < settings.maximumNewtonIterations;
            ++iteration) {
            if (value->residual <= settings.residualTolerance) {
                const auto state = MakeInteriorState(coordinates);

                if (!state.has_value()) {
                    return std::nullopt;
                }

                return SimplexEquilibrium{
                    *state,
                    value->residual
                };
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
                || std::abs(determinant) <= 1e-14) {
                return std::nullopt;
            }

            const double stepX =
                (
                    jacobian->dxByDy * value->dy
                    - jacobian->dyByDy * value->dx
                    )
                / determinant;

            const double stepY =
                (
                    jacobian->dyByDx * value->dx
                    - jacobian->dxByDx * value->dy
                    )
                / determinant;

            if (!std::isfinite(stepX) || !std::isfinite(stepY)) {
                return std::nullopt;
            }

            bool acceptedStep = false;
            double scale = 1.0;

            for (int attempt = 0;
                attempt < kMaximumBacktrackingSteps;
                ++attempt) {
                const Coordinates candidate{
                    coordinates.x + scale * stepX,
                    coordinates.y + scale * stepY
                };

                const auto candidateValue = EvaluateVectorField(
                    dynamics,
                    candidate
                );

                if (candidateValue.has_value()
                    && candidateValue->residual < value->residual) {
                    coordinates = candidate;
                    value = candidateValue;
                    acceptedStep = true;
                    break;
                }

                scale *= 0.5;
            }

            if (!acceptedStep) {
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
        for (const SimplexEquilibrium& existing : equilibria) {
            const double dx = candidate.state.X() - existing.state.X();
            const double dy = candidate.state.Y() - existing.state.Y();
            const double dz = candidate.state.Z() - existing.state.Z();

            const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

            if (distance <= duplicateDistance) {
                return true;
            }
        }

        return false;
    }
}

bool SimplexEquilibriumSearchSettings::IsComputable() const
{
    return latticeResolution >= 3
        && maximumNewtonIterations > 0
        && std::isfinite(finiteDifferenceStep)
        && std::isfinite(residualTolerance)
        && std::isfinite(duplicateDistance)
        && finiteDifferenceStep > 0.0
        && residualTolerance > 0.0
        && duplicateDistance > 0.0;
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
        Coordinates{ 1.0 / 3.0, 1.0 / 3.0 }
    ).has_value()) {
        return std::nullopt;
    }

    std::vector<SimplexEquilibrium> equilibria;

    auto tryAddEquilibrium = [&](const Coordinates& seed)
        {
            const auto equilibrium = RefineEquilibrium(
                dynamics,
                seed,
                settings
            );

            if (equilibrium.has_value()
                && !IsDuplicate(
                    *equilibrium,
                    equilibria,
                    settings.duplicateDistance
                )) {
                equilibria.push_back(*equilibrium);
            }
        };

    const double resolution =
        static_cast<double>(settings.latticeResolution);

    for (int xIndex = 1;
        xIndex < settings.latticeResolution - 1;
        ++xIndex) {
        for (int yIndex = 1;
            xIndex + yIndex < settings.latticeResolution;
            ++yIndex) {
            tryAddEquilibrium(Coordinates{
                static_cast<double>(xIndex) / resolution,
                static_cast<double>(yIndex) / resolution
                });
        }
    }

    constexpr double kNearEdge = 1e-3;

    tryAddEquilibrium(Coordinates{ 1.0 / 3.0, 1.0 / 3.0 });
    tryAddEquilibrium(Coordinates{ 1.0 - 2.0 * kNearEdge, kNearEdge });
    tryAddEquilibrium(Coordinates{ kNearEdge, 1.0 - 2.0 * kNearEdge });
    tryAddEquilibrium(Coordinates{ kNearEdge, kNearEdge });

    std::sort(
        equilibria.begin(),
        equilibria.end(),
        [](const SimplexEquilibrium& left,
            const SimplexEquilibrium& right)
        {
            if (left.state.X() != right.state.X()) {
                return left.state.X() < right.state.X();
            }

            return left.state.Y() < right.state.Y();
        }
    );

    return equilibria;
}