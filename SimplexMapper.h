#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

#include "SimplexState.h"
#include "Vec2f.h"

class SimplexMapper final
{
public:
    constexpr SimplexMapper(
        Vec2f cooperatorsVertex,
        Vec2f defectorsVertex,
        Vec2f lonersVertex
    ) noexcept
        : cooperatorsVertex_(cooperatorsVertex),
          defectorsVertex_(defectorsVertex),
          lonersVertex_(lonersVertex)
    {
    }

    [[nodiscard]] Vec2f ToNdcPosition(const SimplexState& state) const noexcept
    {
        return {
            static_cast<float>(
                state.X() * cooperatorsVertex_.x
                + state.Y() * defectorsVertex_.x
                + state.Z() * lonersVertex_.x
            ),
            static_cast<float>(
                state.X() * cooperatorsVertex_.y
                + state.Y() * defectorsVertex_.y
                + state.Z() * lonersVertex_.y
            )
        };
    }

    [[nodiscard]] std::optional<SimplexState> FromNdcPosition(
        Vec2f position
    ) const
    {
        constexpr double kClickEpsilon = 1e-5;
        const auto barycentric = ToBarycentricCoordinates(position);
        if (!barycentric.has_value()) {
            return std::nullopt;
        }

        return SimplexState::TryCreate(
            barycentric->x,
            barycentric->y,
            barycentric->z,
            kClickEpsilon
        );
    }

    [[nodiscard]] std::optional<SimplexState> FromNdcPositionClamped(
        Vec2f position
    ) const
    {
        if (const auto inside = FromNdcPosition(position); inside.has_value()) {
            return inside;
        }

        const Vec2f onCooperatorDefector = ClosestPointOnSegment(
            position,
            cooperatorsVertex_,
            defectorsVertex_
        );
        const Vec2f onDefectorLoner = ClosestPointOnSegment(
            position,
            defectorsVertex_,
            lonersVertex_
        );
        const Vec2f onLonerCooperator = ClosestPointOnSegment(
            position,
            lonersVertex_,
            cooperatorsVertex_
        );

        Vec2f closest = onCooperatorDefector;
        double closestDistance = SquaredDistance(position, closest);

        const double defectorLonerDistance = SquaredDistance(
            position,
            onDefectorLoner
        );
        if (defectorLonerDistance < closestDistance) {
            closest = onDefectorLoner;
            closestDistance = defectorLonerDistance;
        }

        const double lonerCooperatorDistance = SquaredDistance(
            position,
            onLonerCooperator
        );
        if (lonerCooperatorDistance < closestDistance) {
            closest = onLonerCooperator;
        }

        const auto barycentric = ToBarycentricCoordinates(closest);
        if (!barycentric.has_value()) {
            return std::nullopt;
        }

        return SimplexState::TryCreate(
            barycentric->x,
            barycentric->y,
            barycentric->z,
            1e-7
        );
    }

private:
    struct BarycentricCoordinates final
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    [[nodiscard]] static double SquaredDistance(
        Vec2f left,
        Vec2f right
    ) noexcept
    {
        const double dx = static_cast<double>(left.x) - right.x;
        const double dy = static_cast<double>(left.y) - right.y;
        return dx * dx + dy * dy;
    }

    [[nodiscard]] static Vec2f ClosestPointOnSegment(
        Vec2f point,
        Vec2f start,
        Vec2f end
    ) noexcept
    {
        const double segmentX = static_cast<double>(end.x) - start.x;
        const double segmentY = static_cast<double>(end.y) - start.y;
        const double lengthSquared = segmentX * segmentX + segmentY * segmentY;

        if (lengthSquared <= 0.0) {
            return start;
        }

        const double pointX = static_cast<double>(point.x) - start.x;
        const double pointY = static_cast<double>(point.y) - start.y;
        const double parameter = std::clamp(
            (pointX * segmentX + pointY * segmentY) / lengthSquared,
            0.0,
            1.0
        );

        return {
            static_cast<float>(start.x + parameter * segmentX),
            static_cast<float>(start.y + parameter * segmentY)
        };
    }

    [[nodiscard]] std::optional<BarycentricCoordinates>
    ToBarycentricCoordinates(Vec2f position) const
    {
        constexpr double kDegenerateTriangleEpsilon = 1e-12;

        const double ax = cooperatorsVertex_.x;
        const double ay = cooperatorsVertex_.y;
        const double bx = defectorsVertex_.x;
        const double by = defectorsVertex_.y;
        const double cx = lonersVertex_.x;
        const double cy = lonersVertex_.y;
        const double px = position.x;
        const double py = position.y;

        const double denominator =
            (by - cy) * (ax - cx)
            + (cx - bx) * (ay - cy);

        if (!std::isfinite(denominator)
            || std::abs(denominator) <= kDegenerateTriangleEpsilon) {
            return std::nullopt;
        }

        const double xValue =
            ((by - cy) * (px - cx) + (cx - bx) * (py - cy))
            / denominator;
        const double yValue =
            ((cy - ay) * (px - cx) + (ax - cx) * (py - cy))
            / denominator;
        const double zValue = 1.0 - xValue - yValue;

        if (!std::isfinite(xValue)
            || !std::isfinite(yValue)
            || !std::isfinite(zValue)) {
            return std::nullopt;
        }

        return BarycentricCoordinates{ xValue, yValue, zValue };
    }

    Vec2f cooperatorsVertex_;
    Vec2f defectorsVertex_;
    Vec2f lonersVertex_;
};
