#pragma once

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
    )
        : cooperatorsVertex_(cooperatorsVertex),
        defectorsVertex_(defectorsVertex),
        lonersVertex_(lonersVertex)
    {
    }

    [[nodiscard]] Vec2f ToNdcPosition(const SimplexState& state) const
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

    [[nodiscard]] std::optional<SimplexState> FromNdcPosition(Vec2f position) const
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

    [[nodiscard]] std::optional<SimplexState> FromNdcPositionClamped(Vec2f position) const
    {
        const auto barycentric = ToBarycentricCoordinates(position);

        if (!barycentric.has_value()) {
            return std::nullopt;
        }

        return SimplexState::Normalized(
            barycentric->x,
            barycentric->y,
            barycentric->z
        );
    }

private:
    struct BarycentricCoordinates
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    [[nodiscard]] std::optional<BarycentricCoordinates> ToBarycentricCoordinates(Vec2f position) const
    {
        constexpr double kDegenerateTriangleEpsilon = 1e-7;

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

        if (std::abs(denominator) <= kDegenerateTriangleEpsilon) {
            return std::nullopt;
        }

        const double xValue =
            ((by - cy) * (px - cx)
                + (cx - bx) * (py - cy))
            / denominator;

        const double yValue =
            ((cy - ay) * (px - cx)
                + (ax - cx) * (py - cy))
            / denominator;

        const double zValue = 1.0 - xValue - yValue;

        return BarycentricCoordinates{
            xValue,
            yValue,
            zValue
        };
    }

    Vec2f cooperatorsVertex_;
    Vec2f defectorsVertex_;
    Vec2f lonersVertex_;
};