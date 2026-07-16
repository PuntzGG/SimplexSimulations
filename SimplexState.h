#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

class SimplexState final
{
public:
    static constexpr double kDefaultEpsilon = 1e-9;

    [[nodiscard]] static bool IsValidValues(
        double xValue,
        double yValue,
        double zValue,
        double epsilon = kDefaultEpsilon
    )
    {
        if (!std::isfinite(xValue)
            || !std::isfinite(yValue)
            || !std::isfinite(zValue)
            || !std::isfinite(epsilon)
            || epsilon < 0.0) {
            return false;
        }

        return xValue >= -epsilon
            && yValue >= -epsilon
            && zValue >= -epsilon
            && std::abs((xValue + yValue + zValue) - 1.0) <= epsilon;
    }

    [[nodiscard]] static std::optional<SimplexState> TryCreate(
        double xValue,
        double yValue,
        double zValue,
        double epsilon = kDefaultEpsilon
    )
    {
        if (!IsValidValues(xValue, yValue, zValue, epsilon)) {
            return std::nullopt;
        }

        xValue = ClampTinyNegativeToZero(xValue, epsilon);
        yValue = ClampTinyNegativeToZero(yValue, epsilon);
        zValue = ClampTinyNegativeToZero(zValue, epsilon);

        const double sum = xValue + yValue + zValue;
        if (!std::isfinite(sum) || sum <= 0.0) {
            return std::nullopt;
        }

        // Valid inputs may still differ from an exact unit sum by a tiny
        // floating-point amount. Store an exact normalized representative.
        return SimplexState{
            xValue / sum,
            yValue / sum,
            zValue / sum
        };
    }

    [[nodiscard]] static SimplexState Normalized(
        double xValue,
        double yValue,
        double zValue
    )
    {
        if (!std::isfinite(xValue)
            || !std::isfinite(yValue)
            || !std::isfinite(zValue)) {
            return SimplexState{ 1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0 };
        }

        xValue = std::max(0.0, xValue);
        yValue = std::max(0.0, yValue);
        zValue = std::max(0.0, zValue);

        const double sum = xValue + yValue + zValue;
        if (!std::isfinite(sum) || sum <= 0.0) {
            return SimplexState{ 1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0 };
        }

        return SimplexState{
            xValue / sum,
            yValue / sum,
            zValue / sum
        };
    }

    [[nodiscard]] double X() const noexcept { return x_; }
    [[nodiscard]] double Y() const noexcept { return y_; }
    [[nodiscard]] double Z() const noexcept { return z_; }

    [[nodiscard]] bool IsValid(double epsilon = kDefaultEpsilon) const
    {
        return IsValidValues(x_, y_, z_, epsilon);
    }

private:
    constexpr SimplexState(double xValue, double yValue, double zValue) noexcept
        : x_(xValue),
          y_(yValue),
          z_(zValue)
    {
    }

    [[nodiscard]] static double ClampTinyNegativeToZero(
        double value,
        double epsilon
    ) noexcept
    {
        return value < 0.0 && value >= -epsilon ? 0.0 : value;
    }

    double x_ = 0.0;
    double y_ = 0.0;
    double z_ = 0.0;
};
