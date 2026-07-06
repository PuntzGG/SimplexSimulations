#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

struct SimplexState final
{
public:
	static constexpr double kDefaultEpsilon = 1e-9;

	[[nodiscard]] static bool IsValidValues(double xValue, double yValue, double zValue, double epsilon = kDefaultEpsilon)
	{
		return xValue >= -epsilon
			&& yValue >= -epsilon
			&& zValue >= -epsilon
			&& std::abs((xValue + yValue + zValue) - 1.0) <= epsilon;
	}
	[[nodiscard]] static std::optional<SimplexState> TryCreate(double xValue, double yValue, double zValue, double epsilon = kDefaultEpsilon)
	{
		if (!IsValidValues(xValue, yValue, zValue, epsilon)) {
			return std::nullopt;
		}

		return SimplexState{
			ClampTinyNegativeToZero(xValue, epsilon),
			ClampTinyNegativeToZero(yValue, epsilon),
			ClampTinyNegativeToZero(zValue, epsilon)
		};
	}

	[[nodiscard]] static SimplexState Normalized(double xValue, double yValue, double zValue)
	{
		xValue = std::max(0.0, xValue);
		yValue = std::max(0.0, yValue);
		zValue = std::max(0.0, zValue);

		const double sum = xValue + yValue + zValue;

		if (sum <= 0.0) {
			return SimplexState(1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0);
		}

		return SimplexState{
			xValue / sum,
			yValue / sum,
			zValue / sum
		};
	}

	[[nodiscard]] double X() const { return x_; }
	[[nodiscard]] double Y() const { return y_; }
	[[nodiscard]] double Z() const { return z_; }

	[[nodiscard]] bool IsValid(double epsilon = kDefaultEpsilon) const
	{
		return IsValidValues(x_, y_, z_, epsilon);
	}

private:
	constexpr SimplexState(double xValue, double yValue, double zValue)
	:	x_(xValue),
		y_(yValue),
		z_(zValue)
	{}

	[[nodiscard]] static double ClampTinyNegativeToZero(double value, double epsilon)
	{
		if (value < 0.0 && value >= -epsilon) {
			return 0.0;
		}

		return value;
	}

	double x_ = 0.0; //Cooperators
	double y_ = 0.0; //Defectors
	double z_ = 0.0; //Loners
};