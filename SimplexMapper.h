#pragma once

#include "SimplexState.h"
#include "Vec2f.h"

class SimplexMapper final
{
public:
	constexpr SimplexMapper(Vec2f cooperatorsVertex, Vec2f defectorsVertex, Vec2f lonersVertex)
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
				+ state.Z() * lonersVertex_.x),
			static_cast<float>(
				state.X() * cooperatorsVertex_.y
				+ state.Y() * defectorsVertex_.y
				+ state.Z() * lonersVertex_.y)
		};
	}

private:
	Vec2f cooperatorsVertex_;
	Vec2f defectorsVertex_;
	Vec2f lonersVertex_;
};