#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "SimplexDynamicModel.h"
#include "SimplexState.h"

enum class HeatMapNormalizationMode
{
    Relative,
    LockedRange
};

struct SpeedHeatMapSettings final
{
    int resolution = 48;
    HeatMapNormalizationMode normalization =
        HeatMapNormalizationMode::Relative;
    double lockedMinimumSpeed = 0.0;
    double lockedMaximumSpeed = 1.0;

    [[nodiscard]] bool IsComputable() const noexcept;
};

struct SpeedHeatMapSample final
{
    SimplexState state;
    double speed = 0.0;
    float normalizedSpeed = 0.0F;
    std::uint32_t regionIdentifier = 0U;
};

struct SpeedHeatMapResult final
{
    int resolution = 0;
    double sampledMinimumSpeed = 0.0;
    double sampledMaximumSpeed = 0.0;
    double displayedMinimumSpeed = 0.0;
    double displayedMaximumSpeed = 0.0;
    std::vector<SpeedHeatMapSample> samples;
    std::vector<std::uint32_t> triangleIndices;
    std::vector<std::uint8_t> mixedRegionTriangles;
};

struct HeatMapColor final
{
    float red = 0.0F;
    float green = 0.0F;
    float blue = 0.0F;
};

class SpeedHeatMapGenerator final
{
public:
    [[nodiscard]] std::optional<SpeedHeatMapResult> Generate(
        const SimplexDynamicModel& dynamics,
        const SpeedHeatMapSettings& settings
    ) const;

    [[nodiscard]] static HeatMapColor Palette(float normalizedSpeed) noexcept;
};
