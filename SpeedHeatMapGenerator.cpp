#include "SpeedHeatMapGenerator.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace
{
    [[nodiscard]] std::uint32_t SampleIndex(int resolution, int x, int y)
    {
        const int rowStart = x * (resolution + 1) - x * (x - 1) / 2;
        return static_cast<std::uint32_t>(rowStart + y);
    }

    [[nodiscard]] float NormalizeSpeed(
        double speed,
        double minimum,
        double maximum
    ) noexcept
    {
        const double range = maximum - minimum;
        if (!std::isfinite(speed)
            || !std::isfinite(range)
            || range <= std::numeric_limits<double>::epsilon()
                * std::max({ 1.0, std::abs(minimum), std::abs(maximum) })) {
            return 0.0F;
        }

        return static_cast<float>(std::clamp(
            (speed - minimum) / range,
            0.0,
            1.0
        ));
    }

    [[nodiscard]] HeatMapColor Interpolate(
        const HeatMapColor& first,
        const HeatMapColor& second,
        float amount
    ) noexcept
    {
        return HeatMapColor{
            first.red + (second.red - first.red) * amount,
            first.green + (second.green - first.green) * amount,
            first.blue + (second.blue - first.blue) * amount
        };
    }
}

bool SpeedHeatMapSettings::IsComputable() const noexcept
{
    if (resolution < 2 || resolution > 256) {
        return false;
    }

    if (normalization == HeatMapNormalizationMode::Relative) {
        return true;
    }

    return std::isfinite(lockedMinimumSpeed)
        && std::isfinite(lockedMaximumSpeed)
        && lockedMinimumSpeed >= 0.0
        && lockedMaximumSpeed > lockedMinimumSpeed;
}

std::optional<SpeedHeatMapResult> SpeedHeatMapGenerator::Generate(
    const SimplexDynamicModel& dynamics,
    const SpeedHeatMapSettings& settings
) const
{
    if (!settings.IsComputable()) {
        return std::nullopt;
    }

    SpeedHeatMapResult result;
    result.resolution = settings.resolution;
    const std::size_t sampleCount = static_cast<std::size_t>(
        (settings.resolution + 1) * (settings.resolution + 2) / 2
    );
    result.samples.reserve(sampleCount);
    result.sampledMinimumSpeed = std::numeric_limits<double>::infinity();
    result.sampledMaximumSpeed = 0.0;

    const double resolution = static_cast<double>(settings.resolution);
    for (int xIndex = 0; xIndex <= settings.resolution; ++xIndex) {
        for (int yIndex = 0;
             xIndex + yIndex <= settings.resolution;
             ++yIndex) {
            const double x = static_cast<double>(xIndex) / resolution;
            const double y = static_cast<double>(yIndex) / resolution;
            const SimplexState state = SimplexState::Normalized(
                x,
                y,
                1.0 - x - y
            );
            const auto derivative = dynamics.Evaluate(state);
            if (!derivative.has_value()
                || !derivative->IsFinite()
                || !derivative->IsTangent()) {
                return std::nullopt;
            }

            const double speed = std::sqrt(
                derivative->dx * derivative->dx
                + derivative->dy * derivative->dy
                + derivative->dz * derivative->dz
            );
            if (!std::isfinite(speed)) {
                return std::nullopt;
            }

            result.sampledMinimumSpeed = std::min(
                result.sampledMinimumSpeed,
                speed
            );
            result.sampledMaximumSpeed = std::max(
                result.sampledMaximumSpeed,
                speed
            );
            result.samples.push_back(SpeedHeatMapSample{
                state,
                speed,
                0.0F,
                dynamics.RegionIdentifier(state)
            });
        }
    }

    result.displayedMinimumSpeed =
        settings.normalization == HeatMapNormalizationMode::Relative
            ? result.sampledMinimumSpeed
            : settings.lockedMinimumSpeed;
    result.displayedMaximumSpeed =
        settings.normalization == HeatMapNormalizationMode::Relative
            ? result.sampledMaximumSpeed
            : settings.lockedMaximumSpeed;
    for (SpeedHeatMapSample& sample : result.samples) {
        sample.normalizedSpeed = NormalizeSpeed(
            sample.speed,
            result.displayedMinimumSpeed,
            result.displayedMaximumSpeed
        );
    }

    result.triangleIndices.reserve(
        static_cast<std::size_t>(settings.resolution)
        * static_cast<std::size_t>(settings.resolution)
        * 3U
    );
    result.mixedRegionTriangles.reserve(
        static_cast<std::size_t>(settings.resolution)
        * static_cast<std::size_t>(settings.resolution)
    );
    const auto addTriangle = [&](std::uint32_t first,
                                 std::uint32_t second,
                                 std::uint32_t third) {
        result.triangleIndices.push_back(first);
        result.triangleIndices.push_back(second);
        result.triangleIndices.push_back(third);
        const std::uint32_t firstRegion =
            result.samples[first].regionIdentifier;
        const bool mixed = firstRegion != result.samples[second].regionIdentifier
            || firstRegion != result.samples[third].regionIdentifier;
        result.mixedRegionTriangles.push_back(mixed ? 1U : 0U);
    };

    for (int xIndex = 0; xIndex < settings.resolution; ++xIndex) {
        const int trianglesInRow = settings.resolution - xIndex;
        for (int yIndex = 0; yIndex < trianglesInRow; ++yIndex) {
            const std::uint32_t a = SampleIndex(
                settings.resolution,
                xIndex,
                yIndex
            );
            const std::uint32_t b = SampleIndex(
                settings.resolution,
                xIndex + 1,
                yIndex
            );
            const std::uint32_t c = SampleIndex(
                settings.resolution,
                xIndex,
                yIndex + 1
            );
            addTriangle(a, b, c);

            if (yIndex + 1 < trianglesInRow) {
                const std::uint32_t d = SampleIndex(
                    settings.resolution,
                    xIndex + 1,
                    yIndex + 1
                );
                addTriangle(b, d, c);
            }
        }
    }

    return result;
}

HeatMapColor SpeedHeatMapGenerator::Palette(float normalizedSpeed) noexcept
{
    constexpr std::array<HeatMapColor, 6> colors{
        HeatMapColor{ 0.02F, 0.05F, 0.35F },
        HeatMapColor{ 0.00F, 0.75F, 1.00F },
        HeatMapColor{ 0.00F, 0.75F, 0.22F },
        HeatMapColor{ 1.00F, 0.92F, 0.00F },
        HeatMapColor{ 1.00F, 0.40F, 0.00F },
        HeatMapColor{ 0.88F, 0.00F, 0.00F }
    };
    const float clamped = std::clamp(normalizedSpeed, 0.0F, 1.0F);
    const float scaled = clamped * static_cast<float>(colors.size() - 1U);
    const std::size_t first = std::min(
        static_cast<std::size_t>(scaled),
        colors.size() - 2U
    );
    const float amount = scaled - static_cast<float>(first);
    return Interpolate(colors[first], colors[first + 1U], amount);
}
