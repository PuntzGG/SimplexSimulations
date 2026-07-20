#include "StreamlineFieldGenerator.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

#include "SimplexTrajectoryIntegrator.h"
#include "TrajectorySettings.h"

namespace
{
    [[nodiscard]] double Distance(Vec2f first, Vec2f second) noexcept
    {
        const double dx = static_cast<double>(second.x - first.x);
        const double dy = static_cast<double>(second.y - first.y);
        return std::sqrt(dx * dx + dy * dy);
    }

    [[nodiscard]] bool IsBoundaryState(const SimplexState& state) noexcept
    {
        constexpr double tolerance = 1e-12;
        return state.X() <= tolerance
            || state.Y() <= tolerance
            || state.Z() <= tolerance;
    }

    [[nodiscard]] SimplexState InterpolateState(
        const SimplexState& first,
        const SimplexState& second,
        double amount
    )
    {
        return SimplexState::Normalized(
            first.X() + (second.X() - first.X()) * amount,
            first.Y() + (second.Y() - first.Y()) * amount,
            first.Z() + (second.Z() - first.Z()) * amount
        );
    }

    [[nodiscard]] std::vector<SimplexState> ResampleByVisualArcLength(
        const std::vector<SimplexState>& source,
        const SimplexMapper& mapper,
        double spacing
    )
    {
        if (source.size() < 2U || !std::isfinite(spacing) || spacing <= 0.0) {
            return source;
        }

        std::vector<double> cumulative(source.size(), 0.0);
        for (std::size_t index = 1; index < source.size(); ++index) {
            cumulative[index] = cumulative[index - 1U] + Distance(
                mapper.ToNdcPosition(source[index - 1U]),
                mapper.ToNdcPosition(source[index])
            );
        }

        const double totalLength = cumulative.back();
        if (totalLength <= spacing) {
            return { source.front(), source.back() };
        }

        std::vector<SimplexState> result;
        result.reserve(static_cast<std::size_t>(totalLength / spacing) + 2U);
        result.push_back(source.front());
        std::size_t segment = 1U;
        for (double target = spacing; target < totalLength; target += spacing) {
            while (segment < cumulative.size()
                   && cumulative[segment] < target) {
                ++segment;
            }
            if (segment >= cumulative.size()) {
                break;
            }

            const double segmentLength =
                cumulative[segment] - cumulative[segment - 1U];
            if (segmentLength <= std::numeric_limits<double>::epsilon()) {
                continue;
            }
            const double amount = std::clamp(
                (target - cumulative[segment - 1U]) / segmentLength,
                0.0,
                1.0
            );
            result.push_back(InterpolateState(
                source[segment - 1U],
                source[segment],
                amount
            ));
        }

        if (Distance(
                mapper.ToNdcPosition(result.back()),
                mapper.ToNdcPosition(source.back())
            ) > spacing * 0.1) {
            result.push_back(source.back());
        }
        return result;
    }

    struct OccupancyGrid final
    {
        explicit OccupancyGrid(int sideLength)
            : side(sideLength),
              owners(static_cast<std::size_t>(sideLength * sideLength), -1)
        {
        }

        [[nodiscard]] int Cell(Vec2f point) const noexcept
        {
            const int x = std::clamp(
                static_cast<int>((point.x + 1.0F) * 0.5F * side),
                0,
                side - 1
            );
            const int y = std::clamp(
                static_cast<int>((point.y + 1.0F) * 0.5F * side),
                0,
                side - 1
            );
            return y * side + x;
        }

        int side = 0;
        std::vector<int> owners;
    };
}

bool StreamlineFieldSettings::IsComputable() const noexcept
{
    if (density < 2
        || density > 64
        || !std::isfinite(integrationTime)
        || !std::isfinite(integrationTimeStep)
        || !std::isfinite(minimumVisualArcLength)
        || integrationTime <= 0.0
        || integrationTimeStep <= 0.0
        || maximumStepsPerPath <= 0
        || minimumVisualArcLength <= 0.0) {
        return false;
    }

    TrajectorySettings trajectory;
    trajectory.totalTime = integrationTime;
    trajectory.timeStep = integrationTimeStep;
    trajectory.maxSteps = maximumStepsPerPath;
    return trajectory.IsComputable();
}

std::optional<StreamlineFieldResult> StreamlineFieldGenerator::Generate(
    const SimplexDynamicModel& dynamics,
    const SimplexMapper& mapper,
    const StreamlineFieldSettings& settings
) const
{
    if (!settings.IsComputable()) {
        return std::nullopt;
    }

    std::vector<SimplexState> boundarySeeds;
    std::vector<SimplexState> interiorSeeds;
    const double density = static_cast<double>(settings.density);
    for (int xIndex = 0; xIndex <= settings.density; ++xIndex) {
        for (int yIndex = 0;
             xIndex + yIndex <= settings.density;
             ++yIndex) {
            const SimplexState seed = SimplexState::Normalized(
                static_cast<double>(xIndex) / density,
                static_cast<double>(yIndex) / density,
                static_cast<double>(settings.density - xIndex - yIndex)
                    / density
            );
            (IsBoundaryState(seed) ? boundarySeeds : interiorSeeds)
                .push_back(seed);
        }
    }

    std::vector<SimplexState> seeds;
    seeds.reserve(boundarySeeds.size() + interiorSeeds.size());
    seeds.insert(seeds.end(), boundarySeeds.begin(), boundarySeeds.end());
    seeds.insert(seeds.end(), interiorSeeds.begin(), interiorSeeds.end());

    TrajectorySettings trajectorySettings;
    trajectorySettings.totalTime = settings.integrationTime;
    trajectorySettings.timeStep = settings.integrationTimeStep;
    trajectorySettings.maxSteps = settings.maximumStepsPerPath;
    const SimplexTrajectoryIntegrator integrator;
    const double sampleSpacing = std::max(
        settings.minimumVisualArcLength,
        0.8 / static_cast<double>(settings.density * 3)
    );
    OccupancyGrid occupancy(std::max(24, settings.density * 5));

    StreamlineFieldResult result;
    result.candidateSeedCount = static_cast<int>(seeds.size());
    result.paths.reserve(seeds.size());

    int nextPathIdentifier = 0;
    for (const SimplexState& seed : seeds) {
        const auto integrated = integrator.Integrate(
            dynamics,
            seed,
            trajectorySettings
        );
        if (!integrated.has_value()) {
            return std::nullopt;
        }

        std::vector<SimplexState> sampled = ResampleByVisualArcLength(
            *integrated,
            mapper,
            sampleSpacing
        );
        if (sampled.size() < 2U) {
            continue;
        }

        const int seedCell = occupancy.Cell(mapper.ToNdcPosition(sampled.front()));
        if (occupancy.owners[static_cast<std::size_t>(seedCell)] >= 0) {
            continue;
        }

        std::vector<SimplexState> accepted;
        std::vector<int> touchedCells;
        accepted.reserve(sampled.size());
        touchedCells.reserve(sampled.size());
        for (const SimplexState& state : sampled) {
            const int cell = occupancy.Cell(mapper.ToNdcPosition(state));
            const int owner = occupancy.owners[static_cast<std::size_t>(cell)];
            if (owner >= 0) {
                break;
            }
            if (std::find(touchedCells.begin(), touchedCells.end(), cell)
                == touchedCells.end()) {
                touchedCells.push_back(cell);
            }
            accepted.push_back(state);
        }

        if (accepted.size() < 2U) {
            continue;
        }

        const Vec2f firstPosition = mapper.ToNdcPosition(accepted.front());
        const Vec2f lastPosition = mapper.ToNdcPosition(accepted.back());
        if (Distance(firstPosition, lastPosition)
            < settings.minimumVisualArcLength) {
            continue;
        }

        for (std::size_t index = 1; index < accepted.size(); ++index) {
            result.lineSegmentVertices.push_back(
                mapper.ToNdcPosition(accepted[index - 1U])
            );
            result.lineSegmentVertices.push_back(
                mapper.ToNdcPosition(accepted[index])
            );
        }

        bool arrowAdded = false;
        for (std::size_t index = accepted.size() - 1U; index > 0U; --index) {
            const Vec2f end = mapper.ToNdcPosition(accepted[index]);
            const Vec2f start = mapper.ToNdcPosition(accepted[index - 1U]);
            const double length = Distance(start, end);
            if (length < settings.minimumVisualArcLength) {
                continue;
            }

            result.arrows.push_back(StreamlineArrow{
                end,
                Vec2f{
                    static_cast<float>((end.x - start.x) / length),
                    static_cast<float>((end.y - start.y) / length)
                }
            });
            arrowAdded = true;
            break;
        }

        if (!arrowAdded) {
            const std::size_t segmentVertexCount =
                (accepted.size() - 1U) * 2U;
            result.lineSegmentVertices.resize(
                result.lineSegmentVertices.size() - segmentVertexCount
            );
            continue;
        }

        const int pathIdentifier = nextPathIdentifier++;
        for (const int cell : touchedCells) {
            occupancy.owners[static_cast<std::size_t>(cell)] = pathIdentifier;
        }

        result.paths.push_back(StreamlinePath{
            std::move(accepted),
            IsBoundaryState(seed)
        });
        ++result.acceptedSeedCount;
    }

    return result;
}
