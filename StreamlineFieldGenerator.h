#pragma once

#include <optional>
#include <vector>

#include "SimplexDynamicModel.h"
#include "SimplexMapper.h"
#include "SimplexState.h"
#include "Vec2f.h"

struct StreamlineFieldSettings final
{
    int density = 18;
    double integrationTime = 8.0;
    double integrationTimeStep = 0.02;
    int maximumStepsPerPath = 5000;
    double minimumVisualArcLength = 1e-4;

    [[nodiscard]] bool IsComputable() const noexcept;
};

struct StreamlinePath final
{
    std::vector<SimplexState> states;
    bool startsOnBoundary = false;
};

struct StreamlineArrow final
{
    Vec2f tip;
    Vec2f direction;
};

struct StreamlineFieldResult final
{
    int candidateSeedCount = 0;
    int acceptedSeedCount = 0;
    std::vector<StreamlinePath> paths;
    std::vector<Vec2f> lineSegmentVertices;
    std::vector<StreamlineArrow> arrows;
};

class StreamlineFieldGenerator final
{
public:
    [[nodiscard]] std::optional<StreamlineFieldResult> Generate(
        const SimplexDynamicModel& dynamics,
        const SimplexMapper& mapper,
        const StreamlineFieldSettings& settings
    ) const;
};
