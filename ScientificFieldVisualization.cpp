#include "ScientificFieldVisualization.h"

#include <cmath>
#include <utility>

bool ScientificFieldVisualization::Create()
{
    return heatMapMesh_.Create() && streamlineMesh_.Create();
}

bool ScientificFieldVisualization::RebuildAll(
    const SimplexDynamicModel& dynamics,
    const SimplexMapper& mapper,
    int viewportWidth,
    int viewportHeight
)
{
    const auto candidateHeatMap = heatMapGenerator_.Generate(
        dynamics,
        heatMapSettings_
    );
    const auto candidateStreamlines = streamlineGenerator_.Generate(
        dynamics,
        mapper,
        streamlineSettings_
    );
    if (!candidateHeatMap.has_value()
        || !candidateStreamlines.has_value()
        || !heatMapMesh_.SetData(*candidateHeatMap, mapper)
        || !streamlineMesh_.SetData(
            *candidateStreamlines,
            viewportWidth,
            viewportHeight,
            arrowLengthPixels_
        )) {
        return false;
    }

    heatMap_ = std::move(*candidateHeatMap);
    streamlines_ = std::move(*candidateStreamlines);
    return true;
}

bool ScientificFieldVisualization::SetHeatMapSettings(
    const SpeedHeatMapSettings& settings,
    const SimplexDynamicModel& dynamics,
    const SimplexMapper& mapper
)
{
    const auto candidate = heatMapGenerator_.Generate(dynamics, settings);
    if (!candidate.has_value() || !heatMapMesh_.SetData(*candidate, mapper)) {
        return false;
    }

    heatMapSettings_ = settings;
    heatMap_ = std::move(*candidate);
    return true;
}

bool ScientificFieldVisualization::SetStreamlineSettings(
    const StreamlineFieldSettings& settings,
    const SimplexDynamicModel& dynamics,
    const SimplexMapper& mapper,
    int viewportWidth,
    int viewportHeight
)
{
    const auto candidate = streamlineGenerator_.Generate(
        dynamics,
        mapper,
        settings
    );
    if (!candidate.has_value()
        || !streamlineMesh_.SetData(
            *candidate,
            viewportWidth,
            viewportHeight,
            arrowLengthPixels_
        )) {
        return false;
    }

    streamlineSettings_ = settings;
    streamlines_ = std::move(*candidate);
    return true;
}

bool ScientificFieldVisualization::SetArrowLengthPixels(
    float arrowLengthPixels,
    int viewportWidth,
    int viewportHeight
)
{
    if (!std::isfinite(arrowLengthPixels)
        || arrowLengthPixels <= 0.0F
        || !streamlines_.has_value()
        || !streamlineMesh_.SetData(
            *streamlines_,
            viewportWidth,
            viewportHeight,
            arrowLengthPixels
        )) {
        return false;
    }

    arrowLengthPixels_ = arrowLengthPixels;
    return true;
}

bool ScientificFieldVisualization::RefreshViewport(
    int viewportWidth,
    int viewportHeight
)
{
    return !streamlines_.has_value()
        || streamlineMesh_.SetData(
            *streamlines_,
            viewportWidth,
            viewportHeight,
            arrowLengthPixels_
        );
}

void ScientificFieldVisualization::DrawHeatMap() const
{
    heatMapMesh_.Draw();
}

void ScientificFieldVisualization::DrawStreamlines() const
{
    streamlineMesh_.Draw();
}

const SpeedHeatMapSettings&
ScientificFieldVisualization::HeatMapSettings() const noexcept
{
    return heatMapSettings_;
}

const StreamlineFieldSettings&
ScientificFieldVisualization::StreamlineSettings() const noexcept
{
    return streamlineSettings_;
}

const std::optional<SpeedHeatMapResult>&
ScientificFieldVisualization::HeatMap() const noexcept
{
    return heatMap_;
}

const std::optional<StreamlineFieldResult>&
ScientificFieldVisualization::Streamlines() const noexcept
{
    return streamlines_;
}

float ScientificFieldVisualization::ArrowLengthPixels() const noexcept
{
    return arrowLengthPixels_;
}
