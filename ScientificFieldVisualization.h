#pragma once

#include <optional>

#include "SimplexDynamicModel.h"
#include "SimplexMapper.h"
#include "SpeedHeatMapGenerator.h"
#include "SpeedHeatMapMesh.h"
#include "StreamlineFieldGenerator.h"
#include "StreamlineMesh.h"

class ScientificFieldVisualization final
{
public:
    [[nodiscard]] bool Create();

    [[nodiscard]] bool RebuildAll(
        const SimplexDynamicModel& dynamics,
        const SimplexMapper& mapper,
        int viewportWidth,
        int viewportHeight
    );
    [[nodiscard]] bool SetHeatMapSettings(
        const SpeedHeatMapSettings& settings,
        const SimplexDynamicModel& dynamics,
        const SimplexMapper& mapper
    );
    [[nodiscard]] bool SetStreamlineSettings(
        const StreamlineFieldSettings& settings,
        const SimplexDynamicModel& dynamics,
        const SimplexMapper& mapper,
        int viewportWidth,
        int viewportHeight
    );
    [[nodiscard]] bool SetArrowLengthPixels(
        float arrowLengthPixels,
        int viewportWidth,
        int viewportHeight
    );
    [[nodiscard]] bool RefreshViewport(
        int viewportWidth,
        int viewportHeight
    );

    void DrawHeatMap() const;
    void DrawStreamlines() const;

    [[nodiscard]] const SpeedHeatMapSettings& HeatMapSettings() const noexcept;
    [[nodiscard]] const StreamlineFieldSettings& StreamlineSettings() const noexcept;
    [[nodiscard]] const std::optional<SpeedHeatMapResult>& HeatMap() const noexcept;
    [[nodiscard]] const std::optional<StreamlineFieldResult>& Streamlines() const noexcept;
    [[nodiscard]] float ArrowLengthPixels() const noexcept;

private:
    SpeedHeatMapSettings heatMapSettings_;
    StreamlineFieldSettings streamlineSettings_;
    SpeedHeatMapGenerator heatMapGenerator_;
    StreamlineFieldGenerator streamlineGenerator_;
    SpeedHeatMapMesh heatMapMesh_;
    StreamlineMesh streamlineMesh_;
    std::optional<SpeedHeatMapResult> heatMap_;
    std::optional<StreamlineFieldResult> streamlines_;
    float arrowLengthPixels_ = 8.0F;
};
