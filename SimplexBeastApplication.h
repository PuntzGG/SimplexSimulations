#pragma once

#include <SDL3/SDL.h>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "EquilibriumInspector.h"
#include "EquilibriumMesh.h"
#include "EquilibriumPathMesh.h"
#include "ImGuiLayer.h"
#include "LogitEquilibriumSweep.h"
#include "PointMesh.h"
#include "RealTimeSimulationController.h"
#include "ScientificFieldVisualization.h"
#include "SdlOpenGlWindow.h"
#include "SdlSystem.h"
#include "ShaderProgram.h"
#include "SimplexDisplayLayout.h"
#include "SimplexMapper.h"
#include "SimplexMesh.h"
#include "SimulationSession.h"
#include "TrajectoryMesh.h"

class SimplexBeastApplication final
{
public:
    [[nodiscard]] int Run();

private:
    struct ViewportRectangle final
    {
        float left = 0.0F;
        float top = 0.0F;
        float width = 1.0F;
        float height = 1.0F;
    };

    [[nodiscard]] bool Initialize();
    void RunFrameLoop();
    void ProcessEvent(const SDL_Event& event);
    void ApplyPendingPointerState();
    void UpdateRealTimePlayback();
    void DrawControlPanel();
    void DrawInspectorWindow();
    void DrawCoordinateLabels();
    void RenderScene();

    [[nodiscard]] bool UpdateViewport();
    [[nodiscard]] ViewportRectangle ComputeViewport(
        int availableWidth,
        int availableHeight
    ) const noexcept;
    [[nodiscard]] Vec2f WindowToNdc(float windowX, float windowY) const;
    [[nodiscard]] Vec2f NdcToWindow(Vec2f ndc) const;

    [[nodiscard]] bool RefreshTrajectory();
    void RefreshStateMarkers();
    [[nodiscard]] bool RebuildEquilibria();
    [[nodiscard]] bool RebuildAllDerivedVisualizations();
    [[nodiscard]] bool GenerateEquilibriumSweep(
        const LogitEquilibriumSweepSettings& settings
    );
    void ClearEquilibriumSweep();
    void ClearEquilibriumSelection();
    [[nodiscard]] bool SelectEquilibriumAt(float windowX, float windowY);
    [[nodiscard]] bool RefreshEigenvectorOverlay();

    [[nodiscard]] bool SetActiveState(const SimplexState& state);
    [[nodiscard]] bool EnterRealTimeMode();
    [[nodiscard]] bool LeaveRealTimeMode();
    [[nodiscard]] bool ApplyDynamicsKind(DynamicsKind kind);
    [[nodiscard]] bool ApplyParameterDraft();
    [[nodiscard]] bool ApplyTrajectoryDraft();
    [[nodiscard]] bool ApplyHeatMapDraft();
    [[nodiscard]] bool ApplyStreamlineDraft();

    [[nodiscard]] const SimplexState& DisplayedState() const noexcept;
    [[nodiscard]] bool IsShiftDown() const noexcept;
    void ReportError(std::string message);

    static constexpr int kWindowWidth = 1280;
    static constexpr int kWindowHeight = 1000;

    SdlSystem sdlSystem_;
    SdlOpenGlWindow window_;
    ImGuiLayer imguiLayer_;

    ShaderProgram colorShader_;
    ShaderProgram outlineShader_;
    ShaderProgram pointShader_;
    ShaderProgram ringPointShader_;
    ShaderProgram heatMapShader_;

    SimplexMesh simplexMesh_;
    TrajectoryMesh trajectoryMesh_;
    EquilibriumMesh equilibriumMesh_;
    EquilibriumMesh equilibriumSweepSampleMesh_;
    EquilibriumPathMesh equilibriumSweepPathMesh_;
    EquilibriumPathMesh eigenvectorPathMesh_;
    PointMesh statePointMesh_;
    PointMesh targetPointMesh_;
    PointMesh selectedEquilibriumPointMesh_;
    ScientificFieldVisualization fieldVisualization_;

    SimplexMapper simplexMapper_{
        SimplexDisplayLayout::kCooperatorsVertex,
        SimplexDisplayLayout::kDefectorsVertex,
        SimplexDisplayLayout::kLonersVertex
    };
    SimulationSession simulation_;
    RealTimeSimulationController realTimeController_;
    EquilibriumInspector equilibriumInspector_;

    std::vector<SimplexEquilibrium> equilibria_;
    std::optional<LogitEquilibriumSweepResult> equilibriumSweep_;
    std::optional<SimplexState> pendingPointerState_;
    OpggParameters parameterDraft_;
    TrajectorySettings trajectoryDraft_;
    SpeedHeatMapSettings heatMapDraft_;
    StreamlineFieldSettings streamlineDraft_;

    ViewportRectangle logicalViewport_;
    int drawableViewportWidth_ = 1;
    int drawableViewportHeight_ = 1;
    int selectedEquilibriumSweep_ = 0;
    float arrowLengthDraft_ = 8.0F;
    bool running_ = false;
    bool draggingSimplexPoint_ = false;
    bool realTimeMode_ = false;
    bool showHeatMap_ = true;
    bool showStreamlines_ = true;
    bool showEquilibria_ = true;
    bool hasResponseTarget_ = false;
    bool hasSelectedEquilibrium_ = false;
    bool gpuFieldsCreated_ = false;
    std::string statusMessage_;
};
