#include "SimplexBeastApplication.h"

#include <GL/glew.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <iostream>
#include <iterator>
#include <limits>
#include <string_view>
#include <utility>

#include "ShaderSources.h"
#include "imgui.h"

namespace
{
    [[nodiscard]] std::vector<Vec2f> MapStates(
        const std::vector<SimplexState>& states,
        const SimplexMapper& mapper
    )
    {
        std::vector<Vec2f> positions;
        positions.reserve(states.size());
        for (const SimplexState& state : states) {
            positions.push_back(mapper.ToNdcPosition(state));
        }
        return positions;
    }

    [[nodiscard]] int DynamicsComboIndex(DynamicsKind kind) noexcept
    {
        switch (kind) {
        case DynamicsKind::Logit:
            return 0;
        case DynamicsKind::EqualSplitBestResponse:
            return 1;
        case DynamicsKind::Replicator:
            return 2;
        case DynamicsKind::Custom:
            return 0;
        }
        return 0;
    }

    [[nodiscard]] DynamicsKind DynamicsFromComboIndex(int index) noexcept
    {
        switch (index) {
        case 1:
            return DynamicsKind::EqualSplitBestResponse;
        case 2:
            return DynamicsKind::Replicator;
        default:
            return DynamicsKind::Logit;
        }
    }

    void DrawHeatMapLegend(const SpeedHeatMapResult& heatMap)
    {
        ImGui::Text(
            "Speed |ds/dt|: %.4g to %.4g",
            heatMap.displayedMinimumSpeed,
            heatMap.displayedMaximumSpeed
        );
        constexpr float values[] = { 0.0F, 0.2F, 0.4F, 0.6F, 0.8F, 1.0F };
        for (std::size_t index = 0; index < std::size(values); ++index) {
            const HeatMapColor color = SpeedHeatMapGenerator::Palette(
                values[index]
            );
            const std::string identifier = "##speedLegend"
                + std::to_string(index);
            ImGui::ColorButton(
                identifier.c_str(),
                ImVec4(color.red, color.green, color.blue, 1.0F),
                ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoBorder,
                ImVec2(28.0F, 10.0F)
            );
            if (index + 1U < std::size(values)) {
                ImGui::SameLine(0.0F, 0.0F);
            }
        }
        ImGui::TextUnformatted("blue: slow     green: intermediate     red: fast");
    }

    [[nodiscard]] bool SliderDouble(
        const char* label,
        double& value,
        double minimum,
        double maximum,
        const char* format = "%.4g",
        ImGuiSliderFlags flags = ImGuiSliderFlags_None
    )
    {
        return ImGui::SliderScalar(
            label,
            ImGuiDataType_Double,
            &value,
            &minimum,
            &maximum,
            format,
            flags
        );
    }

    void DrawComplexValue(const char* label, std::complex<double> value)
    {
        if (std::abs(value.imag()) <= 1e-12) {
            ImGui::Text("%s %.9g", label, value.real());
            return;
        }
        ImGui::Text(
            "%s %.9g %+.9gi",
            label,
            value.real(),
            value.imag()
        );
    }
}

int SimplexBeastApplication::Run()
{
    if (!Initialize()) {
        (void)SDL_ShowSimpleMessageBox(
            SDL_MESSAGEBOX_ERROR,
            "SimplexBeast startup error",
            "SimplexBeast could not initialize. Run the Debug build for detailed diagnostics and verify that SDL3 and GLEW DLLs are beside the executable.",
            window_.NativeWindow()
        );
        return 1;
    }

    RunFrameLoop();
    return 0;
}

bool SimplexBeastApplication::Initialize()
{
    if (!sdlSystem_.Initialize()
        || !window_.Create("SimplexBeast", kWindowWidth, kWindowHeight)) {
        return false;
    }

    glewExperimental = GL_TRUE;
    const GLenum glewError = glewInit();
    if (glewError != GLEW_OK) {
        std::cerr << "glewInit failed: "
                  << glewGetErrorString(glewError) << '\n';
        return false;
    }

    if (!imguiLayer_.Initialize(
            window_.NativeWindow(),
            window_.GlContext(),
            "#version 330 core"
        )
        || !colorShader_.Create(
            ShaderSources::kSimplexVertex,
            ShaderSources::kSimplexFragment
        )
        || !outlineShader_.Create(
            ShaderSources::kSimplexVertex,
            ShaderSources::kSimplexOutlineFragment
        )
        || !pointShader_.Create(
            ShaderSources::kPointVertex,
            ShaderSources::kPointFragment
        )
        || !ringPointShader_.Create(
            ShaderSources::kPointVertex,
            ShaderSources::kRingPointFragment
        )
        || !heatMapShader_.Create(
            ShaderSources::kHeatMapVertex,
            ShaderSources::kHeatMapFragment
        )) {
        return false;
    }

    if (!simplexMesh_.Create()
        || !trajectoryMesh_.Create()
        || !equilibriumMesh_.Create()
        || !equilibriumSweepSampleMesh_.Create()
        || !equilibriumSweepPathMesh_.Create()
        || !eigenvectorPathMesh_.Create()
        || !statePointMesh_.Create()
        || !targetPointMesh_.Create()
        || !selectedEquilibriumPointMesh_.Create()
        || !fieldVisualization_.Create()) {
        std::cerr << "Failed to create one or more OpenGL meshes.\n";
        return false;
    }
    gpuFieldsCreated_ = true;

    statePointMesh_.SetColor(1.0F, 0.30F, 0.02F);
    statePointMesh_.SetSize(14.0F);
    targetPointMesh_.SetColor(0.64F, 0.20F, 0.90F);
    targetPointMesh_.SetSize(21.0F);
    selectedEquilibriumPointMesh_.SetColor(1.0F, 0.82F, 0.08F);
    selectedEquilibriumPointMesh_.SetSize(23.0F);
    equilibriumMesh_.SetSize(15.0F);
    equilibriumSweepSampleMesh_.SetSize(5.0F);

    if (!simulation_.Initialize()) {
        std::cerr << "Failed to initialize the simulation session.\n";
        return false;
    }
    parameterDraft_ = simulation_.Parameters();
    trajectoryDraft_ = simulation_.Settings();
    heatMapDraft_ = fieldVisualization_.HeatMapSettings();
    streamlineDraft_ = fieldVisualization_.StreamlineSettings();
    arrowLengthDraft_ = fieldVisualization_.ArrowLengthPixels();

    if (!UpdateViewport()
        || !RefreshTrajectory()
        || !RebuildEquilibria()
        || !fieldVisualization_.RebuildAll(
            simulation_.ActiveDynamics(),
            simplexMapper_,
            drawableViewportWidth_,
            drawableViewportHeight_
        )) {
        return false;
    }

    running_ = true;
    std::cout << "SimplexBeast initialized successfully.\n";
    return true;
}

void SimplexBeastApplication::RunFrameLoop()
{
    while (running_) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ProcessEvent(event);
        }
        if (!running_) {
            break;
        }

        ApplyPendingPointerState();
        UpdateRealTimePlayback();

        imguiLayer_.BeginFrame();
        DrawCoordinateLabels();
        DrawControlPanel();
        DrawInspectorWindow();
        RenderScene();
        imguiLayer_.Render();
        window_.SwapBuffers();
    }
}

void SimplexBeastApplication::ProcessEvent(const SDL_Event& event)
{
    imguiLayer_.ProcessEvent(event);

    if (event.type == SDL_EVENT_QUIT) {
        running_ = false;
        return;
    }

    if (event.type == SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED) {
        if (!UpdateViewport()) {
            ReportError("Failed to update the OpenGL viewport.");
        }
    }

    if (event.type == SDL_EVENT_WINDOW_FOCUS_GAINED
        || event.type == SDL_EVENT_WINDOW_FOCUS_LOST
        || event.type == SDL_EVENT_WINDOW_MINIMIZED
        || event.type == SDL_EVENT_WINDOW_RESTORED) {
        realTimeController_.ResetClockBaseline();
    }

    if (event.type == SDL_EVENT_KEY_DOWN
        && !event.key.repeat
        && !imguiLayer_.WantsKeyboardInput()) {
        std::optional<SimplexState> shortcutState;
        switch (event.key.scancode) {
        case SDL_SCANCODE_1:
            shortcutState = SimplexState::Normalized(1.0, 0.0, 0.0);
            break;
        case SDL_SCANCODE_2:
            shortcutState = SimplexState::Normalized(0.0, 1.0, 0.0);
            break;
        case SDL_SCANCODE_3:
            shortcutState = SimplexState::Normalized(0.0, 0.0, 1.0);
            break;
        case SDL_SCANCODE_C:
            shortcutState = SimplexState::Normalized(1.0, 1.0, 1.0);
            break;
        case SDL_SCANCODE_SPACE:
            if (realTimeMode_) {
                if (realTimeController_.IsRunning()) {
                    realTimeController_.Pause();
                }
                else {
                    (void)realTimeController_.Start();
                }
            }
            break;
        default:
            break;
        }
        if (shortcutState.has_value() && !SetActiveState(*shortcutState)) {
            ReportError("Failed to apply the keyboard-selected state.");
        }
    }

    if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN
        && event.button.button == SDL_BUTTON_LEFT
        && !imguiLayer_.WantsMouseInput()) {
        if (IsShiftDown()) {
            draggingSimplexPoint_ = false;
            if (!SelectEquilibriumAt(event.button.x, event.button.y)) {
                ClearEquilibriumSelection();
            }
            return;
        }

        const auto state = simplexMapper_.FromNdcPosition(
            WindowToNdc(event.button.x, event.button.y)
        );
        if (state.has_value()) {
            pendingPointerState_ = *state;
            draggingSimplexPoint_ = true;
        }
    }

    if (event.type == SDL_EVENT_MOUSE_MOTION
        && draggingSimplexPoint_
        && !imguiLayer_.WantsMouseInput()) {
        const auto state = simplexMapper_.FromNdcPositionClamped(
            WindowToNdc(event.motion.x, event.motion.y)
        );
        if (state.has_value()) {
            pendingPointerState_ = *state;
        }
    }

    if (event.type == SDL_EVENT_MOUSE_BUTTON_UP
        && event.button.button == SDL_BUTTON_LEFT) {
        draggingSimplexPoint_ = false;
    }
}

void SimplexBeastApplication::ApplyPendingPointerState()
{
    if (!pendingPointerState_.has_value()) {
        return;
    }

    const SimplexState state = *pendingPointerState_;
    pendingPointerState_.reset();
    if (!SetActiveState(state)) {
        ReportError("Failed to update the dragged simplex state.");
    }
}

void SimplexBeastApplication::UpdateRealTimePlayback()
{
    if (!realTimeMode_) {
        return;
    }

    const RealTimeUpdateResult update = realTimeController_.Update(
        simulation_.ActiveDynamics(),
        draggingSimplexPoint_
    );
    if (update.stateChanged) {
        RefreshStateMarkers();
    }
    if (update.numericalError) {
        ReportError(std::string(realTimeController_.StatusText()));
    }
}

bool SimplexBeastApplication::UpdateViewport()
{
    int drawableWidth = 0;
    int drawableHeight = 0;
    if (!window_.GetDrawableSize(drawableWidth, drawableHeight)
        || drawableWidth <= 0
        || drawableHeight <= 0) {
        return false;
    }

    const ViewportRectangle drawableViewport = ComputeViewport(
        drawableWidth,
        drawableHeight
    );
    drawableViewportWidth_ = std::max(
        1,
        static_cast<int>(std::lround(drawableViewport.width))
    );
    drawableViewportHeight_ = std::max(
        1,
        static_cast<int>(std::lround(drawableViewport.height))
    );
    glViewport(
        static_cast<int>(std::lround(drawableViewport.left)),
        static_cast<int>(std::lround(
            static_cast<float>(drawableHeight)
                - drawableViewport.top
                - drawableViewport.height
        )),
        drawableViewportWidth_,
        drawableViewportHeight_
    );

    int logicalWidth = 0;
    int logicalHeight = 0;
    if (!window_.GetWindowSize(logicalWidth, logicalHeight)
        || logicalWidth <= 0
        || logicalHeight <= 0) {
        return false;
    }
    logicalViewport_ = ComputeViewport(logicalWidth, logicalHeight);
    return !gpuFieldsCreated_
        || fieldVisualization_.RefreshViewport(
            drawableViewportWidth_,
            drawableViewportHeight_
        );
}

SimplexBeastApplication::ViewportRectangle
SimplexBeastApplication::ComputeViewport(
    int availableWidth,
    int availableHeight
) const noexcept
{
    const float width = static_cast<float>(std::max(1, availableWidth));
    const float height = static_cast<float>(std::max(1, availableHeight));
    const float targetAspect = static_cast<float>(kWindowWidth)
        / static_cast<float>(kWindowHeight);
    const float availableAspect = width / height;
    if (availableAspect > targetAspect) {
        const float viewportWidth = height * targetAspect;
        return ViewportRectangle{
            (width - viewportWidth) * 0.5F,
            0.0F,
            viewportWidth,
            height
        };
    }

    const float viewportHeight = width / targetAspect;
    return ViewportRectangle{
        0.0F,
        (height - viewportHeight) * 0.5F,
        width,
        viewportHeight
    };
}

Vec2f SimplexBeastApplication::WindowToNdc(
    float windowX,
    float windowY
) const
{
    return Vec2f{
        2.0F * (windowX - logicalViewport_.left) / logicalViewport_.width
            - 1.0F,
        1.0F - 2.0F * (windowY - logicalViewport_.top)
            / logicalViewport_.height
    };
}

Vec2f SimplexBeastApplication::NdcToWindow(Vec2f ndc) const
{
    return Vec2f{
        logicalViewport_.left + (ndc.x + 1.0F) * 0.5F
            * logicalViewport_.width,
        logicalViewport_.top + (1.0F - ndc.y) * 0.5F
            * logicalViewport_.height
    };
}

bool SimplexBeastApplication::RefreshTrajectory()
{
    if (simulation_.Trajectory().empty()
        || !trajectoryMesh_.SetPoints(
            MapStates(simulation_.Trajectory(), simplexMapper_),
            0.96F,
            0.20F,
            0.10F
        )) {
        return false;
    }
    RefreshStateMarkers();
    return true;
}

void SimplexBeastApplication::RefreshStateMarkers()
{
    const SimplexState& state = DisplayedState();
    statePointMesh_.SetPosition(simplexMapper_.ToNdcPosition(state));
    const auto target = simulation_.ActiveDynamics().ResponseTarget(state);
    hasResponseTarget_ = target.has_value();
    if (target.has_value()) {
        targetPointMesh_.SetPosition(simplexMapper_.ToNdcPosition(*target));
    }
}

bool SimplexBeastApplication::RebuildEquilibria()
{
    const auto found = simulation_.FindEquilibria();
    if (!found.has_value()) {
        return false;
    }

    std::vector<Vec2f> positions;
    positions.reserve(found->size());
    for (const SimplexEquilibrium& equilibrium : *found) {
        positions.push_back(simplexMapper_.ToNdcPosition(equilibrium.state));
    }
    if (!equilibriumMesh_.SetPoints(
            positions,
            0.10F,
            0.88F,
            0.34F
        )) {
        return false;
    }

    equilibria_ = *found;
    ClearEquilibriumSelection();
    return true;
}

bool SimplexBeastApplication::RebuildAllDerivedVisualizations()
{
    ClearEquilibriumSweep();
    ClearEquilibriumSelection();
    if (!fieldVisualization_.RebuildAll(
            simulation_.ActiveDynamics(),
            simplexMapper_,
            drawableViewportWidth_,
            drawableViewportHeight_
        )
        || !RebuildEquilibria()) {
        return false;
    }
    RefreshStateMarkers();
    return true;
}

bool SimplexBeastApplication::GenerateEquilibriumSweep(
    const LogitEquilibriumSweepSettings& settings
)
{
    const auto generated = simulation_.GenerateEquilibriumSweep(settings);
    if (!generated.has_value()) {
        return false;
    }

    std::vector<std::vector<Vec2f>> paths;
    std::vector<Vec2f> samples;
    paths.reserve(generated->branches.size());
    for (const LogitEquilibriumBranch& branch : generated->branches) {
        std::vector<Vec2f> path;
        path.reserve(branch.samples.size());
        for (const LogitEquilibriumSweepSample& sample : branch.samples) {
            const Vec2f position = simplexMapper_.ToNdcPosition(
                sample.equilibrium.state
            );
            path.push_back(position);
            samples.push_back(position);
        }
        paths.push_back(std::move(path));
    }

    const bool punishmentSweep = settings.parameter
        == LogitEquilibriumSweepParameter::PunishmentFraction;
    const float red = punishmentSweep ? 0.75F : 0.10F;
    const float green = punishmentSweep ? 0.25F : 0.75F;
    const float blue = 0.95F;
    if (!equilibriumSweepPathMesh_.SetPaths(paths, red, green, blue)
        || !equilibriumSweepSampleMesh_.SetPoints(
            samples,
            red,
            green,
            blue
        )) {
        return false;
    }

    equilibriumSweep_ = std::move(*generated);
    return true;
}

void SimplexBeastApplication::ClearEquilibriumSweep()
{
    equilibriumSweep_.reset();
    selectedEquilibriumSweep_ = 0;
    (void)equilibriumSweepPathMesh_.SetPaths({}, 0.0F, 0.0F, 0.0F);
    (void)equilibriumSweepSampleMesh_.SetPoints({}, 0.0F, 0.0F, 0.0F);
}

void SimplexBeastApplication::ClearEquilibriumSelection()
{
    equilibriumInspector_.Clear();
    hasSelectedEquilibrium_ = false;
    (void)eigenvectorPathMesh_.SetPaths({}, 0.0F, 0.0F, 0.0F);
}

bool SimplexBeastApplication::SelectEquilibriumAt(
    float windowX,
    float windowY
)
{
    if (!showEquilibria_ || equilibria_.empty()) {
        return false;
    }

    constexpr float hitRadiusPixels = 18.0F;
    const float hitRadiusSquared = hitRadiusPixels * hitRadiusPixels;
    std::optional<std::size_t> nearest;
    float nearestDistanceSquared = hitRadiusSquared;
    for (std::size_t index = 0; index < equilibria_.size(); ++index) {
        const Vec2f marker = NdcToWindow(
            simplexMapper_.ToNdcPosition(equilibria_[index].state)
        );
        const float dx = marker.x - windowX;
        const float dy = marker.y - windowY;
        const float distanceSquared = dx * dx + dy * dy;
        if (distanceSquared <= nearestDistanceSquared) {
            nearestDistanceSquared = distanceSquared;
            nearest = index;
        }
    }

    if (!nearest.has_value()
        || !equilibriumInspector_.Select(
            *nearest,
            equilibria_,
            simulation_.ActiveDynamics()
        )) {
        return false;
    }

    selectedEquilibriumPointMesh_.SetPosition(
        simplexMapper_.ToNdcPosition(equilibria_[*nearest].state)
    );
    hasSelectedEquilibrium_ = true;
    return RefreshEigenvectorOverlay();
}

bool SimplexBeastApplication::RefreshEigenvectorOverlay()
{
    std::vector<std::vector<Vec2f>> paths;
    if (!equilibriumInspector_.HasSelection()
        || !equilibriumInspector_.Analysis().jacobian.has_value()) {
        return eigenvectorPathMesh_.SetPaths(paths, 1.0F, 0.78F, 0.05F);
    }

    const SimplexState& equilibrium =
        equilibriumInspector_.Equilibrium().state;
    const Vec2f center = simplexMapper_.ToNdcPosition(equilibrium);
    const Vec2f xVertex = simplexMapper_.ToNdcPosition(
        SimplexState::Normalized(1.0, 0.0, 0.0)
    );
    const Vec2f yVertex = simplexMapper_.ToNdcPosition(
        SimplexState::Normalized(0.0, 1.0, 0.0)
    );
    const Vec2f zVertex = simplexMapper_.ToNdcPosition(
        SimplexState::Normalized(0.0, 0.0, 1.0)
    );

    constexpr double imaginaryTolerance = 1e-9;
    constexpr float halfLengthPixels = 34.0F;
    for (const SimplexEigenpair& eigenpair
         : equilibriumInspector_.Analysis().eigenpairs) {
        if (!eigenpair.IsReal(imaginaryTolerance)) {
            continue;
        }

        const float vx = static_cast<float>(
            eigenpair.simplexEigenvector[0].real()
        );
        const float vy = static_cast<float>(
            eigenpair.simplexEigenvector[1].real()
        );
        const float vz = static_cast<float>(
            eigenpair.simplexEigenvector[2].real()
        );
        float directionX = vx * xVertex.x + vy * yVertex.x + vz * zVertex.x;
        float directionY = vx * xVertex.y + vy * yVertex.y + vz * zVertex.y;
        float pixelX = directionX * drawableViewportWidth_ * 0.5F;
        float pixelY = directionY * drawableViewportHeight_ * 0.5F;
        const float pixelLength = std::sqrt(pixelX * pixelX + pixelY * pixelY);
        if (!std::isfinite(pixelLength) || pixelLength <= 1e-6F) {
            continue;
        }
        pixelX /= pixelLength;
        pixelY /= pixelLength;
        const float ndcX = pixelX * halfLengthPixels * 2.0F
            / drawableViewportWidth_;
        const float ndcY = pixelY * halfLengthPixels * 2.0F
            / drawableViewportHeight_;
        paths.push_back({
            Vec2f{ center.x - ndcX, center.y - ndcY },
            Vec2f{ center.x + ndcX, center.y + ndcY }
        });
    }

    return eigenvectorPathMesh_.SetPaths(paths, 1.0F, 0.78F, 0.05F);
}

bool SimplexBeastApplication::SetActiveState(const SimplexState& state)
{
    if (!state.IsValid()) {
        return false;
    }

    if (realTimeMode_) {
        if (!realTimeController_.Reseed(state)) {
            return false;
        }
        RefreshStateMarkers();
        return true;
    }

    if (!simulation_.SetCurrentState(state) || !RefreshTrajectory()) {
        return false;
    }
    return true;
}

bool SimplexBeastApplication::EnterRealTimeMode()
{
    if (realTimeMode_) {
        return true;
    }
    if (!realTimeController_.Enter(simulation_.CurrentState())) {
        return false;
    }

    realTimeMode_ = true;
    RefreshStateMarkers();
    statusMessage_ = "Real Time mode entered paused.";
    return true;
}

bool SimplexBeastApplication::LeaveRealTimeMode()
{
    if (!realTimeMode_) {
        return true;
    }

    const SimplexState finalLiveState = realTimeController_.LiveState();
    realTimeController_.Leave();
    realTimeMode_ = false;
    if (!simulation_.SetCurrentState(finalLiveState)
        || !RefreshTrajectory()) {
        return false;
    }

    statusMessage_ = "Live state adopted as the static starting state.";
    return true;
}

bool SimplexBeastApplication::ApplyDynamicsKind(DynamicsKind kind)
{
    if (kind == simulation_.ActiveDynamicsKind()) {
        return true;
    }
    if (!simulation_.SetDynamicsKind(kind)) {
        return false;
    }

    if (realTimeMode_) {
        realTimeController_.NotifyDynamicsChanged();
    }
    if (!RefreshTrajectory() || !RebuildAllDerivedVisualizations()) {
        return false;
    }

    statusMessage_ = std::string("Dynamics changed to ")
        + std::string(DynamicsName(kind))
        + (realTimeMode_ ? "; Real Time playback paused." : ".");
    return true;
}

bool SimplexBeastApplication::ApplyParameterDraft()
{
    if (!simulation_.SetParameters(parameterDraft_)) {
        parameterDraft_ = simulation_.Parameters();
        return false;
    }

    parameterDraft_ = simulation_.Parameters();
    if (realTimeMode_) {
        realTimeController_.NotifyDynamicsChanged();
    }
    if (!RefreshTrajectory() || !RebuildAllDerivedVisualizations()) {
        return false;
    }

    statusMessage_ = realTimeMode_
        ? "Parameters applied; Real Time playback paused."
        : "Parameters applied.";
    return true;
}

bool SimplexBeastApplication::ApplyTrajectoryDraft()
{
    if (!simulation_.SetTrajectorySettings(trajectoryDraft_)) {
        trajectoryDraft_ = simulation_.Settings();
        return false;
    }
    trajectoryDraft_ = simulation_.Settings();
    if (!RefreshTrajectory()) {
        return false;
    }
    statusMessage_ = "Static trajectory settings applied.";
    return true;
}

bool SimplexBeastApplication::ApplyHeatMapDraft()
{
    if (!fieldVisualization_.SetHeatMapSettings(
            heatMapDraft_,
            simulation_.ActiveDynamics(),
            simplexMapper_
        )) {
        heatMapDraft_ = fieldVisualization_.HeatMapSettings();
        return false;
    }
    heatMapDraft_ = fieldVisualization_.HeatMapSettings();
    statusMessage_ = "Speed heat map regenerated.";
    return true;
}

bool SimplexBeastApplication::ApplyStreamlineDraft()
{
    if (!fieldVisualization_.SetStreamlineSettings(
            streamlineDraft_,
            simulation_.ActiveDynamics(),
            simplexMapper_,
            drawableViewportWidth_,
            drawableViewportHeight_
        )) {
        streamlineDraft_ = fieldVisualization_.StreamlineSettings();
        return false;
    }
    streamlineDraft_ = fieldVisualization_.StreamlineSettings();
    statusMessage_ = "Streamline field regenerated.";
    return true;
}

const SimplexState& SimplexBeastApplication::DisplayedState() const noexcept
{
    return realTimeMode_
        ? realTimeController_.LiveState()
        : simulation_.CurrentState();
}

bool SimplexBeastApplication::IsShiftDown() const noexcept
{
    const SDL_Keymod modifiers = SDL_GetModState();
    return (modifiers & SDL_KMOD_SHIFT) != 0;
}

void SimplexBeastApplication::ReportError(std::string message)
{
    statusMessage_ = std::move(message);
    std::cerr << "SimplexBeast: " << statusMessage_ << '\n';
}

void SimplexBeastApplication::DrawControlPanel()
{
    ImGui::SetNextWindowPos(ImVec2(12.0F, 12.0F), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(390.0F, 930.0F), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(
        ImVec2(330.0F, 380.0F),
        ImVec2(500.0F, 980.0F)
    );
    if (!ImGui::Begin("SimplexBeast - Scientific Controls")) {
        ImGui::End();
        return;
    }
    ImGui::PushItemWidth(185.0F);

    const SimplexState& state = DisplayedState();
    ImGui::Text("Dynamics: %s", DynamicsName(simulation_.ActiveDynamicsKind()).data());
    ImGui::Text(
        "State  x %.6f   y %.6f   z %.6f",
        state.X(),
        state.Y(),
        state.Z()
    );

    int dynamicsIndex = DynamicsComboIndex(simulation_.ActiveDynamicsKind());
    constexpr const char* dynamicsItems =
        "Logit\0Equal-split Best Response\0Replicator\0";
    if (ImGui::Combo("Dynamic", &dynamicsIndex, dynamicsItems)
        && !ApplyDynamicsKind(DynamicsFromComboIndex(dynamicsIndex))) {
        ReportError("The requested dynamics could not be applied.");
    }
    if (simulation_.ActiveDynamicsKind()
        == DynamicsKind::EqualSplitBestResponse) {
        ImGui::TextDisabled(
            "Ties split equally (abs %.1e, rel %.1e).",
            simulation_.BestResponseOptions().absoluteTieTolerance,
            simulation_.BestResponseOptions().relativeTieTolerance
        );
    }
    else if (simulation_.ActiveDynamicsKind() == DynamicsKind::Replicator) {
        ImGui::TextDisabled(
            "Replicator has no separate response-target point."
        );
    }

    int modeIndex = realTimeMode_ ? 1 : 0;
    constexpr const char* modeItems = "Static trajectory\0Real Time\0";
    if (ImGui::Combo("Mode", &modeIndex, modeItems)) {
        const bool applied = modeIndex == 1
            ? EnterRealTimeMode()
            : LeaveRealTimeMode();
        if (!applied) {
            ReportError("The requested playback mode could not be entered.");
        }
    }

    if (realTimeMode_) {
        if (realTimeController_.IsRunning()) {
            if (ImGui::Button("Pause")) {
                realTimeController_.Pause();
            }
        }
        else if (ImGui::Button("Start")) {
            if (!realTimeController_.Start()) {
                ReportError("Real Time playback could not start.");
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset")) {
            realTimeController_.Reset();
            RefreshStateMarkers();
        }
        ImGui::SameLine();
        ImGui::Text("%s", realTimeController_.StatusText().data());

        double playbackSpeed = realTimeController_.Settings().playbackSpeed;
        if (SliderDouble(
                "Playback speed",
                playbackSpeed,
                0.1,
                8.0,
                "%.2fx",
                ImGuiSliderFlags_Logarithmic
            )
            && !realTimeController_.SetPlaybackSpeed(playbackSpeed)) {
            ReportError("Invalid Real Time playback speed.");
        }
        ImGui::Text(
            "Simulated time: %.3f   fixed dt: %.4g",
            realTimeController_.ElapsedSimulationTime(),
            realTimeController_.Settings().fixedIntegrationStep
        );
        ImGui::TextDisabled(
            "Click/drag to reseed. Space toggles Start/Pause."
        );
    }
    else {
        ImGui::TextDisabled("Click/drag to choose the trajectory start.");
    }
    ImGui::TextDisabled("Shift+click a green equilibrium to inspect it.");

    if (ImGui::CollapsingHeader(
            "OPGG parameters",
            ImGuiTreeNodeFlags_DefaultOpen
        )) {
        bool commit = false;
        (void)ImGui::SliderInt("Group size (n)", &parameterDraft_.groupSize, 2, 20);
        commit = commit || ImGui::IsItemDeactivatedAfterEdit();

        constexpr double interiorGap = 1e-3;
        const double multiplicationMinimum = 1.0 + interiorGap;
        const double multiplicationMaximum =
            static_cast<double>(parameterDraft_.groupSize) - interiorGap;
        parameterDraft_.multiplicationFactor = std::clamp(
            parameterDraft_.multiplicationFactor,
            multiplicationMinimum,
            multiplicationMaximum
        );
        (void)SliderDouble(
            "Multiplication (r)",
            parameterDraft_.multiplicationFactor,
            multiplicationMinimum,
            multiplicationMaximum,
            "%.4f"
        );
        commit = commit || ImGui::IsItemDeactivatedAfterEdit();

        const double lonerMinimum = 1e-4;
        const double lonerMaximum = std::max(
            lonerMinimum * 2.0,
            parameterDraft_.multiplicationFactor - 1.0 - lonerMinimum
        );
        parameterDraft_.lonerPayoffMultiplier = std::clamp(
            parameterDraft_.lonerPayoffMultiplier,
            lonerMinimum,
            lonerMaximum
        );
        (void)SliderDouble(
            "Loner multiplier (sigma)",
            parameterDraft_.lonerPayoffMultiplier,
            lonerMinimum,
            lonerMaximum,
            "%.4f"
        );
        commit = commit || ImGui::IsItemDeactivatedAfterEdit();

        (void)SliderDouble(
            "Contribution cost (c)",
            parameterDraft_.contributionCost,
            0.01,
            5.0,
            "%.4f"
        );
        commit = commit || ImGui::IsItemDeactivatedAfterEdit();
        (void)SliderDouble(
            "Punishment fraction (v)",
            parameterDraft_.punishmentFraction,
            0.0,
            1.0,
            "%.4f"
        );
        commit = commit || ImGui::IsItemDeactivatedAfterEdit();

        if (simulation_.ActiveDynamicsKind() == DynamicsKind::Logit) {
            (void)SliderDouble(
                "Logit noise (eta)",
                parameterDraft_.logitNoise,
                0.001,
                1.0,
                "%.4f",
                ImGuiSliderFlags_Logarithmic
            );
            commit = commit || ImGui::IsItemDeactivatedAfterEdit();
        }
        else {
            ImGui::TextDisabled("eta is used only by Logit dynamics.");
        }

        if (commit && !ApplyParameterDraft()) {
            ReportError("Invalid OPGG parameters; the last valid values were restored.");
        }
    }

    if (ImGui::CollapsingHeader("Selected trajectory")) {
        bool commit = false;
        (void)SliderDouble(
            "Trajectory time",
            trajectoryDraft_.totalTime,
            0.5,
            30.0,
            "%.2f"
        );
        commit = commit || ImGui::IsItemDeactivatedAfterEdit();
        (void)SliderDouble(
            "Trajectory dt",
            trajectoryDraft_.timeStep,
            0.005,
            0.1,
            "%.4f",
            ImGuiSliderFlags_Logarithmic
        );
        commit = commit || ImGui::IsItemDeactivatedAfterEdit();
        if (commit && !ApplyTrajectoryDraft()) {
            ReportError("Invalid trajectory settings; previous values restored.");
        }
        if (realTimeMode_) {
            ImGui::TextDisabled(
                "The projected path is cached but hidden during Real Time playback."
            );
        }
    }

    if (ImGui::CollapsingHeader(
            "Speed heat map",
            ImGuiTreeNodeFlags_DefaultOpen
        )) {
        (void)ImGui::Checkbox("Show speed heat map", &showHeatMap_);
        bool applyHeatMap = false;
        (void)ImGui::SliderInt(
            "Resolution",
            &heatMapDraft_.resolution,
            4,
            128
        );
        applyHeatMap = applyHeatMap || ImGui::IsItemDeactivatedAfterEdit();

        int normalization = heatMapDraft_.normalization
                == HeatMapNormalizationMode::Relative
            ? 0
            : 1;
        if (ImGui::RadioButton("Relative", &normalization, 0)) {
            heatMapDraft_.normalization = HeatMapNormalizationMode::Relative;
            applyHeatMap = true;
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Locked range", &normalization, 1)) {
            heatMapDraft_.normalization = HeatMapNormalizationMode::LockedRange;
            applyHeatMap = true;
        }

        if (heatMapDraft_.normalization
            == HeatMapNormalizationMode::LockedRange) {
            (void)ImGui::InputDouble(
                "Minimum speed",
                &heatMapDraft_.lockedMinimumSpeed,
                0.0,
                0.0,
                "%.8g"
            );
            applyHeatMap = applyHeatMap || ImGui::IsItemDeactivatedAfterEdit();
            (void)ImGui::InputDouble(
                "Maximum speed",
                &heatMapDraft_.lockedMaximumSpeed,
                0.0,
                0.0,
                "%.8g"
            );
            applyHeatMap = applyHeatMap || ImGui::IsItemDeactivatedAfterEdit();
        }
        if (applyHeatMap && !ApplyHeatMapDraft()) {
            ReportError("Invalid heat-map settings; previous values restored.");
        }

        if (fieldVisualization_.HeatMap().has_value()) {
            DrawHeatMapLegend(*fieldVisualization_.HeatMap());
            if (simulation_.ActiveDynamicsKind()
                == DynamicsKind::EqualSplitBestResponse) {
                const std::size_t mixedCells = static_cast<std::size_t>(
                    std::count_if(
                        fieldVisualization_.HeatMap()->mixedRegionTriangles.begin(),
                        fieldVisualization_.HeatMap()->mixedRegionTriangles.end(),
                        [](std::uint8_t value) { return value != 0U; }
                    )
                );
                ImGui::TextDisabled(
                    "%zu switching-boundary cells are flat shaded.",
                    mixedCells
                );
            }
        }
    }

    if (ImGui::CollapsingHeader(
            "Streamlines and endpoint arrows",
            ImGuiTreeNodeFlags_DefaultOpen
        )) {
        (void)ImGui::Checkbox("Show streamline field", &showStreamlines_);
        bool applyStreamlines = false;
        (void)ImGui::SliderInt("Density", &streamlineDraft_.density, 3, 40);
        applyStreamlines = applyStreamlines
            || ImGui::IsItemDeactivatedAfterEdit();
        (void)SliderDouble(
            "Field trajectory time",
            streamlineDraft_.integrationTime,
            0.5,
            20.0,
            "%.2f"
        );
        applyStreamlines = applyStreamlines
            || ImGui::IsItemDeactivatedAfterEdit();
        (void)SliderDouble(
            "Field dt",
            streamlineDraft_.integrationTimeStep,
            0.005,
            0.1,
            "%.4f",
            ImGuiSliderFlags_Logarithmic
        );
        applyStreamlines = applyStreamlines
            || ImGui::IsItemDeactivatedAfterEdit();
        if (applyStreamlines && !ApplyStreamlineDraft()) {
            ReportError("Invalid streamline settings; previous values restored.");
        }

        if (ImGui::SliderFloat(
                "Arrow length (pixels)",
                &arrowLengthDraft_,
                4.0F,
                18.0F,
                "%.1f"
            )
            && !fieldVisualization_.SetArrowLengthPixels(
                arrowLengthDraft_,
                drawableViewportWidth_,
                drawableViewportHeight_
            )) {
            arrowLengthDraft_ = fieldVisualization_.ArrowLengthPixels();
            ReportError("The requested arrow size could not be applied.");
        }
        if (fieldVisualization_.Streamlines().has_value()) {
            const StreamlineFieldResult& result =
                *fieldVisualization_.Streamlines();
            ImGui::Text(
                "%d accepted of %d deterministic seeds; %zu arrows",
                result.acceptedSeedCount,
                result.candidateSeedCount,
                result.arrows.size()
            );
        }
    }

    if (ImGui::CollapsingHeader(
            "Equilibria",
            ImGuiTreeNodeFlags_DefaultOpen
        )) {
        if (ImGui::Checkbox("Show equilibria", &showEquilibria_)) {
            if (showEquilibria_ && !RebuildEquilibria()) {
                showEquilibria_ = false;
                ReportError("Equilibria could not be regenerated.");
            }
            if (!showEquilibria_) {
                ClearEquilibriumSelection();
            }
        }
        double maximumResidual = 0.0;
        for (const SimplexEquilibrium& equilibrium : equilibria_) {
            maximumResidual = std::max(maximumResidual, equilibrium.residual);
        }
        ImGui::Text(
            "%zu verified rest points; max residual %.3g",
            equilibria_.size(),
            maximumResidual
        );

        if (simulation_.ActiveDynamicsKind() == DynamicsKind::Logit) {
            int requestedSweep = selectedEquilibriumSweep_;
            bool sweepModeChanged = false;
            sweepModeChanged = ImGui::RadioButton(
                "No sweep",
                &requestedSweep,
                0
            ) || sweepModeChanged;
            ImGui::SameLine();
            sweepModeChanged = ImGui::RadioButton(
                "eta sweep",
                &requestedSweep,
                1
            ) || sweepModeChanged;
            ImGui::SameLine();
            sweepModeChanged = ImGui::RadioButton(
                "v sweep",
                &requestedSweep,
                2
            ) || sweepModeChanged;
            if (sweepModeChanged) {
                ClearEquilibriumSweep();
                selectedEquilibriumSweep_ = requestedSweep;
            }
            if (selectedEquilibriumSweep_ != 0
                && ImGui::Button("Generate selected sweep")) {
                LogitEquilibriumSweepSettings settings;
                settings.parameter = selectedEquilibriumSweep_ == 1
                    ? LogitEquilibriumSweepParameter::LogitNoise
                    : LogitEquilibriumSweepParameter::PunishmentFraction;
                settings.minimumParameter = selectedEquilibriumSweep_ == 1
                    ? 0.001
                    : 0.0;
                settings.maximumParameter = 1.0;
                if (!GenerateEquilibriumSweep(settings)) {
                    ReportError("The Logit equilibrium sweep failed.");
                }
                else {
                    statusMessage_ = "Logit equilibrium sweep generated.";
                }
            }
            if (equilibriumSweep_.has_value()) {
                std::size_t sampleCount = 0U;
                for (const LogitEquilibriumBranch& branch
                     : equilibriumSweep_->branches) {
                    sampleCount += branch.samples.size();
                }
                ImGui::Text(
                    "%zu branches, %zu verified sweep samples",
                    equilibriumSweep_->branches.size(),
                    sampleCount
                );
            }
        }
        else {
            ImGui::TextDisabled("Parameter sweeps are Logit-only.");
        }
    }

    ImGui::Separator();
    if (!statusMessage_.empty()) {
        ImGui::TextWrapped("Last action: %s", statusMessage_.c_str());
    }
    ImGui::TextDisabled("Shortcuts: 1/2/3 corners, C center, Space playback.");
    ImGui::PopItemWidth();
    ImGui::End();
}

void SimplexBeastApplication::DrawInspectorWindow()
{
    if (!equilibriumInspector_.HasSelection()) {
        return;
    }

    bool open = true;
    ImGui::SetNextWindowPos(ImVec2(885.0F, 18.0F), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(380.0F, 510.0F), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Equilibrium Inspector", &open)) {
        ImGui::End();
        if (!open) {
            ClearEquilibriumSelection();
        }
        return;
    }

    const SimplexEquilibrium& equilibrium = equilibriumInspector_.Equilibrium();
    const SimplexJacobianAnalysis& analysis = equilibriumInspector_.Analysis();
    ImGui::Text(
        "%s equilibrium %zu of %zu",
        DynamicsName(equilibriumInspector_.SelectedDynamicsKind()).data(),
        equilibriumInspector_.SelectedIndex() + 1U,
        equilibria_.size()
    );
    ImGui::Text(
        "%s; %s",
        SimplexEquilibriumLocationName(equilibrium.location),
        equilibrium.isIsolated ? "isolated candidate" : "non-isolated representative"
    );
    ImGui::SeparatorText("Coordinates");
    ImGui::Text("x (cooperators) = %.12g", equilibrium.state.X());
    ImGui::Text("y (defectors)   = %.12g", equilibrium.state.Y());
    ImGui::Text("z (loners)      = %.12g", equilibrium.state.Z());
    ImGui::Text("verified residual ||f|| = %.6g", equilibrium.residual);

    ImGui::SeparatorText("Payoffs at this state");
    if (equilibriumInspector_.Payoffs().has_value()) {
        const StrategyPayoffs& payoffs = *equilibriumInspector_.Payoffs();
        ImGui::Text("pi_x (cooperators) = %.9g", payoffs.cooperators);
        ImGui::Text("pi_y (defectors)   = %.9g", payoffs.defectors);
        ImGui::Text("pi_z (loners)      = %.9g", payoffs.loners);
    }
    else {
        ImGui::TextDisabled("Payoffs are unavailable for this dynamic.");
    }

    ImGui::SeparatorText("Reduced Jacobian and stability");
    ImGui::TextWrapped("Status: %s", JacobianAnalysisStatusName(analysis.status));
    ImGui::TextWrapped("%s", analysis.message.c_str());
    ImGui::Text(
        "Basis: (x, y), with z = 1 - x - y%s",
        analysis.isBoundaryConstrained ? " [feasible one-sided stencil]" : ""
    );
    if (analysis.jacobian.has_value()) {
        const ReducedJacobian& jacobian = *analysis.jacobian;
        ImGui::Text("J = [ %.9g   %.9g ]", jacobian.dxByDx, jacobian.dxByDy);
        ImGui::Text("    [ %.9g   %.9g ]", jacobian.dyByDx, jacobian.dyByDy);
        ImGui::Text(
            "Classification: %s",
            StabilityClassificationName(analysis.classification)
        );
        ImGui::Text(
            "finite-difference relative check: %.3g",
            analysis.finiteDifferenceRelativeError
        );
        if (analysis.isRepeatedEigenvalue) {
            ImGui::TextDisabled("Repeated eigenvalue detected.");
        }
        if (analysis.isDefective) {
            ImGui::TextDisabled(
                "The repeated eigenvalue is defective; a full eigenbasis is unavailable."
            );
        }

        for (std::size_t index = 0; index < analysis.eigenpairs.size(); ++index) {
            const SimplexEigenpair& pair = analysis.eigenpairs[index];
            ImGui::PushID(static_cast<int>(index));
            ImGui::Separator();
            const std::string eigenvalueLabel = "lambda_"
                + std::to_string(index + 1U)
                + " =";
            DrawComplexValue(eigenvalueLabel.c_str(), pair.eigenvalue);
            if (pair.hasEigenvector) {
                ImGui::Text(
                    "reduced v_%zu = (%.7g %+.7gi, %.7g %+.7gi)",
                    index + 1U,
                    pair.reducedEigenvector[0].real(),
                    pair.reducedEigenvector[0].imag(),
                    pair.reducedEigenvector[1].real(),
                    pair.reducedEigenvector[1].imag()
                );
                ImGui::TextWrapped(
                    "simplex tangent v_%zu = (%.7g %+.7gi, %.7g %+.7gi, %.7g %+.7gi)",
                    index + 1U,
                    pair.simplexEigenvector[0].real(),
                    pair.simplexEigenvector[0].imag(),
                    pair.simplexEigenvector[1].real(),
                    pair.simplexEigenvector[1].imag(),
                    pair.simplexEigenvector[2].real(),
                    pair.simplexEigenvector[2].imag()
                );
            }
            else {
                ImGui::TextDisabled("No stable eigenvector basis was resolved.");
            }
            ImGui::PopID();
        }
    }
    else if (analysis.status
        == JacobianAnalysisStatus::NonsmoothSwitchingSurface) {
        ImGui::TextDisabled(
            "Equal-split Best Response is nonsmooth at payoff ties, so a classical Jacobian and ordinary eigenpairs are not declared here."
        );
    }

    ImGui::End();
    if (!open) {
        ClearEquilibriumSelection();
    }
}

void SimplexBeastApplication::DrawCoordinateLabels()
{
    ImDrawList* drawList = ImGui::GetBackgroundDrawList();
    if (drawList == nullptr) {
        return;
    }

    const ImU32 color = IM_COL32(24, 24, 28, 255);
    const Vec2f xPosition = NdcToWindow(simplexMapper_.ToNdcPosition(
        SimplexState::Normalized(1.0, 0.0, 0.0)
    ));
    const Vec2f yPosition = NdcToWindow(simplexMapper_.ToNdcPosition(
        SimplexState::Normalized(0.0, 1.0, 0.0)
    ));
    const Vec2f zPosition = NdcToWindow(simplexMapper_.ToNdcPosition(
        SimplexState::Normalized(0.0, 0.0, 1.0)
    ));

    drawList->AddText(
        ImVec2(xPosition.x - 42.0F, xPosition.y - 27.0F),
        color,
        "x  Cooperators"
    );
    drawList->AddText(
        ImVec2(yPosition.x - 92.0F, yPosition.y + 5.0F),
        color,
        "y  Defectors"
    );
    drawList->AddText(
        ImVec2(zPosition.x + 10.0F, zPosition.y + 5.0F),
        color,
        "z  Loners"
    );
}

void SimplexBeastApplication::RenderScene()
{
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glClearColor(0.975F, 0.978F, 0.985F, 1.0F);
    glClear(GL_COLOR_BUFFER_BIT);

    if (showHeatMap_ && fieldVisualization_.HeatMap().has_value()) {
        heatMapShader_.Use();
        fieldVisualization_.DrawHeatMap();
    }
    else {
        colorShader_.Use();
        simplexMesh_.Draw();
    }

    colorShader_.Use();
    if (showStreamlines_ && fieldVisualization_.Streamlines().has_value()) {
        fieldVisualization_.DrawStreamlines();
    }
    if (!realTimeMode_) {
        trajectoryMesh_.Draw();
    }
    if (equilibriumSweep_.has_value()) {
        equilibriumSweepPathMesh_.Draw();
        pointShader_.Use();
        equilibriumSweepSampleMesh_.Draw();
        colorShader_.Use();
    }
    eigenvectorPathMesh_.Draw();

    outlineShader_.Use();
    simplexMesh_.DrawOutline();

    if (showEquilibria_) {
        pointShader_.Use();
        equilibriumMesh_.Draw();
    }

    if (hasSelectedEquilibrium_ && showEquilibria_) {
        ringPointShader_.Use();
        selectedEquilibriumPointMesh_.Draw();
    }
    if (hasResponseTarget_) {
        ringPointShader_.Use();
        targetPointMesh_.Draw();
    }

    pointShader_.Use();
    statePointMesh_.Draw();
    glUseProgram(0);
    glBindVertexArray(0);
}
