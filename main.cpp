#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <GL/glew.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <utility>
#include <vector>

#include "EquilibriumMesh.h"
#include "EquilibriumPathMesh.h"
#include "ImGuiLayer.h"
#include "LogitEquilibriumSweep.h"
#include "PointMesh.h"
#include "SdlOpenGlWindow.h"
#include "SdlSystem.h"
#include "ShaderProgram.h"
#include "ShaderSources.h"
#include "SimplexEquilibriumFinder.h"
#include "SimplexMapper.h"
#include "SimplexMesh.h"
#include "SimulationSession.h"
#include "TrajectoryMesh.h"
#include "imgui.h"

namespace
{
    constexpr int kWindowWidth = 1200;
    constexpr int kWindowHeight = 1100;

        struct ViewportRectangle final
    {
        float left = 0.0f;
        float top = 0.0f;
        float width = 1.0f;
        float height = 1.0f;
    };

    [[nodiscard]] bool QueryWindowSize(
        SDL_Window* window,
        int& width,
        int& height
    )
    {
        width = 0;
        height = 0;
        return window != nullptr
            && SDL_GetWindowSize(window, &width, &height)
            && width > 0
            && height > 0;
    }

    [[nodiscard]] ViewportRectangle ComputeViewport(
        int availableWidth,
        int availableHeight
    )
    {
        const float width = static_cast<float>(availableWidth);
        const float height = static_cast<float>(availableHeight);
        const float targetAspect =
            static_cast<float>(kWindowWidth)
            / static_cast<float>(kWindowHeight);
        const float availableAspect = width / height;

        if (availableAspect > targetAspect) {
            const float viewportWidth = height * targetAspect;
            return ViewportRectangle{
                (width - viewportWidth) * 0.5f,
                0.0f,
                viewportWidth,
                height
            };
        }

        const float viewportHeight = width / targetAspect;
        return ViewportRectangle{
            0.0f,
            (height - viewportHeight) * 0.5f,
            width,
            viewportHeight
        };
    }

    [[nodiscard]] Vec2f WindowToNdcPosition(
        SDL_Window* window,
        float windowX,
        float windowY
    )
    {
        int width = 0;
        int height = 0;
        if (!QueryWindowSize(window, width, height)) {
            return {};
        }

        const ViewportRectangle viewport = ComputeViewport(width, height);
        return {
            (2.0f * (windowX - viewport.left) / viewport.width) - 1.0f,
            1.0f - (2.0f * (windowY - viewport.top) / viewport.height)
        };
    }

    [[nodiscard]] ImVec2 NdcToWindowPosition(
        SDL_Window* window,
        Vec2f ndcPosition
    )
    {
        int width = 0;
        int height = 0;
        if (!QueryWindowSize(window, width, height)) {
            return {};
        }

        const ViewportRectangle viewport = ComputeViewport(width, height);
        return ImVec2{
            viewport.left
                + (ndcPosition.x + 1.0f) * 0.5f * viewport.width,
            viewport.top
                + (1.0f - ndcPosition.y) * 0.5f * viewport.height
        };
    }

    void UpdateOpenGlViewport(SDL_Window* window)
    {
        int pixelWidth = 0;
        int pixelHeight = 0;
        if (window == nullptr
            || !SDL_GetWindowSizeInPixels(
                window,
                &pixelWidth,
                &pixelHeight
            )
            || pixelWidth <= 0
            || pixelHeight <= 0) {
            return;
        }

        const ViewportRectangle viewport = ComputeViewport(
            pixelWidth,
            pixelHeight
        );
        const int viewportX = static_cast<int>(std::lround(viewport.left));
        const int viewportY = static_cast<int>(std::lround(
            static_cast<float>(pixelHeight)
                - viewport.top
                - viewport.height
        ));
        const int viewportWidth = static_cast<int>(std::lround(viewport.width));
        const int viewportHeight = static_cast<int>(std::lround(viewport.height));

        glViewport(
            viewportX,
            viewportY,
            viewportWidth,
            viewportHeight
        );
    }

    [[nodiscard]] std::vector<Vec2f> BuildTrajectoryPositions(
        const std::vector<SimplexState>& states,
        const SimplexMapper& simplexMapper
    )
    {
        std::vector<Vec2f> positions;
        positions.reserve(states.size());

        for (const SimplexState& state : states) {
            positions.push_back(simplexMapper.ToNdcPosition(state));
        }

        return positions;
    }

    [[nodiscard]] bool IsEquilibriumSweepCompatible(
        const LogitEquilibriumSweepResult& sweep,
        const OpggParameters& parameters
    )
    {
        const OpggParameters& baseline = sweep.baselineParameters;

        switch (sweep.parameter) {
        case LogitEquilibriumSweepParameter::LogitNoise:
            return parameters.groupSize == baseline.groupSize
                && parameters.multiplicationFactor
                == baseline.multiplicationFactor
                && parameters.lonerPayoffMultiplier
                == baseline.lonerPayoffMultiplier
                && parameters.contributionCost
                == baseline.contributionCost
                && parameters.punishmentFraction
                == baseline.punishmentFraction;

        case LogitEquilibriumSweepParameter::PunishmentFraction:
            return parameters.groupSize == baseline.groupSize
                && parameters.multiplicationFactor
                == baseline.multiplicationFactor
                && parameters.lonerPayoffMultiplier
                == baseline.lonerPayoffMultiplier
                && parameters.contributionCost
                == baseline.contributionCost
                && parameters.logitNoise == baseline.logitNoise;
        }

        return false;
    }
}

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    SdlSystem sdlSystem;

    if (!sdlSystem.Initialize()) {
        return 1;
    }

    SdlOpenGlWindow window;

    if (!window.Create(
        "Simplex Simulations",
        kWindowWidth,
        kWindowHeight
    )) {
        return 1;
    }

    glewExperimental = GL_TRUE;

    const GLenum glewError = glewInit();

    if (glewError != GLEW_OK) {
        std::cerr
            << "glewInit failed: "
            << glewGetErrorString(glewError)
            << "\n";

        return 1;
    }

    ImGuiLayer imguiLayer;

    if (!imguiLayer.Initialize(
        window.NativeWindow(),
        window.GlContext(),
        "#version 330 core"
    )) {
        return 1;
    }

    ShaderProgram shaderProgram;

    if (!shaderProgram.Create(
        ShaderSources::kSimplexVertex,
        ShaderSources::kSimplexFragment
    )) {
        return 1;
    }

    ShaderProgram simplexOutlineShader;

    if (!simplexOutlineShader.Create(
        ShaderSources::kSimplexVertex,
        ShaderSources::kSimplexOutlineFragment
    )) {
        return 1;
    }

    ShaderProgram pointShaderProgram;

    if (!pointShaderProgram.Create(
        ShaderSources::kPointVertex,
        ShaderSources::kPointFragment
    )) {
        return 1;
    }

    SimplexMesh simplexMesh;

    if (!simplexMesh.Create()) {
        std::cerr << "Failed to create simplex mesh.\n";
        return 1;
    }

    PointMesh statePointMesh;

    if (!statePointMesh.Create()) {
        std::cerr << "Failed to create state point mesh.\n";
        return 1;
    }

    TrajectoryMesh trajectoryMesh;

    if (!trajectoryMesh.Create()) {
        std::cerr << "Failed to create trajectory mesh.\n";
        return 1;
    }

    EquilibriumMesh equilibriumMesh;

    if (!equilibriumMesh.Create()) {
        std::cerr << "Failed to create equilibrium mesh.\n";
        return 1;
    }

    equilibriumMesh.SetSize(15.0f);

    EquilibriumPathMesh equilibriumPathMesh;

    if (!equilibriumPathMesh.Create()) {
        std::cerr << "Failed to create equilibrium path mesh.\n";
        return 1;
    }

    EquilibriumMesh equilibriumSweepSampleMesh;

    if (!equilibriumSweepSampleMesh.Create()) {
        std::cerr
            << "Failed to create equilibrium sweep sample mesh.\n";

        return 1;
    }

    equilibriumSweepSampleMesh.SetSize(5.0f);

    const SimplexMapper simplexMapper(
        Vec2f{ 0.0f, 0.75f },   // Cooperators / x
        Vec2f{ 0.75f, -0.55f }, // Defectors / y
        Vec2f{ -0.75f, -0.55f }   // Loners / z
    );

    const Vec2f xSimplexVertex = simplexMapper.ToNdcPosition(
        SimplexState::Normalized(1.0, 0.0, 0.0)
    );

    const Vec2f ySimplexVertex = simplexMapper.ToNdcPosition(
        SimplexState::Normalized(0.0, 1.0, 0.0)
    );

    const Vec2f zSimplexVertex = simplexMapper.ToNdcPosition(
        SimplexState::Normalized(0.0, 0.0, 1.0)
    );

    SimulationSession simulation;

    if (!simulation.Initialize()) {
        std::cerr << "Failed to initialize the simulation session.\n";
        return 1;
    }

    std::vector<SimplexEquilibrium> equilibria;
    bool showEquilibria = false;

    constexpr int kNoEquilibriumSweep = 0;
    constexpr int kLogitNoiseSweep = 1;
    constexpr int kPunishmentSweep = 2;

    int selectedEquilibriumSweep = kNoEquilibriumSweep;

    std::optional<LogitEquilibriumSweepResult> equilibriumSweepResult;

    auto refreshSimulationVisualization = [&]() -> bool
        {
            const std::vector<SimplexState>& trajectory =
                simulation.Trajectory();

            if (trajectory.empty()) {
                std::cerr
                    << "Simulation session returned an empty trajectory.\n";

                return false;
            }

            const std::vector<Vec2f> trajectoryPositions =
                BuildTrajectoryPositions(
                    trajectory,
                    simplexMapper
                );

            if (!trajectoryMesh.SetPoints(
                trajectoryPositions,
                0.05f,
                0.05f,
                0.05f
            )) {
                std::cerr << "Failed to upload trajectory mesh.\n";
                return false;
            }

            statePointMesh.SetPosition(
                simplexMapper.ToNdcPosition(simulation.CurrentState())
            );

            return true;
        };

    auto rebuildEquilibriumVisualization = [&]() -> bool
        {
            const auto foundEquilibria = simulation.FindEquilibria();

            if (!foundEquilibria.has_value()) {
                std::cerr << "Failed to find Logit rest points.\n";
                return false;
            }

            std::vector<Vec2f> equilibriumPositions;
            equilibriumPositions.reserve(foundEquilibria->size());

            for (const SimplexEquilibrium& equilibrium
                : *foundEquilibria) {
                equilibriumPositions.push_back(
                    simplexMapper.ToNdcPosition(equilibrium.state)
                );
            }

            if (!equilibriumMesh.SetPoints(
                equilibriumPositions,
                0.1f,
                0.85f,
                0.35f
            )) {
                std::cerr
                    << "Failed to upload equilibrium markers.\n";

                return false;
            }

            equilibria = *foundEquilibria;
            return true;
        };

    auto rebuildEquilibriumSweepVisualization =
        [&](const LogitEquilibriumSweepSettings& settings) -> bool
        {
            const auto generatedSweep =
                simulation.GenerateEquilibriumSweep(settings);

            if (!generatedSweep.has_value()) {
                std::cerr
                    << "Failed to generate equilibrium branches.\n";

                return false;
            }

            std::vector<std::vector<Vec2f>> branchPaths;
            branchPaths.reserve(generatedSweep->branches.size());

            std::vector<Vec2f> sweepSamplePositions;

            for (const LogitEquilibriumBranch& branch
                : generatedSweep->branches) {
                std::vector<Vec2f> path;
                path.reserve(branch.samples.size());

                for (const LogitEquilibriumSweepSample& sample
                    : branch.samples) {
                    const Vec2f position = simplexMapper.ToNdcPosition(
                        sample.equilibrium.state
                    );

                    path.push_back(position);
                    sweepSamplePositions.push_back(position);
                }

                branchPaths.push_back(std::move(path));
            }

            float red = 0.1f;
            float green = 0.75f;
            float blue = 0.95f;

            if (settings.parameter
                == LogitEquilibriumSweepParameter::PunishmentFraction) {
                red = 0.75f;
                green = 0.25f;
                blue = 0.95f;
            }

            if (!equilibriumPathMesh.SetPaths(
                branchPaths,
                red,
                green,
                blue
            )) {
                std::cerr
                    << "Failed to upload equilibrium branches.\n";

                return false;
            }

            if (!equilibriumSweepSampleMesh.SetPoints(
                sweepSamplePositions,
                red,
                green,
                blue
            )) {
                std::cerr
                    << "Failed to upload equilibrium sweep samples.\n";

                return false;
            }

            equilibriumSweepResult = *generatedSweep;
            return true;
        };

    auto setDisplayedState = [&](const SimplexState& state) -> bool
        {
            if (!simulation.SetCurrentState(state)) {
                std::cerr
                    << "Failed to update the simulation state.\n";

                return false;
            }

            return refreshSimulationVisualization();
        };

    auto tryStartDraggingFromWindowPosition =
        [&](float windowX, float windowY) -> bool
        {
            const Vec2f clickedNdcPosition =
                WindowToNdcPosition(window.NativeWindow(), windowX, windowY);

            const auto clickedState =
                simplexMapper.FromNdcPosition(clickedNdcPosition);

            if (!clickedState.has_value()) {
                return false;
            }

            if (!setDisplayedState(*clickedState)) {
                return false;
            }

            return true;
        };

    auto setDisplayedStateFromWindowPositionClamped =
        [&](float windowX, float windowY) -> bool
        {
            const Vec2f draggedNdcPosition =
                WindowToNdcPosition(window.NativeWindow(), windowX, windowY);

            const auto draggedState =
                simplexMapper.FromNdcPositionClamped(draggedNdcPosition);

            if (!draggedState.has_value()) {
                return false;
            }

            return setDisplayedState(*draggedState);
        };

    auto drawSimplexCoordinateLabels = [&]()
        {
            ImDrawList* drawList = ImGui::GetForegroundDrawList();

            const ImU32 labelColor = IM_COL32(25, 25, 25, 255);

            const ImVec2 xPosition =
                NdcToWindowPosition(window.NativeWindow(), xSimplexVertex);

            const ImVec2 yPosition =
                NdcToWindowPosition(window.NativeWindow(), ySimplexVertex);

            const ImVec2 zPosition =
                NdcToWindowPosition(window.NativeWindow(), zSimplexVertex);

            drawList->AddText(
                ImVec2(xPosition.x - 4.0f, xPosition.y + 14.0f),
                labelColor,
                "x"
            );

            drawList->AddText(
                ImVec2(yPosition.x + 18.0f, yPosition.y - 28.0f),
                labelColor,
                "y"
            );

            drawList->AddText(
                ImVec2(zPosition.x - 28.0f, zPosition.y - 28.0f),
                labelColor,
                "z"
            );
        };

    statePointMesh.SetColor(1.0f, 0.3f, 0.0f);
    statePointMesh.SetSize(14.0f);

    if (!refreshSimulationVisualization()) {
        return 1;
    }

    std::cout << "Window created successfully.\n";

    UpdateOpenGlViewport(window.NativeWindow());

    bool running = true;
    bool draggingSimplexPoint = false;

    while (running) {
        SDL_Event event;

        while (SDL_PollEvent(&event)) {
            imguiLayer.ProcessEvent(event);

            if (event.type == SDL_EVENT_QUIT) {
                running = false;
            }

            if (event.type == SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED) {
                UpdateOpenGlViewport(window.NativeWindow());
            }

            if (
                event.type == SDL_EVENT_KEY_DOWN
                && !event.key.repeat
                && !imguiLayer.WantsKeyboardInput()
                ) {
                switch (event.key.scancode) {
                case SDL_SCANCODE_1:
                    if (!setDisplayedState(
                        SimplexState::Normalized(1.0, 0.0, 0.0)
                    )) {
                        running = false;
                    }
                    break;

                case SDL_SCANCODE_2:
                    if (!setDisplayedState(
                        SimplexState::Normalized(0.0, 1.0, 0.0)
                    )) {
                        running = false;
                    }
                    break;

                case SDL_SCANCODE_3:
                    if (!setDisplayedState(
                        SimplexState::Normalized(0.0, 0.0, 1.0)
                    )) {
                        running = false;
                    }
                    break;

                case SDL_SCANCODE_C:
                    if (!setDisplayedState(
                        SimplexState::Normalized(1.0, 1.0, 1.0)
                    )) {
                        running = false;
                    }
                    break;

                default:
                    break;
                }
            }

            if (
                event.type == SDL_EVENT_MOUSE_BUTTON_DOWN
                && event.button.button == SDL_BUTTON_LEFT
                && !imguiLayer.WantsMouseInput()
                ) {
                draggingSimplexPoint = tryStartDraggingFromWindowPosition(
                    event.button.x,
                    event.button.y
                );

                if (!draggingSimplexPoint) {
                    continue;
                }
            }

            if (
                event.type == SDL_EVENT_MOUSE_MOTION
                && draggingSimplexPoint
                && !imguiLayer.WantsMouseInput()
                ) {
                if (!setDisplayedStateFromWindowPositionClamped(
                    event.motion.x,
                    event.motion.y
                )) {
                    running = false;
                }
            }

            if (
                event.type == SDL_EVENT_MOUSE_BUTTON_UP
                && event.button.button == SDL_BUTTON_LEFT
                ) {
                draggingSimplexPoint = false;
            }
        }

        imguiLayer.BeginFrame();

        drawSimplexCoordinateLabels();

        ImGui::Begin("Simulation Controls");

        const SimplexState& currentState = simulation.CurrentState();

        ImGui::TextUnformatted("Current simplex state");
        ImGui::Text("x (Cooperators): %.4f", currentState.X());
        ImGui::Text("y (Defectors): %.4f", currentState.Y());
        ImGui::Text("z (Loners): %.4f", currentState.Z());

        ImGui::Separator();

        ImGui::TextUnformatted("Logit dynamics");
        ImGui::Separator();

        constexpr int kMinimumGroupSize = 2;
        constexpr int kMaximumGroupSize = 20;
        constexpr double kModelBoundaryMargin = 1e-4;
        constexpr double kMinimumContributionCost = 0.01;
        constexpr double kMaximumContributionCost = 5.0;
        constexpr double kMinimumPunishmentFraction = 0.0;
        constexpr double kMaximumPunishmentFraction = 1.0;
        constexpr double kMinimumLogitNoise = 0.001;
        constexpr double kMaximumLogitNoise = 1.0;

        OpggParameters candidateParameters = simulation.Parameters();
        bool parametersChanged = false;

        if (ImGui::SliderInt(
                "Group size (n)",
                &candidateParameters.groupSize,
                kMinimumGroupSize,
                kMaximumGroupSize)) {
            parametersChanged = true;
        }

        const double minimumMultiplicationFactor =
            1.0 + 2.0 * kModelBoundaryMargin;
        const double maximumMultiplicationFactor =
            static_cast<double>(candidateParameters.groupSize)
                - kModelBoundaryMargin;
        const double clampedMultiplicationFactor = std::clamp(
            candidateParameters.multiplicationFactor,
            minimumMultiplicationFactor,
            maximumMultiplicationFactor
        );
        if (clampedMultiplicationFactor
            != candidateParameters.multiplicationFactor) {
            candidateParameters.multiplicationFactor =
                clampedMultiplicationFactor;
            parametersChanged = true;
        }

        if (ImGui::SliderScalar(
                "Multiplication factor (r)",
                ImGuiDataType_Double,
                &candidateParameters.multiplicationFactor,
                &minimumMultiplicationFactor,
                &maximumMultiplicationFactor,
                "%.3f")) {
            parametersChanged = true;
        }

        const double minimumLonerPayoffMultiplier = kModelBoundaryMargin;
        const double maximumLonerPayoffMultiplier = std::max(
            minimumLonerPayoffMultiplier,
            candidateParameters.multiplicationFactor
                - 1.0
                - kModelBoundaryMargin
        );
        const double clampedLonerPayoffMultiplier = std::clamp(
            candidateParameters.lonerPayoffMultiplier,
            minimumLonerPayoffMultiplier,
            maximumLonerPayoffMultiplier
        );
        if (clampedLonerPayoffMultiplier
            != candidateParameters.lonerPayoffMultiplier) {
            candidateParameters.lonerPayoffMultiplier =
                clampedLonerPayoffMultiplier;
            parametersChanged = true;
        }

        if (ImGui::SliderScalar(
                "Loner payoff multiplier (sigma)",
                ImGuiDataType_Double,
                &candidateParameters.lonerPayoffMultiplier,
                &minimumLonerPayoffMultiplier,
                &maximumLonerPayoffMultiplier,
                "%.3f")) {
            parametersChanged = true;
        }

        if (ImGui::SliderScalar(
                "Contribution cost (c)",
                ImGuiDataType_Double,
                &candidateParameters.contributionCost,
                &kMinimumContributionCost,
                &kMaximumContributionCost,
                "%.3f")) {
            parametersChanged = true;
        }

        if (ImGui::SliderScalar(
                "Punishment fraction (v)",
                ImGuiDataType_Double,
                &candidateParameters.punishmentFraction,
                &kMinimumPunishmentFraction,
                &kMaximumPunishmentFraction,
                "%.3f")) {
            parametersChanged = true;
        }

        if (ImGui::SliderScalar(
                "Logit noise (eta)",
                ImGuiDataType_Double,
                &candidateParameters.logitNoise,
                &kMinimumLogitNoise,
                &kMaximumLogitNoise,
                "%.4f",
                ImGuiSliderFlags_Logarithmic
                    | ImGuiSliderFlags_ClampOnInput)) {
            parametersChanged = true;
        }

        ImGui::TextUnformatted("Model domain: 1 < r < n and 0 < sigma < r - 1.");


        if (parametersChanged) {
            const bool keepEquilibriumSweep =
                !equilibriumSweepResult.has_value()
                || IsEquilibriumSweepCompatible(
                    *equilibriumSweepResult,
                    candidateParameters
                );

            if (!simulation.SetParameters(candidateParameters)) {
                std::cerr
                    << "Failed to update simulation parameters.\n";
            }
            else {
                if (!keepEquilibriumSweep) {
                    equilibriumSweepResult.reset();
                }

                if (
                    !refreshSimulationVisualization()
                    || (
                        showEquilibria
                        && !rebuildEquilibriumVisualization()
                        )
                    ) {
                    running = false;
                }
            }
        }

        ImGui::Separator();
        ImGui::TextUnformatted("Lower eta: closer to best response.");
        ImGui::TextUnformatted("Higher eta: more exploratory choices.");

        ImGui::Separator();

        const bool equilibriumVisibilityChanged = ImGui::Checkbox(
            "Show Logit rest points",
            &showEquilibria
        );

        if (equilibriumVisibilityChanged && showEquilibria) {
            if (!rebuildEquilibriumVisualization()) {
                showEquilibria = false;
            }
        }

        if (showEquilibria) {
            if (equilibria.empty()) {
                ImGui::TextUnformatted("No verified rest points found.");
            }
            else {
                double largestResidual = 0.0;

                for (const SimplexEquilibrium& equilibrium : equilibria) {
                    largestResidual = std::max(
                        largestResidual,
                        equilibrium.residual
                    );
                }

                ImGui::Text(
                    "Verified rest points: %d",
                    static_cast<int>(equilibria.size())
                );

                ImGui::Text(
                    "Largest residual: %.2e",
                    largestResidual
                );
            }
        }

        ImGui::Separator();
        ImGui::TextUnformatted("Equilibrium branches");

        const int previousEquilibriumSweep = selectedEquilibriumSweep;

        ImGui::RadioButton(
            "Sweep off",
            &selectedEquilibriumSweep,
            kNoEquilibriumSweep
        );

        ImGui::SameLine();

        ImGui::RadioButton(
            "Sweep eta",
            &selectedEquilibriumSweep,
            kLogitNoiseSweep
        );

        ImGui::SameLine();

        ImGui::RadioButton(
            "Sweep punishment",
            &selectedEquilibriumSweep,
            kPunishmentSweep
        );

        if (selectedEquilibriumSweep != previousEquilibriumSweep) {
            equilibriumSweepResult.reset();
        }

        if (selectedEquilibriumSweep != kNoEquilibriumSweep) {
            LogitEquilibriumSweepSettings sweepSettings;

            if (selectedEquilibriumSweep == kLogitNoiseSweep) {
                sweepSettings.parameter =
                    LogitEquilibriumSweepParameter::LogitNoise;

                sweepSettings.minimumParameter = kMinimumLogitNoise;
                sweepSettings.maximumParameter = kMaximumLogitNoise;
            }
            else {
                sweepSettings.parameter =
                    LogitEquilibriumSweepParameter::PunishmentFraction;

                sweepSettings.minimumParameter =
                    kMinimumPunishmentFraction;

                sweepSettings.maximumParameter =
                    kMaximumPunishmentFraction;
            }

            const char* generateLabel =
                selectedEquilibriumSweep == kLogitNoiseSweep
                ? "Generate eta branches"
                : "Generate punishment branches";

            if (ImGui::Button(generateLabel)) {
                equilibriumSweepResult.reset();

                if (!rebuildEquilibriumSweepVisualization(
                    sweepSettings
                )) {
                    equilibriumSweepResult.reset();
                }
            }

            ImGui::Text(
                "Range: %.4g to %.4g",
                sweepSettings.minimumParameter,
                sweepSettings.maximumParameter
            );

            ImGui::TextUnformatted(
                "Other game parameters are held fixed."
            );

            if (equilibriumSweepResult.has_value()) {
                int visibleBranchCount = 0;
                int verifiedSampleCount = 0;
                double largestResidual = 0.0;

                for (const LogitEquilibriumBranch& branch
                    : equilibriumSweepResult->branches) {
                    if (branch.samples.size() >= 2) {
                        ++visibleBranchCount;
                    }

                    for (const LogitEquilibriumSweepSample& sample
                        : branch.samples) {
                        ++verifiedSampleCount;

                        largestResidual = std::max(
                            largestResidual,
                            sample.equilibrium.residual
                        );
                    }
                }

                ImGui::Text(
                    "Visible branch lines: %d",
                    visibleBranchCount
                );

                ImGui::Text(
                    "Verified samples: %d",
                    verifiedSampleCount
                );

                ImGui::Text(
                    "Largest residual: %.2e",
                    largestResidual
                );

                ImGui::TextUnformatted(
                    "Dots are verified samples; lines are confident links."
                );
            }
            else {
                ImGui::TextUnformatted(
                    "Generate branches for the current game parameters."
                );
            }
        }

        if (ImGui::CollapsingHeader("Trajectory integration")) {
            constexpr double kMinimumTrajectoryTime = 1.0;
            constexpr double kMaximumTrajectoryTime = 20.0;

            constexpr double kMinimumTimeStep = 0.005;
            constexpr double kMaximumTimeStep = 0.1;

            TrajectorySettings candidateSettings = simulation.Settings();
            bool trajectorySettingsChanged = false;

            if (ImGui::SliderScalar(
                "Trajectory duration",
                ImGuiDataType_Double,
                &candidateSettings.totalTime,
                &kMinimumTrajectoryTime,
                &kMaximumTrajectoryTime,
                "%.1f"
            )) {
                trajectorySettingsChanged = true;
            }

            if (ImGui::SliderScalar(
                "RK4 time step",
                ImGuiDataType_Double,
                &candidateSettings.timeStep,
                &kMinimumTimeStep,
                &kMaximumTimeStep,
                "%.3f",
                ImGuiSliderFlags_Logarithmic
                | ImGuiSliderFlags_ClampOnInput
            )) {
                trajectorySettingsChanged = true;
            }

            if (trajectorySettingsChanged) {
                if (!simulation.SetTrajectorySettings(
                    candidateSettings
                )) {
                    std::cerr
                        << "Failed to update trajectory settings.\n";
                }
                else if (!refreshSimulationVisualization()) {
                    running = false;
                }
            }

            ImGui::TextUnformatted(
                "Smaller steps give a finer RK4 approximation."
            );
        }

        ImGui::End();

        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        shaderProgram.Use();

        simplexMesh.Draw();
        trajectoryMesh.Draw();

        if (
            selectedEquilibriumSweep != kNoEquilibriumSweep
            && equilibriumSweepResult.has_value()
            ) {
            equilibriumPathMesh.Draw();
        }

        simplexOutlineShader.Use();
        simplexMesh.DrawOutline();

        pointShaderProgram.Use();

        if (
            selectedEquilibriumSweep != kNoEquilibriumSweep
            && equilibriumSweepResult.has_value()
            ) {
            equilibriumSweepSampleMesh.Draw();
        }

        if (showEquilibria) {
            equilibriumMesh.Draw();
        }

        statePointMesh.Draw();

        imguiLayer.Render();

        window.SwapBuffers();
    }

    return 0;
}