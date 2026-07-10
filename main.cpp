#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <GL/glew.h>
#include <algorithm>
#include <iostream>
#include <vector>
#include <optional>
#include <utility>

#include "EquilibriumPathMesh.h"
#include "LogitEquilibriumSweep.h"
#include "EquilibriumMesh.h"
#include "SimplexEquilibriumFinder.h"
#include "ShaderProgram.h"
#include "SimplexMesh.h"
#include "SimplexMapper.h"
#include "PointMesh.h"
#include "ShaderSources.h"
#include "SdlSystem.h"
#include "SdlOpenGlWindow.h"
#include "SimulationSession.h"
#include "TrajectoryMesh.h"
#include "ImGuiLayer.h"
#include "imgui.h"


namespace
{
	constexpr int kWindowWidth = 1200;
	constexpr int kWindowHeight = 1100;

	[[nodiscard]] Vec2f WindowToNdcPosition(float windowX, float windowY)
	{
		return {
			(2.0f * windowX / static_cast<float>(kWindowWidth)) - 1.0f,
			1.0f - (2.0f * windowY / static_cast<float>(kWindowHeight))
		};
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

	if (!window.Create("Simplex Simulations", kWindowWidth, kWindowHeight)) {
		return 1;
	}

	glewExperimental = GL_TRUE;
	GLenum glewError = glewInit();

	if (glewError != GLEW_OK) {
		std::cerr << "glewInit failed: " << glewGetErrorString(glewError) << "\n";
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

	if (!shaderProgram.Create(ShaderSources::kSimplexVertex, ShaderSources::kSimplexFragment)) {
		return 1;
	}

	ShaderProgram pointShaderProgram;

	if (!pointShaderProgram.Create(ShaderSources::kPointVertex, ShaderSources::kPointFragment)) {
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

	const SimplexMapper simplexMapper(
		Vec2f{ 0.0f, 0.75f },  // Cooperators
		Vec2f{-0.75f, -0.55f}, // Defectors
		Vec2f{0.75f, -0.55f}  // Loners
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
			const std::vector<SimplexState>& trajectory = simulation.Trajectory();

			if (trajectory.empty()) {
				std::cerr << "Simulation session returned an empty trajectory.\n";
				return false;
			}

			const std::vector<Vec2f> trajectoryPositions = BuildTrajectoryPositions(
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

			for (const SimplexEquilibrium& equilibrium : *foundEquilibria) {
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
				std::cerr << "Failed to upload equilibrium markers.\n";
				return false;
			}

			equilibria = *foundEquilibria;
			return true;
		};

	auto setDisplayedState = [&](const SimplexState& state) -> bool
		{
			if (!simulation.SetCurrentState(state)) {
				std::cerr << "Failed to update the simulation state.\n";
				return false;
			}

			return refreshSimulationVisualization();
		};

	statePointMesh.SetColor(1.0f, 0.3f, 0.0f);
	statePointMesh.SetSize(14.0f);

	if (!refreshSimulationVisualization()) {
		return 1;
	}

	auto rebuildEquilibriumSweepVisualization =
		[&](const LogitEquilibriumSweepSettings& settings) -> bool
		{
			const auto generatedSweep =
				simulation.GenerateEquilibriumSweep(settings);

			if (!generatedSweep.has_value()) {
				std::cerr << "Failed to generate equilibrium branches.\n";
				return false;
			}

			std::vector<std::vector<Vec2f>> branchPaths;
			branchPaths.reserve(generatedSweep->branches.size());

			for (const LogitEquilibriumBranch& branch
				: generatedSweep->branches) {
				std::vector<Vec2f> path;
				path.reserve(branch.samples.size());

				for (const LogitEquilibriumSweepSample& sample
					: branch.samples) {
					path.push_back(
						simplexMapper.ToNdcPosition(
							sample.equilibrium.state
						)
					);
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
				std::cerr << "Failed to upload equilibrium branches.\n";
				return false;
			}

			equilibriumSweepResult = *generatedSweep;
			return true;
		};

	std::cout << "Window created successfully.\n";

	glViewport(0, 0, kWindowWidth, kWindowHeight);


	bool running = true;
	bool draggingSimplexPoint = false;

	auto tryStartDraggingFromWindowPosition = [&](float windowX, float windowY) -> bool
		{
			const Vec2f clickedNdcPosition = WindowToNdcPosition(windowX, windowY);
			const auto clickedState = simplexMapper.FromNdcPosition(clickedNdcPosition);

			if (!clickedState.has_value()) {
				return false;
			}

			if (!setDisplayedState(*clickedState)) {
				running = false;
				return false;
			}

			return true;
		};

	auto setDisplayedStateFromWindowPositionClamped = [&](float windowX, float windowY)
		{
			const Vec2f draggedNdcPosition = WindowToNdcPosition(windowX, windowY);
			const auto draggedState = simplexMapper.FromNdcPositionClamped(draggedNdcPosition);

			if (draggedState.has_value() && !setDisplayedState(*draggedState)) {
				running = false;
			}
		};

	while (running) {
		SDL_Event event;

		while (SDL_PollEvent(&event)) {
			imguiLayer.ProcessEvent(event);
			if (event.type == SDL_EVENT_QUIT) {
				running = false;
			}

			if (
				event.type == SDL_EVENT_KEY_DOWN
				&& !event.key.repeat
				&& !imguiLayer.WantsKeyboardInput()
				) {
				switch (event.key.scancode) {
				case SDL_SCANCODE_1:
					if (!setDisplayedState(SimplexState::Normalized(1.0, 0.0, 0.0))) {
						running = false;
					}
					break;

				case SDL_SCANCODE_2:
					if (!setDisplayedState(SimplexState::Normalized(0.0, 1.0, 0.0))) {
						running = false;
					}
					break;

				case SDL_SCANCODE_3:
					if (!setDisplayedState(SimplexState::Normalized(0.0, 0.0, 1.0))) {
						running = false;
					}
					break;

				case SDL_SCANCODE_C:
					if (!setDisplayedState(SimplexState::Normalized(1.0, 1.0, 1.0))) {
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
			}

			if (
				event.type == SDL_EVENT_MOUSE_MOTION
				&& draggingSimplexPoint
				&& !imguiLayer.WantsMouseInput()
				) {
				setDisplayedStateFromWindowPositionClamped(
					event.motion.x,
					event.motion.y
				);
			}

			if (event.type == SDL_EVENT_MOUSE_BUTTON_UP && event.button.button == SDL_BUTTON_LEFT) {
				draggingSimplexPoint = false;
			}
		}

		imguiLayer.BeginFrame();

		ImGui::Begin("Simulation Controls");

		ImGui::TextUnformatted("Logit dynamics");
		ImGui::Separator();

		constexpr int kMinimumGroupSize = 2;
		constexpr int kMaximumGroupSize = 20;

		constexpr double kMinimumMultiplicationFactor = 0.1;
		constexpr double kMaximumMultiplicationFactor = 10.0;

		constexpr double kMinimumLonerPayoffMultiplier = 0.0;
		constexpr double kMaximumLonerPayoffMultiplier = 5.0;

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
			kMaximumGroupSize
		)) {
			parametersChanged = true;
		}

		if (ImGui::SliderScalar(
			"Multiplication factor (r)",
			ImGuiDataType_Double,
			&candidateParameters.multiplicationFactor,
			&kMinimumMultiplicationFactor,
			&kMaximumMultiplicationFactor,
			"%.3f"
		)) {
			parametersChanged = true;
		}

		if (ImGui::SliderScalar(
			"Loner payoff multiplier (sigma)",
			ImGuiDataType_Double,
			&candidateParameters.lonerPayoffMultiplier,
			&kMinimumLonerPayoffMultiplier,
			&kMaximumLonerPayoffMultiplier,
			"%.3f"
		)) {
			parametersChanged = true;
		}

		if (ImGui::SliderScalar(
			"Contribution cost (c)",
			ImGuiDataType_Double,
			&candidateParameters.contributionCost,
			&kMinimumContributionCost,
			&kMaximumContributionCost,
			"%.3f"
		)) {
			parametersChanged = true;
		}

		if (ImGui::SliderScalar(
			"Punishment fraction (v)",
			ImGuiDataType_Double,
			&candidateParameters.punishmentFraction,
			&kMinimumPunishmentFraction,
			&kMaximumPunishmentFraction,
			"%.3f"
		)) {
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
			| ImGuiSliderFlags_ClampOnInput
		)) {
			parametersChanged = true;
		}

		if (parametersChanged) {
			const bool keepEquilibriumSweep =
				!equilibriumSweepResult.has_value()
				|| IsEquilibriumSweepCompatible(
					*equilibriumSweepResult,
					candidateParameters
				);

			if (!simulation.SetParameters(candidateParameters)) {
				std::cerr << "Failed to update simulation parameters.\n";
			}
			else {
				if (!keepEquilibriumSweep) {
					equilibriumSweepResult.reset();
				}

				if (
					!refreshSimulationVisualization()
					|| (showEquilibria
						&& !rebuildEquilibriumVisualization())
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

				sweepSettings.minimumParameter = kMinimumPunishmentFraction;
				sweepSettings.maximumParameter = kMaximumPunishmentFraction;
			}

			const char* generateLabel =
				selectedEquilibriumSweep == kLogitNoiseSweep
				? "Generate eta branches"
				: "Generate punishment branches";

			if (ImGui::Button(generateLabel)) {
				equilibriumSweepResult.reset();

				if (!rebuildEquilibriumSweepVisualization(sweepSettings)) {
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
				if (!simulation.SetTrajectorySettings(candidateSettings)) {
					std::cerr << "Failed to update trajectory settings.\n";
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

		pointShaderProgram.Use();

		if (showEquilibria) {
			equilibriumMesh.Draw();
		}

		statePointMesh.Draw();
		imguiLayer.Render();

		window.SwapBuffers();
	}
	return 0;
}
