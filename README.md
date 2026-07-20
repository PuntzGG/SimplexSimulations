# SimplexBeast

SimplexBeast is an interactive C++20 scientific visualizer for three-strategy evolutionary dynamics in the Optional Public Goods Game (OPGG) with punishment. It combines validated numerical integration, model-aware equilibrium analysis, and a phase-portrait view inspired by the project's reference figure.

The maintained application is [SimplexBeast.sln](SimplexBeast.sln). The historical GitHub repository name remains `SimplexSimulations`.

## Capabilities

- Switch between Logit, Equal-split Best Response, and Replicator dynamics without changing the shared OPGG payoff definition.
- Click or drag to select a population state; the highlighted static trajectory is regenerated once per rendered frame.
- See the model-defined response target as a purple ring for Logit and Best Response. Replicator correctly has no separate target.
- Switch to Real Time mode, click or drag to reseed, Start/Pause/Reset playback, and change playback speed independently of the fixed numerical timestep.
- Display a full-simplex speed heat map with adjustable barycentric resolution, a dark-blue-to-red scientific palette, numeric legend, and relative or locked normalization.
- Display deterministic streamline curves seeded on the interior and all borders, with adjustable density, integration time, timestep, and fixed-pixel downstream arrowheads.
- Show model-aware equilibria: interior Logit points, Replicator vertices/edges/interior points, and the finite equal-split Best Response candidate set.
- Shift+click a visible equilibrium to inspect its coordinates, payoffs, reduced Jacobian, stability classification, eigenvalues, and simplex-tangent eigenvectors.
- Generate Logit-noise and punishment-fraction equilibrium sweeps.
- Resize the high-DPI window without stretching the scientific viewport or breaking click mapping.

## Interaction

- Left-click inside the simplex: choose the current state.
- Left-drag: move the state; dragging outside clamps to the nearest simplex border.
- Shift+left-click a green marker: select an equilibrium. Shift+clicking empty space clears the selection and never moves the state.
- `1`, `2`, `3`: choose the cooperator, defector, or loner vertex.
- `C`: choose the simplex center.
- `Space`: Start/Pause while Real Time mode is active.

Real Time mode starts paused. Exiting it adopts the live state as the next static starting state. Changing the active dynamic or OPGG parameters keeps the valid live state but clears the time accumulator and pauses playback.

## Scientific conventions

A state is

```text
s = (x, y, z),  x >= 0, y >= 0, z >= 0, x + y + z = 1
```

where `x`, `y`, and `z` are the cooperator, defector, and loner frequencies. All dynamics use `OpggPayoffEvaluator` and the same payoff vector `pi(s)`.

```text
Logit:        ds_i/dt = softmax_i(pi / eta) - s_i
Best Response: ds/dt = B(s) - s
Replicator:   ds_i/dt = s_i [pi_i(s) - sum_j s_j pi_j(s)]
```

`B(s)` splits mass uniformly across every payoff maximizer within a documented absolute-plus-relative tolerance. That convention makes ties deterministic but leaves the vector field nonsmooth at switching surfaces; the inspector therefore does not claim a classical Jacobian there.

Heat-map speed is the population-space norm

```text
|ds/dt| = sqrt(dx*dx + dy*dy + dz*dz)
```

The equilibrium inspector differentiates the reduced chart `(x, y)` with `z = 1 - x - y`. It uses central finite differences in the interior and feasible one-sided stencils on boundaries, then reports eigenvectors both in reduced coordinates and as full tangent directions whose three components sum to zero.

The RK4 integrator validates every stage, reconstructs `z` from the two independent coordinates, and subdivides a step rather than projecting a materially invalid stage back into the simplex. Real Time playback uses this same stepper with a fixed `dt`; frame duration only decides how many fixed steps are consumed.

More detail is preserved in [FEATURE_PLAN.md](FEATURE_PLAN.md) and [SCIENTIFIC_VALIDATION.md](SCIENTIFIC_VALIDATION.md).

## Architecture

- `SimplexBeastApplication`: SDL/ImGui interaction, invalidation decisions, and render ordering.
- `SimulationSession` and `DynamicsFactory`: transactional model, parameter, state, and trajectory ownership.
- `OpggPayoffEvaluator`, `LogitDynamics`, `BestResponseDynamics`, `ReplicatorDynamics`: pure scientific model layer.
- `SimplexTrajectoryIntegrator` and `RealTimeSimulationController`: shared validated stepping and wall-clock playback.
- `SimplexEquilibriumFinder`, `EquilibriumInspector`, `SimplexJacobianAnalyzer`: model-aware rest points and local stability analysis.
- `SpeedHeatMapGenerator` and `StreamlineFieldGenerator`: deterministic CPU field generation without OpenGL dependencies.
- `ScientificFieldVisualization` and the mesh classes: RAII-owned GPU buffers and drawing.
- `tests/ScientificCoreTests.cpp`: dependency-free scientific regression suite.

## Build on Windows

Requirements:

- Visual Studio 2022 with Desktop development with C++
- x64 toolchain and Windows SDK
- Git submodules
- vcpkg

Initialize Dear ImGui and install the manifest dependencies into the project-local path expected by the Visual Studio project:

```powershell
git submodule update --init --recursive
vcpkg install --triplet x64-windows --x-manifest-root=. --x-install-root=deps\vcpkg_installed
```

Open `SimplexBeast.sln`, select `x64`, and build Debug or Release. Release is a normal windowed application; Debug keeps a console for diagnostics. SDL3 and GLEW DLLs are copied beside the executable by the post-build step.

## Scientific-core tests

Run from a Visual Studio Developer PowerShell:

```powershell
cmake -S tests -B build-tests
cmake --build build-tests --config Release --clean-first
ctest --test-dir build-tests -C Release --output-on-failure
```

The target compiles at `/W4 /WX /permissive-` under MSVC. It covers payoff equivalence, all dynamics and invariants, RK4 behavior, transactional switching, Real Time frame partitioning and stall handling, model-aware equilibria, Jacobian/eigenpair correctness, heat-map topology and normalization, and deterministic border-inclusive streamline output.

## Interpretation limits

SimplexBeast is a numerical scientific tool, not a proof system. Every displayed equilibrium is residual-verified, and interior Replicator candidates additionally equalize all strategy payoffs, but a finite multi-seed search cannot prove mathematical completeness. Logit sweep lines are conservative continuity links between independently verified samples.

Relative heat-map normalization rescales each displayed field separately. Use a locked numeric range when colors must be compared across parameter sets or dynamics.
