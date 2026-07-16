# SimplexSimulations

Interactive C++20 scientific visualizer for three-strategy evolutionary dynamics on a simplex. The current model is a Logit dynamic for the Optional Public Goods Game (OPGG), including fractional punishment, trajectory integration, verified interior rest-point detection, and parameter-sweep visualization.

## Current capabilities

- Cooperators (`x`), defectors (`y`), and loners (`z`) represented on the unit simplex.
- Mouse click and drag state selection, including nearest-border clamping outside the triangle.
- Resizable/high-DPI rendering with a preserved scientific viewport aspect ratio.
- Keyboard shortcuts: `1`, `2`, `3` select pure-strategy corners; `C` selects the center.
- OPGG parameter controls constrained to the valid model domain, plus Logit noise (`eta`).
- RK4 trajectory generation with explicit simplex-domain checks.
- Verified interior Logit rest points with reported numerical residuals.
- Logit-noise and punishment-fraction equilibrium sweeps.
- Conservative branch links: displayed sweep samples are solver results; connecting lines are inferred continuity links.

## Supported build

The maintained Visual Studio configuration is:

- Windows 10/11
- Visual Studio 2022
- x64
- C++20
- SDL3
- OpenGL 3.3 core
- GLEW (dynamic library)
- Dear ImGui submodule

Win32 configurations are intentionally removed because the repository's local dependency layout is x64-only.

## Repository setup

Clone with submodules:

```powershell
git clone --recurse-submodules https://github.com/PuntzGG/SimplexSimulations.git
```

For an existing clone:

```powershell
git submodule update --init --recursive
```

Provide local dependencies under:

```text
deps/include
deps/lib
```

The x64 linker expects:

```text
SDL3.lib
glew32.lib
opengl32.lib
```

The required runtime DLLs must be available beside the executable or on `PATH`. Dear ImGui layout state (`imgui.ini`) is intentionally local and untracked.

## Scientific conventions

A state is

```text
(x, y, z),  x >= 0, y >= 0, z >= 0, x + y + z = 1
```

where:

- `x`: cooperators
- `y`: defectors
- `z`: loners

The Logit target is the softmax of strategy payoffs, and the dynamic is target minus current state. Softmax is evaluated after subtracting the maximum payoff to prevent overflow.

The trajectory integrator uses classical RK4 in two independent simplex coordinates. It reconstructs `z = 1 - x - y`, verifies that every derivative is tangent to the simplex, and does not project materially invalid RK4 stages back into the domain. Only roundoff-sized deviations within the state tolerance are canonicalized. If a full step would create an invalid intermediate state, the step is subdivided; irrecoverable invalid dynamics fail explicitly.

`SimplexEquilibriumFinder` reports verified **interior** rest points only. Boundary equilibrium analysis is a distinct problem and is not implied by an empty interior result.

## Run scientific-core tests

The tests do not require SDL, OpenGL, GLEW, or ImGui.

```powershell
cmake -S tests -B build-tests
cmake --build build-tests --config Release
ctest --test-dir build-tests -C Release --output-on-failure
```

The suite checks model-domain validation, simplex invariants, mapper round trips, nearest-border clamping, the reference Logit derivative, finite behavior at the pure-loner limit, RK4 behavior, timestep convergence, equilibrium residuals, equilibrium sweeps, and transactional session updates.

## Important interpretation note

The program is a numerical scientific tool, not a proof system. Rest points are accepted only after residual verification, but a finite seed search cannot prove that every mathematical equilibrium has been found. Parameter-sweep branch lines are conservative continuity inferences between independently verified samples.
