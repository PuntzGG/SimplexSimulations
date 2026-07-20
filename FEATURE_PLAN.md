# SimplexBeast Feature Design and QA Record

Last updated: 2026-07-19

Planning baseline: commit `e7805e1` on `main`

Status: Implemented and acceptance-tested on Windows. Keep this document as the durable requirements, design rationale, invalidation contract, and regression checklist for future work.

## Implementation record

The requested feature set is implemented in the `SimplexBeast` Visual Studio product:

- `SimplexBeastApplication` owns the event loop, coalesced pointer input, model/mode controls, invalidation decisions, inspector UI, and explicit render order. `main.cpp` is now only the entry point.
- `OpggPayoffEvaluator`, `DynamicsFactory`, `LogitDynamics`, `BestResponseDynamics`, and `ReplicatorDynamics` provide one shared payoff model and three switchable dynamics.
- `SimplexTrajectoryIntegrator::AdvanceOneStep` is shared by batch trajectories, streamlines, and `RealTimeSimulationController`; there is no duplicate real-time integrator.
- `SimplexEquilibriumFinder`, `EquilibriumInspector`, and `SimplexJacobianAnalyzer` implement model-aware rest points, Shift+click selection, reduced Jacobians, classifications, eigenvalues, and tangent eigenvectors.
- `SpeedHeatMapGenerator`/`SpeedHeatMapMesh` implement the full barycentric scalar field, normalization modes, palette shader, and Best Response mixed-region metadata.
- `StreamlineFieldGenerator`/`StreamlineMesh` implement deterministic border-inclusive paths, `GL_LINES` batching, occupancy culling, and downstream endpoint arrows.
- `ScientificFieldVisualization` owns field caches and GPU refresh boundaries. Live playback and ordinary state dragging update only the state/target/trajectory data specified by the invalidation table below.
- `SimplexDisplayLayout` is the single source of display-triangle geometry used by both mapping and the neutral mesh.
- `tests/ScientificCoreTests.cpp` covers the pure scientific layers at MSVC `/W4 /WX /permissive-`.

Windows QA completed for Debug and Release builds, startup/layout, all three dynamic views, Real Time playback, Shift+click inspection, and the reference-inspired heat-map/streamline rendering. A visual QA pass caught and corrected Replicator near-boundary false equilibria: open-simplex Replicator roots now solve payoff equality before the original field residual is verified.

## Purpose

Read this document before modifying the visualization or dynamics features. It preserves the user-facing requirements, mathematical conventions, visual reference, architectural decisions, known risks, implementation order, and verification criteria.

Before coding, re-check the current Git branch, working tree, and source layout. The baseline commit above is a historical reference, not a command to reset the repository.

## Primary references

- Visual reference for the speed heat map and streamline arrows:
  `C:\Users\ASUS\Pictures\Screenshots\Screenshot 2026-07-19 215023.png`
- Project Best Response paper:
  `..\Sources\V2_Best_Response_Dynamics_in_OPGG_with_Punishment_CNMAC_2026.pdf`
- OPGG punishment model:
  `..\Sources\Botta2021.pdf`
- Existing Python/notebook export containing Logit and equal-split Best Response calculations:
  `..\Sources\puntos_eq_opgg.txt`
- Current project overview and numerical conventions:
  `README.md` and `SCIENTIFIC_VALIDATION.md`

The screenshot is the visual target for the heat-map palette, smoothly varying scalar background, curved streamline density, border coverage, and arrow styling.

## Requested features

1. Normal click and drag moves the starting population.
2. Shift+click selects a visible equilibrium instead of starting a drag.
3. Equilibrium markers are clickable and open an information panel.
4. The panel displays equilibrium coordinates.
5. The selected equilibrium is analyzed through the Jacobian and displays eigenvalues and eigenvectors.
6. A speed heat map covers the entire simplex, including its borders.
7. Heat-map resolution is adjustable.
8. The speed palette runs from blue for slow regions through cyan/green/yellow/orange to red for fast regions.
9. A streamline field fills the simplex and borders.
10. Each streamline ends with a directional arrowhead.
11. Streamline density and integration time are adjustable.
12. Logit, Best Response, and Replicator dynamics are switchable.
13. A purple target-population marker updates in real time as the starting state is dragged.
14. A switchable Real Time mode animates the population state over wall-clock time.
15. Clicking or dragging in Real Time mode places/reseeds the moving population point.
16. Real Time mode has explicit Start and Pause controls.
17. A playback-speed control changes simulated time per real second without changing numerical accuracy.
18. The moving simplex point, coordinates, and applicable response target update continuously while the simulation runs.

## Baseline architecture and constraints (historical)

The bullets in this section describe the pre-implementation baseline and explain why the architecture above was introduced. They are not a description of the current source tree.

- `main.cpp` currently owns application initialization, window transforms, event handling, ImGui controls, visualization rebuilding, and rendering. It is already large and must not absorb all new responsibilities.
- `SimulationSession` is currently hard-wired to `LogitDynamics`.
- `LogitDynamics` privately owns the OPGG payoff calculation, even though every planned dynamic needs the same payoffs.
- `SimplexDynamicModel` already provides a useful generic derivative interface.
- `SimplexTrajectoryIntegrator` already provides validated simplex-preserving RK4 integration for continuous vector fields.
- The integrator's reusable RK4 step is currently private; Real Time mode must reuse or expose that implementation rather than duplicating a second numerical method.
- `SimplexEquilibriumFinder` currently searches only for verified interior roots. It estimates a reduced two-dimensional Jacobian internally, but that implementation is private to the solver.
- `TrajectoryMesh` renders one `GL_LINE_STRIP`; it cannot batch independent streamlines without incorrectly connecting them.
- The mouse-motion path currently rebuilds the selected trajectory for every SDL motion event. New expensive layers must never be rebuilt this way.
- The maintained application configuration is C++20, Visual Studio 2022, x64, SDL3, OpenGL 3.3 core, GLEW, and Dear ImGui.
- New source files must be added to `SimplexBeast.vcxproj`, `SimplexBeast.vcxproj.filters`, and the scientific test target where applicable.

## Engineering principles

- Preserve the existing scientific validation and explicit failure behavior.
- Keep mathematical computation independent of OpenGL and ImGui so it can be unit tested.
- Keep GPU ownership in RAII mesh/rendering classes.
- Keep OpenGL uploads on the main thread.
- Use explicit invalidation and caching instead of recomputing every visualization after every input.
- Make model-specific limitations visible in the UI instead of silently producing misleading output.
- Do not expand `main.cpp` with another large collection of lambdas and mutable state.
- Implement in reviewable stages and preserve current Logit behavior during the architectural extraction.

## Shared OPGG payoff model

Extract the payoff calculation from `LogitDynamics` into a reusable component such as:

- `StrategyPayoffs`
- `OpggPayoffEvaluator`

All three dynamics must use exactly this shared calculation so punishment, group-size behavior, and the stable participation polynomial cannot drift between implementations.

The game parameters are:

- group size `n`
- multiplication factor `r`
- loner payoff multiplier `sigma`
- contribution cost `c`
- punishment fraction `v`/`d`

Logit noise `eta` is a dynamics setting, not an OPGG payoff parameter. It may be separated into `LogitSettings` during the dynamics refactor if doing so does not create an unnecessarily broad migration.

## Dynamics definitions

### Logit

Preserve the current definition:

```text
target_i = softmax(payoff_i / eta)
dot(s)_i = target_i - s_i
```

The stabilized maximum-payoff subtraction must remain intact.

Capabilities:

- smooth vector field
- response target available
- classical interior Jacobian available
- equilibria are interior under the current positive Logit target
- eta sweep remains available

### Replicator

Use the shared payoffs:

```text
averagePayoff = sum_i s_i * payoff_i
dot(s)_i = s_i * (payoff_i - averagePayoff)
```

Required properties:

- the derivative is finite and tangent to the simplex
- a zero population coordinate remains zero
- every pure-strategy vertex is a rest point
- edges are invariant

Capabilities:

- smooth vector field
- no scientifically natural response target
- classical Jacobian available
- interior, edge, and vertex equilibria must be searched

### Equal-split Best Response

Use one underlying generator:

```text
B(payoffs) = equal distribution over all strategies tied for maximum payoff
G(s) = B(payoffs(s)) - s
```

Ties must use a documented absolute-plus-relative payoff tolerance. Do not use exact floating-point equality.

The initial continuous visualization mode integrates:

```text
dot(s) = G(s)
```

The project paper's discrete update is:

```text
s[k+1] = s[k] + epsilon * G(s[k])
```

Do not claim the RK4 trajectory is identical to that discrete update. If exact paper reproduction becomes a requirement, add a distinct discrete stepping mode using the same generator.

Capabilities:

- response target available
- piecewise-smooth/discontinuous vector field
- classical Jacobian is valid only away from payoff-tie switching surfaces
- tie equilibria must report that classical Jacobian/eigenanalysis is undefined

The UI label should say `Equal-split Best Response` so the selection convention is explicit.

## Dynamic model API

Preserve a small derivative interface for the integrator. Add an inspection result or capability service that can expose:

- derivative
- strategy payoffs
- optional response target
- model name/kind
- smoothness and analysis capabilities

Do not force rendering or ImGui concerns into the mathematical model.

`SimulationSession::SetDynamicsKind()` must be transactional: construct and validate the candidate model and trajectory before committing the switch.

When dynamics change:

- retain common OPGG game parameters
- retain each model's private settings for when the user switches back
- clear stale equilibrium selection
- invalidate the current trajectory, target, equilibria, heat map, and streamline field
- hide Logit-only eta controls and sweeps outside Logit mode

## Target-population marker

- Current starting state: orange filled circle.
- Response target: purple ring slightly larger than the orange marker.
- Draw the purple ring so both states remain visible when they overlap.
- In static mode, recompute the target from the latest dragged starting state.
- In Real Time mode, recompute the target from the final moving live state after the frame's accepted simulation steps and upload the marker at most once per rendered frame.
- Logit target: softmax distribution.
- Best Response target: equal-split best-reply distribution.
- Replicator: hide the marker and state that a response target is not defined.

The target calculation is cheap. The selected trajectory should still be coalesced to at most one rebuild per rendered frame. Real Time stepping must not rebuild a complete future trajectory merely to update the target or moving marker.

## Real Time simulation mode

Real Time mode is an application playback mode over the same active dynamics and validated numerical stepping used by ordinary trajectories. It must not introduce a frame-rate-dependent or visually convenient replacement equation.

### Modes and state

Provide an explicit mode selector:

- `Static trajectory`
- `Real Time`

The Real Time controller owns:

- live simplex state
- seed state used by Reset
- running/paused state
- playback-speed multiplier
- accumulated unintegrated simulation time
- elapsed simulated time
- latest wall-clock timestamp
- error/behind-realtime status

Recommended mode transitions:

- Entering Real Time copies the current static starting state into the live state and seed state, resets elapsed simulated time and the accumulator, and starts paused.
- Starting resumes from the current live state; it does not reset the state.
- Pausing freezes the live state while UI, inspection, and rendering continue.
- Reset restores the Real Time seed state, clears elapsed simulated time and the accumulator, and pauses.
- Exiting Real Time adopts the current live state as the static starting state and rebuilds the ordinary projected trajectory from that state.

`Reset` is not one of the minimum requested controls, but it is required for repeatable experiments and should accompany Start/Pause.

### Click and drag behavior

- In Real Time mode, an ordinary click inside the simplex replaces both the live state and Reset seed with the clicked state and resets elapsed simulated time.
- If playback was running, it continues from the new state on the next update; if paused, it remains paused.
- Dragging continuously reseeds the live state using the existing clamped simplex interaction.
- Do not advance simulation steps while the pointer is actively dragging. Resume after release only if playback was running before or remains marked running.
- Shift+click retains priority and selects an equilibrium; it never reseeds the live state.
- ImGui mouse capture retains priority over both behaviors.

### Time and speed semantics

Use a monotonic clock such as `std::chrono::steady_clock` or SDL's monotonic high-resolution time. Rendering frame duration must not be used directly as the numerical integration step.

Define:

```text
1.0x playback = 1 model time unit per real second
simulatedDelta = realDelta * speedMultiplier
```

Use a fixed-step accumulator:

```text
accumulator += simulatedDelta
while accumulator >= fixedIntegrationStep:
    liveState = AdvanceOneValidatedStep(liveState, fixedIntegrationStep)
    accumulator -= fixedIntegrationStep
    elapsedSimulationTime += fixedIntegrationStep
```

The speed multiplier changes how quickly simulated time is consumed, not `fixedIntegrationStep`. Therefore changing speed must not change the numerical trajectory at the same simulated time.

The initial speed control should be logarithmic around `1.0x`. Final minimum and maximum values must be based on profiling rather than guessed large ranges.

### Numerical stepping and errors

- Reuse the validated RK4 stage checking and subdivision behavior from `SimplexTrajectoryIntegrator`.
- Expose a focused `AdvanceOneStep`/`AdvanceForDuration` API or a shared stepper; do not copy the private RK4 implementation into the playback controller.
- A failed step pauses playback, retains the last valid state, and presents an explicit error.
- Never normalize or project a materially invalid stage back into the simplex.
- Logit, Replicator, and continuous equal-split Best Response must use the same numerical semantics in batch and Real Time modes.
- For identical initial state, parameters, model, fixed step, and elapsed simulated time, Real Time and batch integration must agree within the existing integration tolerance.

### Frame stalls, focus changes, and catch-up

Prevent the classic fixed-step `spiral of death` without silently corrupting simulated time:

- reset the wall-clock baseline after entering the mode, resuming, window focus restoration, or a long suspension
- cap work per rendered frame using a measured maximum substep budget
- accept at most the amount of new wall-clock time that can be converted into that substep budget; excess wall-clock time is not declared to be simulated time
- if playback cannot keep up with the requested wall-clock rate, slow behind real time and display a `Playback cannot keep pace` status rather than skipping simulation states or enlarging the numerical step
- never perform a huge catch-up jump after the window was minimized, dragged, blocked by a breakpoint, or unfocused for a long time

Whether focus loss automatically pauses playback or merely resets the clock baseline can be finalized during Windows acceptance testing. It must never create a large state jump.

### Real Time UI and rendering

Controls:

- mode selector
- Start/Pause button
- Reset button
- playback-speed multiplier
- current simulated time
- running/paused/behind/error status

Rendering behavior:

- the orange marker represents the moving live state in Real Time mode
- the coordinate display reads the live state
- the purple Logit/Best Response target follows the live state
- Replicator continues to hide the target
- heat map, streamline field, equilibria, and sweeps remain cached because they do not depend on the live state
- hide the full precomputed selected trajectory in Real Time mode to avoid presenting a stale or confusing future path
- an optional short live-history trail is deferred; the required feature is the moving point itself

Changing OPGG parameters, dynamics kind, or a dynamics-specific setting while running should apply transactionally, keep the current valid live state, clear the accumulator, and auto-pause. This avoids combining an old time backlog with a new vector field.

## Interaction and Shift+click

Normal interaction:

- left-button press inside the simplex starts state dragging
- motion updates a pending state
- dragging outside clamps to the nearest simplex border
- release ends dragging

The ordinary click/drag action targets the static starting state in Static mode and the live/seed state in Real Time mode. The interaction controller should resolve the active behavior from the playback mode rather than duplicating two separate SDL event handlers.

Equilibrium inspection:

- Shift+left-click never starts a drag
- query `SDL_GetModState()` on button-down and test `SDL_KMOD_SHIFT`
- only current-parameter, currently visible equilibrium markers are selectable initially
- equilibrium-sweep samples are not selectable because they belong to different parameter values
- choose the nearest marker within a DPI-adjusted window-space radius
- Shift+clicking empty space clears the selection
- parameter or dynamics changes clear stale selection
- ImGui mouse capture takes precedence

Move the current window/NDC mapping helpers into a reusable `SimplexViewportTransform` so drawing and hit-testing share one authoritative transform and DPI scale.

If multiple equilibrium markers overlap, the nearest marker wins; the inspector should also list all current equilibria so the user has a non-pointer selection path.

## Equilibrium discovery

Use a model-aware equilibrium service rather than pretending one algorithm covers every dynamic.

### Logit

- retain the verified multi-seed damped-Newton interior search
- preserve residual verification and deduplication

### Replicator

- verify all three vertices
- search the interior with the existing two-dimensional method
- search each open edge as a one-dimensional rest-point problem
- verify the full three-component derivative at every candidate
- deduplicate results across faces
- be prepared to report a degenerate/non-isolated equilibrium set rather than inventing one discrete point

### Best Response

- use the declared equal-split selection convention
- do not use an ordinary smooth Newton solver across switching surfaces
- detect strict pure best-response rest points separately
- handle tie candidates through payoff-equality/support checks
- represent unavailable or nonsmooth stability analysis explicitly

## Equilibrium inspector and local stability

The inspector displays:

- active dynamics
- equilibrium index/type
- `x`, `y`, and `z`
- numerical residual
- Jacobian status
- reduced Jacobian matrix
- eigenvalues
- eigenvectors
- local classification
- warnings about nonsmooth, boundary, repeated, defective, or inconclusive cases

Use the reduced chart:

```text
J_xy with z = 1 - x - y
```

Label this basis in the UI. Convert each two-dimensional eigenvector into the full tangent direction:

```text
(vx, vy, -vx - vy)
```

For interior smooth points:

- use central finite differences
- calculate with step `h` and `h/2`
- warn or reject if the result is not numerically stable

For smooth boundary points:

- use two independent feasible one-sided perturbation directions
- recover the local linear operator in a consistent tangent basis
- label the classification as simplex-constrained

For Best Response tie surfaces:

- report `Classical Jacobian undefined at this switching surface`
- do not fabricate eigenpairs from a stencil that crosses the discontinuity

Classifications should include:

- attracting node
- repelling node
- saddle
- spiral sink
- spiral source
- center-like
- non-hyperbolic/inconclusive

Use a tolerance scaled to the matrix norm when testing eigenvalue real parts.

For real eigenvectors, draw short bidirectional eigendirection arrows centered on the selected equilibrium. Convert them through `SimplexMapper` and normalize their display length in pixels. Complex eigenvectors remain textual; do not draw them as ordinary real direction arrows.

## Speed heat map

### Scientific quantity

At every sampled population state, evaluate:

```text
speed = sqrt(dx*dx + dy*dy + dz*dz)
```

This population-space norm is independent of the screen layout.

### Sampling

For barycentric subdivision resolution `N`:

```text
sampleCount = (N + 1) * (N + 2) / 2
triangleCount = N * N
```

Include every edge and vertex. Invalid or non-finite dynamic evaluations are generation failures and must not silently become a color.

### Palette and shader

The reference palette is:

```text
dark blue -> cyan -> green -> yellow -> orange -> red
```

The mesh should store position plus normalized scalar speed. A dedicated heat-map shader or one-dimensional color-ramp lookup should apply the palette after scalar interpolation. This avoids interpolating already-mapped RGB values across triangles.

Display a numeric legend with the actual minimum and maximum speed.

Default normalization:

```text
Relative speed over the currently displayed model and parameters
```

Also support a locked numeric range for meaningful repeated comparisons. Independently normalized maps must never be presented as having directly comparable colors.

For hard Best Response, do not smoothly blend across triangles whose vertices belong to different best-reply regions as if the field were continuous. Mark region membership during sampling and subdivide or flat-shade mixed-region cells sufficiently to preserve switching boundaries.

Heat-map controls:

- visibility
- resolution
- relative or locked normalization
- locked minimum/maximum when applicable

Heat-map changes must not rebuild trajectories or equilibria.

## Streamline field and arrows

This layer is a streamline phase portrait, not a straight quiver grid and not a collection of unrelated full trajectories uploaded as one line strip.

### Generation

- deterministically seed the simplex in barycentric coordinates
- include seeds on all borders and vertices
- integrate forward using the active model and field integration settings
- use the density setting to control seed spacing and screen-space occupancy
- use the field trajectory time to control maximum streamline duration
- keep the field timestep separate and validated
- resample generated paths by visual arc length for stable rendering
- suppress or terminate paths that enter an already occupied screen-space region
- never allow the occupancy algorithm to create nondeterministic flicker between identical regenerations

### Arrows

- draw one arrowhead near the downstream endpoint of each nondegenerate streamline
- calculate direction from the last sufficiently long segment
- keep arrow size fixed in screen pixels
- do not encode speed in arrow size or line length; speed is encoded by the heat map
- render no arbitrary arrow for a zero-length/rest-point path

### Boundary semantics

- Replicator paths seeded on an invariant edge remain on that edge
- Logit and Best Response paths may leave a border and move inward
- all generated states must remain valid simplex states

### Rendering

Create a dedicated batched streamline mesh. Store independent line segments as `GL_LINES` or another representation that cannot connect separate paths. Render arrowhead triangles separately or as a second range in the same owned buffers.

Use dark/black field lines to match the screenshot. Add a subtle light halo if required for contrast over dark-blue heat-map areas. The user-selected trajectory should remain visually distinct from the background field.

Streamline controls:

- visibility
- density
- integration time
- integration timestep
- arrow size, if needed after visual testing

Regenerate only when the dynamics, game parameters, streamline density, integration time, or field timestep changes.

## Rendering order

1. Heat-map surface, or neutral simplex fill when disabled
2. Background streamline curves and arrowheads
3. Highlighted user-selected trajectory in Static mode; Real Time hides this full future path
4. Equilibrium sweep paths and samples, when applicable
5. Selected real eigendirection overlays
6. Current equilibrium markers
7. Selected-equilibrium highlight
8. Purple response-target ring for the static or live state, when the active dynamics defines a target
9. Orange static starting-state or moving Real Time marker
10. Simplex outline and coordinate labels
11. ImGui

Exact placement of the outline relative to markers may be adjusted after visual testing, but the outline must remain readable and must not cover interior analysis glyphs.

## State invalidation and performance

Use explicit dirty flags or versioned cached products.

| Change | Rebuild |
| --- | --- |
| Starting-state drag | selected trajectory, orange marker, purple target |
| Accepted Real Time step | live marker, coordinates, applicable purple target, optional status only |
| Real Time click/drag reseed | live/seed state, marker, target, elapsed time and accumulator |
| Start/Pause/speed change | playback controller state only; speed does not change numerical timestep |
| Enter Real Time mode | initialize paused live playback from the static state; hide full future trajectory |
| Exit Real Time mode | adopt live state as static start and rebuild its selected trajectory |
| Dynamics change | active model and every field-derived product |
| OPGG parameter change | trajectory, target, equilibria, heat map, streamlines, relevant sweeps |
| Logit eta change | all Logit-derived products |
| Heat-map resolution/range | heat map only |
| Streamline density/time/step | streamline field only |
| Equilibrium selection | inspector and selection/eigenvector overlay only |
| Window resize or DPI change | viewport transform and pixel-sized geometry only; do not reintegrate |

During dragging, store the latest pending mouse-derived state and apply it once after the event queue is drained. Do not integrate a full selected trajectory for every queued SDL motion event. In Real Time mode, do not advance simulation while actively dragging and do not rebuild a future trajectory on live steps.

Start with synchronous pure-CPU generation triggered after slider release or a short debounce. Add a cancellable `std::jthread` generation job only if profiling proves that supported settings cause unacceptable UI stalls. OpenGL buffer upload remains on the main thread.

Set final slider limits from measured cost rather than arbitrary large values.

## Recommended ownership

Mathematical/domain layer:

- `OpggPayoffEvaluator`
- `LogitDynamics`
- `ReplicatorDynamics`
- `BestResponseDynamics`
- active-dynamics factory/capabilities
- `SimplexTrajectoryIntegrator`
- model-aware equilibrium service
- `SimplexLinearizationAnalyzer`
- `SpeedFieldSampler`
- `StreamlineFieldGenerator`

Application layer:

- `SimulationSession`
- `RealTimeSimulationController`
- `SimplexInteractionController`
- cached visualization state and dirty flags

Presentation layer:

- `SimplexViewportTransform`
- `SimplexSceneRenderer`
- `SpeedHeatMapMesh`
- `StreamlineFieldMesh`
- eigendirection/selection marker mesh or renderer
- ImGui controls and equilibrium inspector

Avoid introducing a broad engine framework. Extract only responsibilities required for clear ownership, testing, and the requested behavior.

## Implementation sequence and gates

### Phase 1: Shared model foundation

- extract shared OPGG payoff evaluation
- preserve current Logit numerical output
- introduce dynamics kind/capabilities
- make `SimulationSession` own the active model transactionally
- update project and test build files

Gate: existing scientific-core tests pass and reference Logit derivatives/trajectories/equilibria remain unchanged within tolerance.

### Phase 2: New dynamics and target marker

- implement Replicator
- implement equal-split Best Response
- add dynamics selector and model-specific UI controls
- add optional response target and purple ring
- coalesce drag updates once per frame

Gate: derivatives are finite/tangent across a lattice; Replicator preserves faces; Best Response tie targets sum to one; switching is transactional.

### Phase 3: Real Time playback

- expose/reuse one-step validated integration without duplicating RK4
- add the Static/Real Time mode selector
- implement Start, Pause, Reset, fixed-step accumulation, playback speed, and simulated-time status
- implement click/drag reseeding and Shift+click priority
- update the moving marker, coordinates, and applicable target without rebuilding field-derived layers
- handle stalls, focus restoration, step failures, and model/parameter changes explicitly

Gate: paused playback never advances; results are frame-rate independent; speed changes do not change the state at equal simulated time; batch and Real Time integration agree; no large catch-up jump occurs after a suspended window; stepping failures pause at the last valid state.

### Phase 4: Equilibria and inspector

- add model-aware equilibrium discovery
- extract/rebuild reusable local linearization
- add Shift+click hit-testing
- add inspector data and selected marker
- add real eigendirection overlays

Gate: every displayed equilibrium is residual-verified; boundary points are covered for Replicator; known Jacobian/eigenpair tests pass; Best Response tie analysis is rejected explicitly.

### Phase 5: Speed heat map

- implement CPU sampler and indexed triangle mesh
- add scalar heat-map shader/palette
- add resolution, visibility, legend, and normalization controls
- handle Best Response region discontinuities

Gate: sample/triangle counts are exact; borders are covered; colors and legend match numeric speed; no NaN/invalid GPU input; the visual resembles the screenshot reference.

### Phase 6: Streamlines and arrows

- implement deterministic seeding and occupancy
- integrate and arc-length-resample paths
- implement batched independent lines and arrowheads
- add density/time/step controls

Gate: no unrelated paths connect; borders are seeded; arrows point downstream; rest points have no bogus arrow; output is deterministic and visually matches the reference.

### Phase 7: Integration and acceptance

- profile maximum supported resolution and density
- tune visual hierarchy and contrast
- complete Debug and Release x64 builds
- run scientific-core tests
- complete Windows, resize, and high-DPI manual acceptance
- update `README.md`, `SCIENTIFIC_VALIDATION.md`, and the Windows checklist for the implemented behavior

Gate: no scientific regression, no stale visualization after state changes, no interaction conflict, and acceptable UI responsiveness.

## Automated test requirements

### Shared payoff and dynamics

- extracted payoffs reproduce current Logit derivatives
- all dynamics are finite and tangent over a simplex lattice
- invalid parameters fail explicitly
- Replicator preserves zero coordinates and pure vertices
- Best Response unique winners and two-way/three-way ties are tested with scale-aware tolerances

### Session and invalidation

- failed dynamics/parameter/settings updates are transactional
- switching dynamics preserves common game parameters
- model-private settings are restored when switching back
- cached products are invalidated by the correct triggers only

### Real Time playback

- entering Real Time initializes from the static state and starts paused
- paused updates never advance the live state or simulated time
- Start resumes without resetting the current live state
- Reset restores the seed, clears accumulated/elapsed time, and pauses
- click/drag reseeding resets elapsed time and preserves the declared running/paused behavior
- Shift+click selects an equilibrium without reseeding
- a fixed simulated duration split across different rendering frame durations reaches the same state
- changing playback speed changes wall-clock completion time but not the state at equal simulated time
- Real Time and batch integration agree for the same model, settings, initial state, fixed step, and simulated duration
- changing dynamics/parameters while running applies transactionally, clears the accumulator, and pauses
- focus restoration or an artificial long frame does not cause a large catch-up step
- exceeding the measured per-frame substep budget produces an explicit behind-realtime status rather than skipping state silently
- a failed numerical step pauses and preserves the last valid state
- heat map, streamlines, equilibria, and sweeps are not invalidated by ordinary live steps

### Equilibria and stability

- interior, edge, and vertex equilibrium cases
- residual verification and deduplication
- known real, complex, repeated, and defective matrices
- verify each reported eigenpair through `J*v approximately lambda*v`
- finite-difference convergence from `h` to `h/2`
- explicit unavailable result at Best Response switching surfaces

### Heat map

- exact barycentric sample and triangle counts
- borders and vertices included
- finite speed values
- zero/minimum maps to the low end and maximum to red
- relative and locked normalization behavior
- discontinuity-region metadata for Best Response

### Streamlines

- deterministic seed count and placement
- borders included
- every path remains on the simplex
- Replicator edge invariance
- independent paths never connect in generated render geometry
- endpoint arrow direction and zero-length suppression
- occupancy behavior remains deterministic

### Interaction and transforms

- window/NDC/state round trips
- DPI-adjusted equilibrium hit-testing
- nearest-marker selection
- Shift+click does not move the starting state
- normal drag does not select equilibria
- selection invalidates when the active model changes

## Manual acceptance requirements

- Compare the heat map and streamline layer directly with the screenshot reference.
- Confirm the complete simplex, including all three borders, is colored and seeded.
- Confirm blue/cyan slow pockets and red fast regions respond correctly to parameter changes.
- Confirm arrows remain readable over every heat-map color.
- Confirm changing heat-map resolution changes detail without changing unrelated layers.
- Confirm changing streamline density/time changes only the field layer.
- Confirm regular dragging remains smooth.
- Confirm entering Real Time starts paused from the selected static state.
- Confirm Start moves the orange point continuously and Pause freezes it exactly where it is.
- Confirm Reset returns to the most recently clicked/dragged Real Time seed.
- Confirm clicking and dragging reseed the live point correctly while Shift+click still selects equilibria.
- Confirm the speed control changes playback rate without changing the path at the same simulated time.
- Confirm coordinates and the purple Logit/Best Response target update as the live point moves.
- Confirm Replicator Real Time playback remains on invariant borders when seeded there.
- Confirm heat map, streamlines, and equilibrium markers do not regenerate or flicker during playback.
- Confirm leaving Real Time adopts the live state as the static starting state and rebuilds one projected trajectory.
- Confirm minimizing, moving, suspending, or refocusing the window does not produce a large catch-up jump.
- Confirm numerical failure or inability to keep pace is visible and never silently skips simulation state.
- Confirm the purple target follows the dragged state in Logit and Best Response modes.
- Confirm Replicator hides the target without displaying stale data.
- Confirm Shift+click selects an equilibrium without moving the orange point.
- Confirm the inspector reports coordinates, residual, Jacobian, eigenpairs, and classification correctly.
- Confirm complex, boundary, and nonsmooth cases are described honestly.
- Confirm rendering and hit-testing remain aligned during resize and at common Windows DPI scales.
- Build and run both Debug and Release x64 configurations.

## Deliberately deferred or optional work

- Exact discrete Best Response trajectory mode using the paper's `epsilon` update
- Clicking parameter-sweep samples with their sampled parameter context
- Background-thread field generation, unless profiling requires it
- Alternative colorblind/perceptually uniform palettes; the requested reference palette remains the default
- Repeated arrowheads on very long streamlines; the initial requirement is one downstream endpoint arrow
- General continuous equilibrium-set rendering beyond what the selected dynamics require
- Optional short/fading Real Time history trail; the required Real Time visualization is the moving point

## Important failure modes to avoid

- Do not conflate the current Logit update rule with Replicator dynamics.
- Do not reuse private payoff formulas independently in three model classes.
- Do not run a smooth Newton/Jacobian calculation across a hard Best Response tie surface.
- Do not report only interior Replicator equilibria.
- Do not interpolate Best Response speed across switching boundaries without acknowledging the discontinuity.
- Do not compare independently normalized heat-map colors as absolute speeds.
- Do not rebuild the heat map or streamline field while the starting point is dragged.
- Do not upload multiple streamlines as one `GL_LINE_STRIP`.
- Do not let slider interaction continuously trigger unbounded expensive regeneration.
- Do not preserve an equilibrium selection after the model or parameters that produced it have changed.
- Do not display a fabricated Replicator response target.
- Do not use raw rendering-frame duration as an RK4 timestep.
- Do not implement playback speed by enlarging the numerical integration step.
- Do not rebuild a complete future trajectory on every Real Time step.
- Do not let Real Time playback advance while paused or while the state is actively dragged.
- Do not apply a large accumulated catch-up interval after minimize, focus loss, a breakpoint, or another stall.
- Do not silently discard simulated time or skip states when playback cannot keep up; report the condition.
- Do not duplicate the validated RK4 implementation in the Real Time controller.
- Do not claim a build, numerical result, or visual match is verified until the corresponding automated and manual checks have run.
