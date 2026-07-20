# SimplexBeast Research Workbench Implementation Plan

Date: 2026-07-20

Status: Planning only; implementation has not begun

Scope: Scientific features 1-9 and interactive additions 1-12

## 1. Purpose

This document is the durable implementation plan for expanding SimplexBeast from a simplex dynamics visualizer into a reproducible scientific workbench. It records the intended behavior, architecture, numerical semantics, interaction design, implementation order, and QA gates so future coding sessions do not have to reconstruct these decisions.

The additions should be designed as one coherent system. Reproducibility, timeline data, numerical methods, analysis jobs, rendering, and interaction tools must share typed inputs and results rather than becoming isolated controls attached directly to the application class.

This plan does not authorize implementation by itself. Before each milestone, inspect the current source and adjust file names and boundaries to the code as it exists at that time.

## 2. Current Architectural Pressure Points

The current application already contains substantial working behavior, but the next set of features will put pressure on three areas:

- `SimplexBeastApplication.cpp` is the orchestration hub and directly owns much of the UI, input handling, mesh management, and result invalidation. Adding every feature there would create unclear ownership and brittle state transitions.
- `SimplexDynamicModel` is centered on continuous derivatives. Exact discrete best response requires a separate mathematical contract rather than pretending a map is an ordinary differential equation.
- Trajectories are primarily represented as state samples. Time-series inspection, export, replay, diagnostics, branching, and provenance require a richer typed result.

The first implementation work should therefore establish data and ownership boundaries while preserving the current visual and numerical behavior.

## 3. Target Architecture

The intended data flow is:

```text
UI panels and interaction tools
              |
              v
     WorkspaceController
              |
              v
      ExperimentDocument
              |
              v
immutable ExperimentSnapshot + ExperimentRevision
              |
              +-------------------------------+
              |                               |
              v                               v
       Evolution factory              AnalysisJobManager
        /             \               /       |        \
continuous-flow    discrete-map  continuation basins   periodic orbits
     engine           engine          atlas    fields   and contours
        \             /                    \    |    /
         SimulationTimeline             typed results
                  \                       /
                   v                     v
                       SceneState
                           |
                           v
                 SceneRenderer and UI
                           |
                           v
                    Export services
```

The application should distinguish scientific state from presentation state:

- `ExperimentDefinition`: user-selected scientific inputs, including the model, parameters, evolution rule, solver settings, initial conditions, and requested analyses.
- `WorkspaceViewState`: camera, selected tab, tool selection, plot ranges, visibility toggles, and other presentation-only state.
- `ExperimentSnapshot`: an immutable, copyable input passed to simulation or analysis work.
- `ExperimentRevision`: a monotonically increasing identifier changed whenever a scientific input changes.
- `ModelFingerprint`: a stable description or hash of all inputs needed to identify the model and numerical configuration used for a result.
- `AnalysisOutcome<T>`: a result wrapper whose status is `complete`, `partial`, `cancelled`, or `failed`, with warnings and diagnostic information.

The evolution abstraction must explicitly separate two mathematical categories:

- `ContinuousSimplexDynamics`, representing `ds/dt = f(s)`.
- `DiscreteSimplexMap`, representing `s[k + 1] = M(s[k])`.

Shared capability metadata should state whether an evolution rule has continuous time, a derivative, a discrete map, a target state, smooth derivatives, adaptive-integration support, equilibrium support, and a well-defined local stability analysis. UI and analysis tools should use these capabilities instead of relying on model-name conditionals.

## 4. Core Data Structures

### 4.1 Simulation samples and trajectory results

Introduce a `SimulationSample` that can hold:

- continuous time or discrete iteration;
- simplex state `(x, y, z)`;
- strategy payoffs and mean payoff where applicable;
- derivative for continuous dynamics or step displacement for a map;
- target state where the evolution rule defines one;
- speed or displacement magnitude;
- distance to a selected or classified equilibrium;
- simplex-invariant and boundary diagnostics.

Introduce a `TrajectoryResult` containing:

- ordered `SimulationSample` values;
- the experiment revision and model fingerprint;
- solver or iterator diagnostics;
- termination reason;
- warnings;
- complete/partial/cancelled/failed status;
- provenance needed for export and comparison.

### 4.2 Timeline

Introduce a `SimulationTimeline` with segmented history and an explicit cursor. It must distinguish:

- the simulation head, which is the newest computed sample;
- the viewing cursor, which may inspect earlier samples;
- branches or segments created after a reseed, parameter change, model change, or resume from historical state.

Moving backward through the timeline is playback of recorded data, not reverse numerical integration. Resuming from a historical point must explicitly create a new branch or truncate the future after confirmation; it must never silently reinterpret old samples as belonging to a changed experiment.

### 4.3 Background analysis

Introduce an `AnalysisJobManager` using cancellable `std::jthread` jobs. Every job must receive:

- an immutable experiment snapshot;
- the revision on which it is based;
- typed settings;
- a stop token;
- deterministic work ordering or a recorded random seed.

Workers may produce CPU-side results or progress chunks, but OpenGL resources must be created and updated on the main thread. Results from a stale revision must be discarded unless the user deliberately pinned them for comparison.

Start with one heavy analysis worker. Add greater concurrency only after profiling memory bandwidth, frame-time impact, and cancellation behavior.

## 5. Scientific Features 1-9

### 5.1 Reproducible experiments and exports

#### Goal

Allow an experiment to be saved, reopened, rerun, inspected, and shared without relying on undocumented UI state.

#### Proposed components

- `ExperimentDocument`
- `ExperimentSerializer`
- `ResultExportService`
- `FileDialogService`
- `BuildProvenance`

#### Experiment format

Use a versioned `.simplexbeast.json` document containing at least:

- schema version;
- application version and source commit when available;
- OPGG and other model parameters;
- evolution rule and all rule-specific settings;
- initial state;
- continuous solver or discrete iterator settings;
- tolerances, event thresholds, and stopping criteria;
- requested analyses and their settings;
- normalization and coordinate conventions;
- deterministic seeds where randomness is used.

Use `nlohmann-json` for serialization unless the dependency review identifies a stronger project-specific reason not to. Parse into a temporary document, validate the complete schema and semantic constraints, and apply it transactionally only after validation succeeds.

Use `std::numeric_limits<double>::max_digits10` and a fixed locale for numeric round trips. Save atomically through a temporary file followed by a replacement operation so an interrupted write does not corrupt the last valid experiment.

SDL file dialogs are asynchronous and their callbacks may not run on the main thread. Dialog callbacks should enqueue a small result for main-thread consumption rather than mutate application or OpenGL state directly.

#### Result bundle

Support a directory or archive-like result bundle containing the relevant subset of:

- `experiment.json`
- `trajectory.csv`
- `equilibria.csv`
- `continuation.csv`
- `basin.csv`
- `periodic_orbits.csv`
- `manifest.json`
- `figure.png`

The manifest must state which files are present, how they were produced, the experiment fingerprint, completion state, warnings, and provenance. A partial or cancelled analysis may be exported only when it is clearly marked incomplete.

#### QA gates

- Saving and loading an experiment preserves all scientific inputs within the documented representation.
- Unsupported or malformed schema versions fail without changing the active experiment.
- A fresh process can reproduce the same deterministic outputs within declared numerical tolerances.
- Every exported result includes sufficient model and numerical provenance.

### 5.2 Time-series diagnostics

#### Goal

Make simulation behavior inspectable through synchronized numerical plots rather than only through motion on the simplex.

#### Proposed components

- `TimelinePanel`
- the richer `SimulationSample`, `TrajectoryResult`, and `SimulationTimeline` types
- a deterministic plot-decimation helper that preserves raw samples

#### Plots and values

The panel should be able to show:

- `x`, `y`, and `z` over time or iteration;
- individual strategy payoffs and mean payoff;
- derivative norm for continuous dynamics or step-displacement norm for maps;
- distance to a selected equilibrium;
- target coordinates when a target exists;
- solver step size, rejected steps, or other diagnostics when useful.

The timeline cursor and simplex marker must remain synchronized. The plot may decimate data for rendering, but inspection and export must always use the original samples.

ImPlot is a strong fit for interactive inspection. Treat it as an interactive plotting dependency, not as the publication-quality export path; publication figures should use a separate deterministic renderer or exporter.

#### QA gates

- Plot cursor, simplex state, and displayed diagnostics refer to the same sample.
- Reseeding or changing scientific inputs creates an explicit timeline segment.
- Playback never invokes backward integration.
- Decimation does not alter exported raw data or reported extrema.

### 5.3 Numerical accuracy mode

#### Goal

Provide controlled-error trajectory computation for scientific analysis while preserving the predictable behavior of the current real-time animation.

#### Proposed components

- `ContinuousTrajectorySolver`
- `FixedRk4Solver`
- `ControlledDopri5Solver`
- event and termination types shared by solvers

#### Behavior

Static trajectory generation should gain an adaptive Dormand-Prince 5(4) option with relative and absolute tolerances. Boost.Odeint is the leading dependency candidate, subject to a focused integration review.

The first adaptive implementation should target smooth Logit and Replicator dynamics. Keep fixed-step RK4 for Real Time mode and for streamline generation initially, because fixed frame pacing and reproducible visual sampling are different requirements from controlled-error offline integration.

Record:

- accepted and rejected step counts;
- right-hand-side evaluation count;
- minimum and maximum accepted step;
- estimated local error information where exposed;
- termination reason and event information.

Supported events should include:

- arrival near an equilibrium with a dwell requirement;
- simplex-boundary contact or invalid-state detection;
- payoff or response switching surfaces where mathematically meaningful;
- Poincare-section crossings;
- requested final time.

If an intermediate adaptive stage is invalid, the solver should reduce its step or report a numerical failure. Projection back into the simplex must not silently replace error control, because projection can change the mathematical trajectory.

#### QA gates

- Analytic benchmark systems demonstrate the expected fixed-step order and adaptive tolerance behavior.
- Tightening tolerances reduces measured global error on smooth benchmarks.
- Dense-output sampling agrees with direct integration within declared tolerance.
- Invariant simplex faces remain invariant within numerical tolerance.
- Invalid intermediate stages trigger controlled recovery or an explicit failure.

### 5.4 Exact discrete Best Response

#### Goal

Offer genuine iterative best-response dynamics as a discrete alternative, clearly separated from continuous best-response-like flow.

#### Mathematical contract

Use a configurable relaxed update such as:

```text
M(s) = (1 - epsilon) s + epsilon B(s),    0 < epsilon <= 1
```

where `B(s)` is the selected best-response target and tie behavior is explicitly defined.

#### Proposed component

- `BestResponseDiscreteMap`

Expose:

- relaxation `epsilon`;
- iteration count;
- iterations per second in Real Time mode;
- single-step control;
- deterministic tie-breaking or a clearly selected tie correspondence policy.

Equilibria satisfy `M(s) = s`. Where the map is differentiable, local stability is determined by multiplier magnitudes `|mu| < 1`, not by continuous-time eigenvalue real parts. On switching surfaces, classical linearization may be unavailable and the UI must say so.

For discrete dynamics:

- label the heat field as step displacement rather than speed;
- render iteration paths rather than continuous streamlines;
- show `B(s)` with the purple target ring;
- optionally show `M(s)` with a ghost next-state marker.

The existing continuous best-response option, if retained, must remain separately named and described.

#### QA gates

- One-step results agree exactly with the map definition and tie policy.
- `epsilon = 1` reaches the selected best-response target in one update.
- Stability labels use discrete multipliers.
- Continuous-only controls and terminology are disabled or relabeled.

### 5.5 Nullclines and payoff-indifference contours

#### Goal

Reveal the geometric structure responsible for equilibria, switching, and motion.

#### Proposed components

- `SimplexLattice`
- `SimplexScalarFieldSampler`
- `MarchingTrianglesContourExtractor`
- `ContourFieldResult`
- `ContourMesh`

Sample scalar fields on a shared triangular simplex lattice. Initial fields should include:

- `dx/dt = 0`, `dy/dt = 0`, and `dz/dt = 0` for continuous dynamics;
- pairwise payoff differences such as `u1 - u2 = 0`;
- discrete component-displacement zeros for maps.

Use marching triangles rather than rectangular marching squares. Handle exact-zero vertices, coincident edges, and duplicate segments deterministically. For nonsmooth best-response regions, sample a categorical region ID and avoid interpolating across a switching boundary as if the field were smooth.

#### QA gates

- Extracted contours match analytic lines on synthetic fields.
- Every generated point lies on or inside the simplex within tolerance.
- Degenerate vertex and edge cases are deterministic.
- Hovered contour values agree with a direct payoff or derivative evaluation.

### 5.6 True parameter continuation and bifurcation analysis

#### Goal

Trace equilibrium branches through parameter space, including folds, instead of presenting independent nearest-neighbor samples as continuation.

The existing `LogitEquilibriumSweep` should remain available but be labeled a sampled equilibrium sweep.

#### Proposed components

- `ContinuationParameter`
- `ContinuationSettings`
- `ContinuationEngine`
- `ContinuationBranch`
- `ContinuationEvent`
- `BifurcationPlotPanel`

#### Numerical method

Implement pseudo-arclength continuation for a reduced simplex equilibrium equation:

```text
F(x, y, p) = 0
```

augmented by an arclength constraint. Each accepted sample should contain:

- simplex state and parameter value;
- nonlinear residual;
- branch tangent;
- reduced Jacobian;
- eigenvalues or map multipliers;
- stability classification;
- corrector iteration count;
- warnings and event candidates.

Initial scope should be:

- interior Logit equilibria;
- interior Replicator equilibria;
- Replicator face equilibria through explicit face reductions;
- smooth discrete maps where the map Jacobian is available.

Nonsmooth best response should wait until a mathematically appropriate nonsmooth continuation design exists.

Detect and conservatively label:

- folds from tangent or augmented-system evidence;
- Hopf candidates from a complex pair crossing the imaginary axis;
- generic zero-eigenvalue events.

Do not claim a transcritical or pitchfork bifurcation from a zero eigenvalue alone. Those labels require branch-intersection and symmetry or normal-form evidence.

AUTO-07p is a useful reference for terminology and validation, but embedding it is not part of the initial implementation.

#### QA gates

- Canonical fold, transcritical, and Hopf benchmark systems are traced correctly.
- Forward and reverse branch traversal agree within tolerance.
- Every accepted branch point satisfies the declared residual tolerance.
- Event labels are backed by explicit numerical evidence and uncertainty is displayed.

### 5.7 Basin-of-attraction analysis

#### Goal

Classify long-term outcomes across the simplex without disguising unresolved or slow trajectories as verified attractors.

#### Proposed components

- `AttractorCatalog`
- `AttractorClassifier`
- `BasinAnalysisSettings`
- `BasinAnalyzer`
- `BasinResult`
- `CategoricalSimplexMesh`

Classification should combine:

- distance to a known attractor;
- local residual or step displacement;
- a dwell time or dwell-iteration condition;
- integration horizon;
- periodic-orbit matching when verified cycles exist;
- boundary behavior;
- timeout and numerical-failure detection.

Required result categories are:

- verified equilibrium;
- verified periodic orbit;
- classified boundary outcome;
- candidate unknown attractor;
- unresolved within horizon;
- numerical failure.

Do not assign a trajectory to the nearest equilibrium based only on its final sample.

The result should support categorical attractor coloring or convergence-time coloring, selectable grid refinement, and basin-fraction summaries. Unknown and failed regions must remain visually distinct.

#### QA gates

- Systems with known separatrices produce the expected basin partition under refinement.
- Shortening the horizon increases unresolved classifications rather than false certainty.
- Basin fractions report the resolution and unresolved fraction.
- Repeated deterministic runs produce the same cell classifications.

### 5.8 Two-parameter atlas

#### Goal

Explore how qualitative behavior changes over a two-dimensional parameter domain.

#### Proposed components

- `ParameterDescriptor`
- `ParameterAxis`
- `ParameterAtlasSettings`
- `ParameterAtlasGenerator`
- `ParameterAtlasResult`
- `ParameterAtlasPanel`

`ParameterDescriptor` should provide:

- stable ID and display label;
- continuous or integer type;
- mathematical domain and UI bounds;
- linear or logarithmic scale;
- transactional read/write access to an experiment definition;
- conditional validity constraints involving other parameters.

Possible cell metrics include:

- number of verified equilibria found;
- stability of a selected branch;
- largest real eigenvalue or largest multiplier magnitude;
- basin fraction for an attractor;
- convergence time;
- presence of a verified periodic orbit;
- failure or unresolved rate.

Computation should be progressive and cancellable. Invalid cells should be hatched or explicitly marked, not assigned a misleading numeric color. Clicking a cell should create a preview snapshot; it must not silently overwrite the active experiment.

#### QA gates

- Axis sampling obeys type, bounds, scale, and conditional constraints.
- Results are invariant to worker chunk order.
- Invalid, incomplete, unresolved, and failed cells are visually distinct.
- Loading a cell into the active experiment requires an explicit action.

### 5.9 Periodic-orbit detection and stability

#### Goal

Detect, refine, and inspect genuine recurrent behavior without confusing slow equilibrium convergence with a long-period cycle.

#### Proposed components

- `RecurrenceDetector`
- `PoincareSection`
- `PeriodicOrbitRefiner`
- `FloquetAnalyzer`

After an initial transient, candidate detection should use repeated same-direction Poincare crossings and recurrence checks. A candidate should then be refined with a shooting formulation or a return-map root solve.

For a verified orbit, store:

- sampled orbit;
- estimated period;
- closure residual;
- number and direction of section crossings;
- observed cycles;
- coordinate minima and maxima;
- monodromy or return-map derivative information;
- the nontrivial Floquet multiplier for the reduced planar flow;
- stability classification and warnings.

Use variational equations when an accurate smooth Jacobian is available; otherwise use carefully scaled finite differences of the return map. Initial scope should cover smooth continuous dynamics. Nonsmooth and discrete cycles need separate follow-up designs.

#### QA gates

- Known limit-cycle benchmarks are detected with convergent period and closure residual.
- Stable equilibrium approaches are rejected as cycle candidates.
- Section orientation prevents double counting.
- Stability estimates converge under tolerance and perturbation refinement.

## 6. Interactive Additions 1-12

### 6.1 Live target tether

Add a `VectorGlyphOverlay` connecting the active state to its target. For Logit and Best Response, the purple ring represents the actual response target. Replicator dynamics has no equivalent target distribution, so show a separately styled velocity arrow and label it as velocity rather than target.

### 6.2 Fading trajectory trail

Render recent trajectory segments with age-dependent alpha. Extend the line representation to carry RGBA values and enable OpenGL blending. Split the trail at experiment revisions, reseeds, and explicit timeline branches so unrelated states are never visually joined.

### 6.3 Playback timeline

Add a `TimelineController` with pause, play, single-step, scrub, replay, loop, and explicit branch-from-cursor behavior. Controls must operate on recorded history unless the simulation head is actively advancing.

### 6.4 Eigenvector perturbation lab

From a selected smooth equilibrium, let the user create feasible perturbations along real eigenvectors. Clamp the perturbation amplitude to remain in the simplex. For complex conjugate pairs, provide a radial or phase-oriented perturbation in the real invariant plane instead of presenting a complex vector as a directly drawable direction. Disable this tool when a nonsmooth model lacks a meaningful classical eigenbasis.

### 6.5 Particle brush

Add an `EnsembleSimulationController` and a brush tool that seeds many initial states in a local simplex neighborhood. Use a batched point or line mesh and the same numerical engine as ordinary trajectories. Store or display the deterministic random seed when randomized sampling is enabled.

### 6.6 Hover probe

Hovering over the simplex should show:

- simplex coordinates;
- payoffs and mean payoff;
- target, derivative, or map displacement;
- speed or displacement magnitude;
- response region;
- nearest verified equilibrium and distance.

Short preview trajectories may be generated after a debounce interval. They should be cheap, cancellable, and clearly distinguished from saved results. Let the user pin a probe for comparison.

### 6.7 Parameter morphing

Dragging a parameter while paused may produce a lightweight preview snapshot. During active simulation, distinguish two behaviors:

- an autonomous experiment where a changed parameter starts a new revision or segment;
- an explicitly time-dependent `ParameterProtocol` where the parameter is part of the mathematical forcing and is stored in every sample.

Never describe an ordinary mid-run UI edit as the same autonomous system. Results that have not yet been recomputed for the current value should display a stale badge.

### 6.8 Comparison race

Add a `ComparisonLane` abstraction so several evolution rules or parameter sets can run from the same seed. Continuous systems should share model time. Discrete systems should display iteration count and, if useful, a separately defined pseudotime; never imply physical time equivalence without a declared convention.

### 6.9 Interactive bifurcation explorer

Synchronize the continuation plot with the simplex preview. Hovering a branch point should preview its state and local stability. Clicking should select it; applying it to the active experiment requires a separate command. Stable and unstable branches must differ by line style or glyph as well as color.

### 6.10 Progressive basin painter

Compute deterministic coarse-to-fine basin chunks and upload completed mesh data on the main thread. Use gray or another neutral treatment for unresolved areas, a distinct failure pattern, and an explicit incomplete badge until the requested resolution has finished.

### 6.11 Periodic-orbit interaction

Selecting an orbit should highlight the loop, synchronize its time-series trace, show section crossings and stability, and offer a one-period camera or marker follow mode. The result panel should expose period and closure residual prominently.

### 6.12 Guided scenarios

Add versioned JSON scenario definitions containing:

- an initial experiment;
- instructions and optional hints;
- scientific observations or goals;
- predicates evaluated against typed model/results state;
- optional recommended visible panels and tools.

Scenarios should not depend on pixel coordinates or hard-coded UI layout. Avoid game-like scoring that rewards a scientifically misleading answer; completion predicates must refer to meaningful states or observations.

## 7. UI, Interaction, and Rendering Structure

### 7.1 Application decomposition

As the feature set grows, move responsibilities out of the application class into focused owners such as:

- `WorkspaceController`
- `SimplexInteractionController`
- `SceneRenderer`
- `ControlPanel`
- `TimelinePanel`
- `DiagnosticsPanel`
- `AnalysisPanel`
- `ExperimentPanel`
- `ScenarioPanel`

Do not split files merely to reduce line counts. Extract a component when it establishes clear ownership, testable state transitions, or a dependency boundary.

### 7.2 Canvas tools

Use an explicit canvas toolbar with mutually understandable modes:

- State
- Probe
- Particle Brush
- Perturb

Shift-click equilibrium selection should remain a global shortcut so it does not depend on the active canvas tool. Every mode and modifier should have visible help text and a pointer cursor or glyph that communicates its action.

### 7.3 Viewport transform

Extract a `SimplexViewportTransform` responsible for:

- barycentric/simplex state to screen coordinates;
- screen coordinates to feasible simplex state;
- hit testing and tolerance conversion;
- resize and DPI behavior.

This prevents hover probes, brushes, selections, contours, and screenshots from implementing subtly different coordinate conversions.

### 7.4 GPU resources

Likely new render resources include:

- `ColoredLineMesh` with per-vertex RGBA;
- `PointCloudMesh`;
- `VectorGlyphMesh`;
- `ContourMesh`;
- indexed scalar-field mesh;
- indexed categorical-field mesh;
- an offscreen framebuffer for deterministic figure export.

Keep scientific values on the CPU side in typed results. GPU buffers are disposable render caches, not the authoritative analysis result.

### 7.5 Plot dependency

If ImPlot is adopted, pin a commit compatible with the vendored ImGui 1.92.8 version and record it in the build system and provenance. Verify keyboard/mouse capture interactions with the simplex canvas before depending on it broadly.

## 8. Background Work and Responsiveness Rules

The following should run outside the main render loop once their cost exceeds a small synchronous budget:

- continuation;
- basin classification;
- two-parameter atlases;
- periodic-orbit searches;
- high-resolution scalar fields;
- large particle ensembles.

Every background result must follow these rules:

1. Copy an immutable snapshot rather than reading mutable UI state.
2. Record the source revision and settings.
3. Check the stop token at bounded intervals.
4. Produce deterministic ordering and record random seeds.
5. Report progress through a thread-safe queue or owned result channel.
6. Perform OpenGL allocation and upload on the main thread.
7. Reject stale results unless deliberately retained for comparison.
8. Mark cancelled or partial results accurately.

## 9. Implementation Milestones

### Milestone 0: Preserve behavior and establish boundaries

- With the user's approval, commit or tag the current verified implementation before structural work.
- Introduce a non-OpenGL `SimplexBeastCore` target for model, solver, analysis, serialization, and unit-testable types.
- Add experiment snapshots, revisions, outcomes, and fingerprints.
- Extract interaction/viewport logic, major panels, and rendering ownership incrementally.
- Split tests by responsibility while retaining an aggregate test command.
- Gate completion on existing tests, a successful build, and a visual comparison showing no unintended behavior change.

### Milestone 1: Reproducibility and observability

Implement:

- Scientific feature 1: reproducible experiments and exports.
- Scientific feature 2: time-series diagnostics.
- Interactive additions 1, 2, 3, and 6: live tether, fading trail, playback timeline, and hover probe.

This milestone creates the data model required by most later work and provides immediate scientific value.

### Milestone 2: Interactive experiments

Implement:

- Interactive addition 4: perturbation lab.
- Interactive addition 5: particle brush.
- Interactive addition 8: comparison race.

Keep ensemble and comparison work on the same solver and sample contracts as single trajectories.

### Milestone 3: Numerical and discrete evolution

Implement:

- Scientific feature 3: numerical accuracy mode.
- Scientific feature 4: exact discrete Best Response.

This milestone must finish the continuous-versus-discrete capability separation before downstream analyses assume stability or time semantics.

### Milestone 4: Explanatory geometry

Implement:

- Scientific feature 5: nullclines and payoff-indifference contours.
- The snapshot-preview portion of interactive addition 7: parameter morphing.

### Milestone 5: Continuation

Implement:

- Scientific feature 6: true continuation and bifurcation analysis.
- Interactive addition 9: bifurcation explorer.
- The continuation-aware portion of interactive addition 7.

### Milestone 6: Global outcome analysis

Implement:

- Scientific feature 7: basin analysis.
- Scientific feature 8: two-parameter atlas.
- Interactive addition 10: progressive basin painter.

### Milestone 7: Recurrent dynamics

Implement:

- Scientific feature 9: periodic-orbit detection and stability.
- Interactive addition 11: periodic-orbit interaction.

### Milestone 8: Guided workflow and presentation

Implement:

- Interactive addition 12: guided scenarios.
- comparison-workflow refinements;
- publication export;
- accessible palettes and patterns;
- a curated initial scenario set.

## 10. Testing Strategy

### 10.1 Unit tests

Unit-test:

- experiment validation and serialization round trips;
- fingerprints and revisions;
- simplex coordinate transforms and feasible perturbation bounds;
- continuous solver accuracy and events;
- discrete map updates and multipliers;
- contour extraction degeneracies;
- classifier decisions;
- continuation predictor/corrector steps;
- recurrence and crossing detection;
- parameter descriptor constraints.

### 10.2 Numerical benchmark tests

Maintain small canonical systems with known behavior for:

- exponential decay or another analytic ODE;
- invariant simplex faces;
- a fold branch;
- a transcritical branch intersection;
- a Hopf crossing;
- known basin separatrices;
- a stable limit cycle;
- a stable and unstable discrete fixed point.

Tests should assert convergence trends and residual bounds, not brittle platform-identical floating-point arrays unless exact identity is part of the contract.

### 10.3 Integration tests

Test complete workflows such as:

- save, close, load, rerun, compare fingerprint;
- real-time run, pause, scrub, branch, and export;
- change a parameter while an analysis job runs and confirm stale rejection;
- cancel a progressive basin or atlas job and export an accurately marked partial result;
- select an equilibrium, perturb along a valid eigenvector, and verify the initial displacement.

### 10.4 Visual QA

Perform visual checks for:

- DPI and window resizing;
- color and non-color category distinctions;
- dense arrows, contours, trails, and particles;
- hover and click hit regions near borders and vertices;
- plot/canvas input capture;
- unresolved, invalid, stale, partial, and failed result states;
- screenshot/export parity.

## 11. Non-Negotiable Scientific QA Rules

- Never mix continuous time and discrete iterations without explicit units or a declared mapping.
- Never use the continuous eigenvalue criterion for a discrete map; use multiplier magnitude.
- Never relabel an unresolved basin cell as an attractor.
- Never call a nearest-neighbor parameter sweep true continuation.
- Never interpolate nonsmooth Best Response regions as though they were a smooth vector field.
- Never call a UI parameter edit an autonomous trajectory when the parameter changed during the run; represent it as a new segment or an explicit protocol.
- Never commit a stale asynchronous result into the active scene.
- Never export a scientific result without experiment and numerical provenance.
- Never rely on color alone to distinguish stability, basin categories, validity, or completion state.
- Preserve the current fixed-step Real Time experience even after adaptive offline integration is added.

## 12. Key Design Decisions to Preserve

1. Scientific state and view state are separate.
2. Continuous flows and discrete maps have separate interfaces and stability semantics.
3. Raw samples remain authoritative even when plots and meshes are decimated.
4. Timeline history is segmented and revision-aware.
5. Background jobs consume immutable snapshots and cannot silently publish stale results.
6. A classification may report uncertainty, partial completion, or failure; the UI must not force every computation into a confident category.
7. The current sampled equilibrium sweep remains useful but is not renamed continuation.
8. Interactive features must expose real model quantities. Replicator velocity, Logit target, and Best Response target are not interchangeable concepts.
9. GPU resources are rendering caches; typed CPU results are the scientific record.
10. Each milestone must preserve buildability, existing behavior, and test coverage before the next one begins.

## 13. Dependencies and References to Evaluate

- [nlohmann-json parse documentation](https://nlohmann.github.io/json/api/basic_json/parse/)
- [SDL3 open-file dialog documentation](https://wiki.libsdl.org/SDL3/SDL_ShowOpenFileDialog)
- [ImPlot](https://github.com/epezent/implot)
- [Boost.Odeint overview](https://www.boost.org/doc/libs/latest/libs/numeric/odeint/doc/html/boost_numeric_odeint/getting_started/overview.html)
- [AUTO-07p](https://auto-07p.github.io/)

These are candidates and references, not blanket approvals to add dependencies. Before adding one, verify license, version compatibility, build integration, platform behavior, and whether the project already has an equivalent facility.

## 14. Definition of Completion

The roadmap is complete only when:

- each feature has a typed and documented mathematical contract;
- continuous and discrete modes use correct terminology and stability criteria;
- experiments and results can be saved with provenance and reproduced;
- long analyses are cancellable, revision-safe, deterministic where promised, and responsive;
- uncertainty and numerical failure remain visible;
- interaction tools and scientific plots are synchronized with the same underlying samples;
- automated numerical tests and manual visual QA cover the declared behavior;
- user documentation explains controls, scientific meaning, limitations, and export formats.
