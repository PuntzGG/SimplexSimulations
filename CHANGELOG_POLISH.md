# SimplexBeast implementation change log

Planning baseline: repository `main` at commit `e7805e1`.

## Product identity and build

- Renamed the Visual Studio solution/project/output to `SimplexBeast`.
- Added a vcpkg manifest for SDL3 and GLEW and project-local x64 dependency paths.
- Added every new source/header to the Visual Studio project and filters.
- Kept Debug as a console application for diagnostics and made Release a normal windowed application.

## Scientific model layer

- Extracted the stable OPGG payoff calculation into `OpggPayoffEvaluator`.
- Added transactional switching between Logit, Equal-split Best Response, and Replicator dynamics.
- Added scale-aware Best Response tie handling and explicit model capabilities.
- Preserved the stable participation polynomial and overflow-safe Logit softmax.

## Simulation and interaction

- Exposed the validated RK4 one-step operation for reuse.
- Added fixed-step Real Time playback with Start/Pause/Reset, reseeding, logarithmic speed control, substep cap, stall handling, and explicit behind/error status.
- Coalesced pointer movement to one state update per rendered frame.
- Added Shift+click equilibrium selection with priority over dragging.
- Added the live/static purple response-target ring where the active model defines one.

## Analysis

- Expanded equilibrium search to model-aware interior, edge, and vertex behavior.
- Solved Replicator interior roots through payoff equality to prevent near-boundary slow states from being misreported as equilibria.
- Added reduced finite-difference Jacobians, boundary stencils, stability classification, complex eigenpairs, tangent-vector lifting, and defective/repeated detection.
- Added a complete equilibrium inspector and real eigendirection overlays.

## Field visualization

- Added a full-simplex barycentric speed heat map with exact topology, numeric legend, relative/locked ranges, and the blue-cyan-green-yellow-orange-red palette.
- Added Best Response region metadata and flat shading for mixed switching cells.
- Added deterministic border-inclusive streamline generation, visual arc-length resampling, occupancy culling, independent `GL_LINES`, and fixed-pixel endpoint arrows.
- Added targeted cache invalidation so live steps do not rebuild static scientific fields.

## Verification

- Clean MSVC scientific-core test build at `/W4 /WX /permissive-`: passed.
- CTest: 1/1 target passed.
- Visual Studio 2022 x64 Debug rebuild: passed.
- Visual Studio 2022 x64 Release rebuild: passed.
- Windows runtime/visual checks: Logit, Best Response, Replicator, Real Time movement/target behavior, Shift+click inspector, heat map, borders, streamlines, arrows, labels, and non-overlapping default layout passed.

See `FEATURE_PLAN.md`, `SCIENTIFIC_VALIDATION.md`, and `WINDOWS_TEST_CHECKLIST.md` for durable requirements and repeatable acceptance criteria.
