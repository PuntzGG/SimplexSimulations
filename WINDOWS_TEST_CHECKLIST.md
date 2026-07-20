# SimplexBeast Windows acceptance checklist

## 1. Clean build and startup

- Open `SimplexBeast.sln` in Visual Studio 2022.
- Select `x64`; rebuild both Debug and Release.
- Confirm SDL3 and GLEW DLLs are copied beside both executables.
- Launch Release and confirm only the SimplexBeast application window appears.
- Confirm the control panel does not obscure the simplex at the default size.

## 2. Scientific-core regression suite

From a Visual Studio Developer PowerShell:

```powershell
cmake -S tests -B build-tests
cmake --build build-tests --config Release --clean-first
ctest --test-dir build-tests -C Release --output-on-failure
```

Expected result: `100% tests passed`.

## 3. Baseline visualization and resizing

- Confirm the heat map covers the entire triangle, including every edge and vertex.
- Confirm the palette progresses dark blue, cyan, green, yellow, orange, red and the legend reports numeric limits.
- Confirm independent black streamline curves and downstream arrowheads fill the interior and borders.
- Confirm the red selected trajectory, orange state, purple target ring, green equilibria, dark outline, and x/y/z labels are distinct.
- Resize horizontally and vertically and test the normal display scaling (for example 125% or 150%). The simplex must not stretch, fixed-pixel arrows must remain readable, and clicking must stay aligned.

## 4. Static-state interaction

- Press `1`, `2`, `3`, and `C`; confirm the marker moves to the three vertices and center.
- Click multiple interior states; confirm one trajectory and one applicable target update.
- Drag across and outside each edge; confirm coalesced updates and nearest-border clamping without field flicker.

## 5. Dynamics switching

- Select Logit; confirm a smooth field, purple softmax target, eta control, and Logit sweep controls.
- Select Equal-split Best Response; confirm the purple target snaps to a pure or uniform-tie state, eta/sweeps are marked Logit-only, and mixed heat-map cells are flat shaded instead of smoothed across switching boundaries.
- Select Replicator; confirm the purple target is hidden and boundary streamlines remain on invariant faces.
- At default parameters, confirm Replicator reports four verified rest points rather than a cloud of near-boundary slow states.
- Repeatedly switch models and confirm shared OPGG parameters and the current state are preserved.

## 6. Real Time playback

- Enter Real Time mode and confirm it starts paused from the current static state.
- Click Start, Pause, and Start again; the orange point and coordinates must move only while running.
- Confirm the red future trajectory is hidden while Real Time is active.
- Change playback speed between `0.1x` and `8x`; the fixed `dt` display must not change.
- Click or drag to reseed, then Reset; Reset must return to the latest seed and pause at simulated time zero.
- Hold a state drag while running; integration must not advance during the drag.
- Minimize or interrupt the window for more than the suspension threshold; restoration must not cause a large catch-up jump.
- Change a model or OPGG parameter while running; the valid live state must remain, playback must pause, and the field must refresh once.
- Leave Real Time; the final live point must become the static start and regenerate exactly one projected trajectory.

## 7. Equilibrium selection and analysis

- Shift+click every visible green marker; confirm ordinary dragging never starts and the orange state does not move.
- Confirm the inspector reports coordinates, location, isolation status, residual, and payoffs.
- For smooth points, confirm the reduced Jacobian, basis, finite-difference check, stability class, eigenvalues, and simplex-tangent eigenvectors are present.
- Confirm real eigendirections appear as short overlays and complex eigendirections remain textual.
- Select a Best Response tie equilibrium, when available; confirm the inspector explicitly reports that a classical Jacobian is undefined at the switching surface.
- Shift+click empty space and confirm the selection clears.
- Change model or parameters and confirm stale selection clears.

## 8. Heat-map controls

- Change resolution from 4 to 128 and confirm detail changes only after editing completes.
- Switch between relative and locked normalization.
- Enter a valid locked minimum/maximum and compare multiple parameter sets using the unchanged range.
- Enter an invalid locked range and confirm the previous valid settings are restored with a visible error.
- Toggle visibility; trajectories, equilibria, and streamlines must remain cached and unchanged.

## 9. Streamline controls

- Change density, field trajectory time, and field timestep; confirm only the field regenerates.
- Change arrow length; arrow size must change in screen pixels without recomputing trajectories.
- Confirm no line joins two unrelated paths, each nondegenerate accepted path has one endpoint arrow, and no arbitrary arrow appears at a rest point.
- Switch away and back without parameter changes; regenerated output must be deterministic.

## 10. Parameters, sweeps, and failure behavior

- Change group size; keep `1 < r < n`.
- Lower `r`; keep `0 < sigma < r - 1`.
- Exercise contribution cost, punishment, and Logit noise limits.
- Generate eta and punishment Logit sweeps; confirm dots are verified samples and lines are conservative branch links.
- Try invalid numeric heat-map inputs and extreme supported integration controls; errors must be visible and prior valid state must remain committed.
