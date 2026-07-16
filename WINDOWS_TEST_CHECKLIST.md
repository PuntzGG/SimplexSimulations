# Windows acceptance checklist

Use this after applying the overlay to the audited repository baseline.

## 1. Clean build

- Open `SimplexSimulations.sln` in Visual Studio 2022.
- Confirm the selected platform is `x64`.
- Build both `Debug` and `Release`.
- Confirm there are no missing SDL3, GLEW, OpenGL, or ImGui symbols.
- Place the required runtime DLLs beside the executable or make them available on `PATH`.

## 2. Scientific-core regression suite

From a Developer PowerShell prompt in the repository root:

```powershell
cmake -S tests -B build-tests
cmake --build build-tests --config Release
ctest --test-dir build-tests -C Release --output-on-failure
```

Expected result: `100% tests passed`.

## 3. Startup and rendering

- Launch the application.
- Confirm the simplex, dark outline, trajectory, orange state marker, and `x/y/z` labels are visible.
- Confirm the labels are dark against the white background.
- Resize the window horizontally and vertically.
- Confirm the simplex does not stretch and mouse selection still aligns with the displayed triangle.
- Check the application on the display scaling used in normal work (for example 125% or 150%).

## 4. State interaction

- Press `1`, `2`, `3`, and `C`; confirm the marker moves to the three corners and center.
- Click several interior states; confirm marker and trajectory update.
- Drag outside each side and corner; confirm the marker follows the nearest simplex boundary without jumping to an unrelated edge.

## 5. OPGG parameter domain

- Change group size and confirm `r` remains strictly between `1` and `n`.
- Lower `r` and confirm `sigma` remains strictly between `0` and `r - 1`.
- Sweep punishment from `0` to `1` and Logit noise over its control range.
- Confirm parameter changes regenerate the trajectory without repeated invalid-parameter errors.

## 6. Equilibria and sweeps

- Enable Logit rest points and confirm all displayed residuals are at or below the configured tolerance.
- Generate an eta sweep and a punishment sweep.
- Confirm dots represent verified samples and lines only join conservatively linked samples.
- Repeat at several valid parameter combinations, including small eta values.

## 7. Trajectory integration

- Test the minimum and maximum trajectory duration and timestep controls.
- Confirm every trajectory remains in the simplex.
- Compare a trajectory at timestep `0.02` with `0.01`; the final positions should be visually indistinguishable for the default parameters.

## 8. Rollback

After testing, retain the timestamped `.scientific-polish-backup-*` directory until the branch is committed. The package's `restore_polish.ps1` restores every replaced file and removes files that the installer added.
