# Scientific polish change log

Baseline: repository `main` at commit `fff4da0` (`Improve equilibrium sweep visualization`, 2026-07-10).

## Numerical core

- Removed projection/clamp normalization from RK4 stages.
- Added tangent-vector and finite-value validation.
- Integrated in two independent simplex coordinates and reconstructed the third.
- Added internal step subdivision for otherwise invalid RK stages.
- Made excessive requested step counts fail instead of silently shortening time.
- Rewrote the OPGG participation term as a stable polynomial valid at `z = 1`.
- Hardened Logit softmax for very small positive noise.
- Added strict OPGG model-domain validation.
- Added explicit interior-equilibrium documentation and final residual verification.
- Hardened equilibrium-sweep endpoint sampling and conservative branch construction.
- Preserved transactional session updates: failed state, parameter, or integration-setting changes do not partially commit.

## Interaction and platform patch

The installer patches the existing `main.cpp` without replacing unrelated UI code:

- mouse/NDC transforms query the current logical window size;
- the scientific viewport preserves its aspect ratio while the window resizes;
- OpenGL viewport uses the current drawable pixel size;
- viewport updates on `SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED`;
- OPGG sliders remain inside `1 < r < n` and `0 < sigma < r - 1`;
- simplex labels use a visible dark color on the white background.

The SDL window wrapper now:

- checks OpenGL attribute-setting failures;
- creates a resizable, high-pixel-density window;
- explicitly makes the context current;
- requests vertical synchronization and reports a warning if unavailable;
- exposes logical and drawable size queries.

The shader-program owner now validates empty sources and OpenGL object creation and emits compile/link diagnostics.

## Build and repository hygiene

- Removes unsupported Win32 configurations.
- Fixes `SdlOpenGlWindow.h` being classified as a compilation unit.
- Removes the tracked generated `imgui.ini` and ignores future local layout files, test builds, and polish backups.
- Adds a complete README and scientific-validation notes.
- Adds a dependency-free scientific-core CMake test target.
