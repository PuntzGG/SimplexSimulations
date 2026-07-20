# SimplexBeast scientific validation notes

## Shared OPGG payoffs

All dynamics call `OpggPayoffEvaluator`; no dynamic owns a private variant of the game. The participation factor is evaluated through the stable polynomial

```text
A(z) = (1/n) sum from j=0 to n-2 of (n-1-j) z^j
```

instead of a quotient with repeated divisions by `1-z`. The polynomial has the exact finite limit `(n-1)/2` at `z = 1`.

The enforced model domain is

```text
n >= 2
1 < r < n
0 < sigma < r - 1
c > 0
0 <= v <= 1
eta > 0
```

`eta` affects only Logit dynamics, but remains in the shared parameter record so model switching preserves it.

## Dynamic definitions

For payoff vector `pi(s)` and mean payoff `bar_pi = sum_i s_i pi_i`:

```text
Logit:         ds/dt = softmax(pi / eta) - s
Best Response: ds/dt = B(s) - s
Replicator:    ds_i/dt = s_i (pi_i - bar_pi)
```

Logit subtracts the maximum payoff before exponentiation, preventing positive exponential overflow for small `eta`.

`B(s)` assigns equal mass to all payoff maximizers within

```text
absoluteTieTolerance + relativeTieTolerance * max(1, max_i |pi_i|)
```

This gives scale-aware deterministic ties. The resulting field is nonsmooth at support changes; a classical Jacobian is not reported on those switching surfaces.

Replicator multiplication by `s_i` preserves every absent-strategy face. The integrator and field tests verify this on vertices and edge trajectories.

## RK4 and Real Time semantics

The shared integrator:

1. rejects non-finite or non-tangent derivatives;
2. evolves the independent `x` and `y` chart coordinates;
3. reconstructs `z = 1 - x - y`;
4. validates each RK4 stage instead of projecting it back into the simplex;
5. subdivides a nominal step when an otherwise valid flow would leave the domain at an intermediate stage;
6. rejects settings that would silently truncate requested time.

Real Time mode calls the same `AdvanceOneStep` implementation at fixed `dt = 0.01`. A steady-clock accumulator converts wall time into a count of fixed steps. Playback speed changes simulated time per wall second, not `dt`. Work is capped per frame; excess wall time is dropped with a `Playback cannot keep pace` status instead of skipping states or taking a larger numerical step. Long suspensions reset the effective baseline and do not create a catch-up jump.

## Equilibrium semantics

- Logit: damped-Newton multi-seed search in the open simplex.
- Equal-split Best Response: residual verification of the seven possible uniform-support targets; no smooth Newton iteration.
- Replicator: exact vertices, scanned/bisected edges, degenerate-edge representatives, and an interior payoff-equality search.

For Replicator, solving the raw vector field in the open simplex can misclassify points arbitrarily close to a boundary: `s_i(pi_i-bar_pi)` becomes small just because `s_i` is small. The implemented interior root function instead solves

```text
pi_x - pi_z = 0
pi_y - pi_z = 0
```

and then independently verifies the original dynamic residual. Boundary points are handled by the explicit face search. This distinction is regression-tested.

Every displayed point is reevaluated and must meet the requested infinity-norm field residual. Results mean “verified roots found,” not a proof that every mathematical equilibrium has been enumerated.

## Jacobian and eigenanalysis

The inspector differentiates the reduced vector field `(dx/dt, dy/dt)` in the chart `z = 1-x-y`.

- Interior: central stencils at `h` and `h/2` with a relative convergence check.
- Boundary: feasible one-sided directions are used and transformed back into the reduced chart.
- Best Response ties: analysis is rejected as nonsmooth.

The analytic 2x2 eigenvalue calculation supports real and complex pairs, detects repeated/defective cases, and classifies nodes, saddles, spirals, center-like cases, and nonhyperbolic/inconclusive cases. Reduced vectors `(v_x,v_y)` are lifted to `(v_x,v_y,-v_x-v_y)`, guaranteeing tangency.

## Field visualization semantics

The heat map samples the exact barycentric lattice counts

```text
samples = (N + 1)(N + 2)/2
triangles = N^2
```

including every border and vertex. Its scalar is `sqrt(dx^2+dy^2+dz^2)`. Relative normalization uses the current sampled range; locked normalization preserves a user-specified numeric range. Mixed Best Response regions are marked and flat shaded to avoid visually inventing a smooth transition across a discontinuity.

Streamline seeds are deterministic and boundary-first. Paths use the validated RK4 integrator, are resampled by visual arc length, and are culled through a deterministic occupancy grid. GPU line data uses `GL_LINES`, so independent paths cannot be connected accidentally. Each nondegenerate accepted path has one downstream arrowhead whose display size is fixed in pixels.

## Automated coverage

The MSVC scientific-core target uses `/W4 /WX /permissive-` and checks:

- state and parameter validation;
- shared-payoff equivalence and the reference Logit derivative;
- finite tangent fields for all three dynamics;
- Replicator face invariance and Best Response tie semantics;
- RK4 accuracy, subdivision behavior, and invalid-field rejection;
- transactional state, parameter, settings, and dynamic changes;
- Real Time pause/start/reset/reseed, frame partitioning, speed, stall, error, and batch agreement;
- Logit, Best Response, and Replicator equilibrium coverage and residuals;
- Replicator interior payoff equality and rejection of near-boundary false roots;
- finite-difference Jacobians, boundary stencils, nonsmooth rejection, eigenpairs, tangency, classifications, and defective matrices;
- heat-map counts, borders, palette, normalization, and mixed-region metadata;
- deterministic streamline segments, arrows, simplex validity, border seeds, and Replicator boundary preservation.
