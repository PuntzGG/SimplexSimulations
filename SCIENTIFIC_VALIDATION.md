# Scientific validation notes

## Corrected numerical semantics

The earlier integrator created every RK4 intermediate state through `SimplexState::Normalized`. That operation clips negative coordinates and rescales the state, changing the differential equation being integrated and potentially hiding a timestep that leaves the admissible domain.

The replacement integrator:

1. verifies finite, tangent derivatives (`dx + dy + dz approximately 0`);
2. advances the independent `x` and `y` coordinates;
3. reconstructs `z = 1 - x - y` exactly;
4. creates stages through simplex validation rather than materially projecting them back into the domain;
5. subdivides a nominal interval only when a stage would otherwise be invalid;
6. rejects settings that would silently truncate the requested integration time;
7. treats near-integral floating-point time ratios robustly, avoiding spurious zero-length final steps.

## Stable participation term

The OPGG participation factor is algebraically evaluated as

```text
A(z) = (1/n) sum from j=0 to n-2 of (n-1-j) z^j
```

rather than through a quotient containing repeated divisions by `1-z`. The polynomial is equivalent for `z != 1` and directly gives the finite limit `(n-1)/2` at `z = 1`.

## Stable Logit softmax

The implementation evaluates

```text
exp((payoff_i - max_payoff) / eta)
```

instead of dividing every payoff by `eta` before subtracting the maximum. All exponents are non-positive, preventing positive exponential overflow even when `eta` is small.

## Model-domain validation

The scientific OPGG parameter domain enforced by the replacement is:

```text
n >= 2
1 < r < n
0 < sigma < r - 1
c > 0
0 <= d <= 1
eta > 0
```

This is intentionally stricter than checking only whether arithmetic can be performed.

## Equilibrium interpretation

The equilibrium finder is an interior, damped-Newton multi-seed search. Every accepted point is independently reevaluated and must satisfy the requested infinity-norm residual tolerance. Its output should be interpreted as "verified roots found," not as a mathematical completeness proof.

## Automated checks included

- valid and invalid simplex-state construction;
- finite-value rejection;
- OPGG model-domain constraints;
- state/NDC/state round trip;
- nearest-simplex clamping for an outside point;
- reference center derivative from the project notebook;
- finite, tangent Logit vector field over a simplex lattice;
- quotient-form/polynomial-form payoff equivalence away from the limit;
- finite and analytically checked pure-loner evaluation;
- exact constant tangent-field integration and rejection of non-tangent fields;
- no `maxSteps` truncation or spurious near-integral step;
- 501-state default trajectory;
- final-state convergence after halving the timestep;
- equilibrium validity and residual verification;
- monotone, in-range equilibrium-sweep samples and residuals;
- transactional `SimulationSession` rejection of invalid updates.
