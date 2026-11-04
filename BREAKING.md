# Breaking Changes

This document describes breaking changes in OptimalControlProblems releases.

## [Unreleased]

#### OptimalControl problems return the continuous problem

- **A problem function of the `OptimalControlBackend` and
  `OptimalControlScalarizedBackend` modelings returns a `Problem`**: the continuous
  model and its initial guess. It no longer discretizes with the trapezoidal rule, and
  no longer takes a description, `grid_size` or keyword arguments for the
  discretization; only `parameters` remains.
- **Why**: OptimalControl v2 removes `direct_transcription`; the scheme, the grid, the
  modeler, the solver and CPU/GPU are chosen in `solve`, as the benchmarks need.

**Migration**:

```julia
# Before
docp = beam(OptimalControlBackend(), :adnlp; grid_size=500)
nlp = nlp_model(docp)

# After
pb = beam(OptimalControlBackend())
sol = solve(model(pb); init=initial_guess(pb), grid_size=500)
```

#### `metadata(:problem)` renamed to `problem_metadata(:problem)`

- **`metadata(::Symbol)` and `metadata()` are removed.** Use `problem_metadata`, or
  `metadata(pb)` on a built `Problem`.
- **Why**: `metadata` is now the function OptimalControl reexports from CTBase; a method
  on `Symbol` would be type piracy, and a function of our own would conflict with it.

**Migration**:

```julia
# Before
metadata(:beam)[:grid_size]

# After
problem_metadata(:beam)[:grid_size]
metadata(beam(OptimalControlBackend()))[:grid_size]
```

#### `CTBase.UnauthorizedCall` replaced by `CTBase.Exceptions.PreconditionError`

- Passing `parameters` to a problem that has none throws
  `CTBase.Exceptions.PreconditionError`: `UnauthorizedCall` does not exist in CTBase 0.30.

#### Shared accessors are the CTModels functions

- `time_grid`, `state`, `control`, `costate`, `variable`, `objective`, `iterations` and
  the `*_components` / `*_dimension` accessors are the functions of CTModels 0.19 that
  OptimalControl reexports (`objective` and `variable` were ExaModels functions).

## [0.5.0] - 2026-03-10 — baseline

Breaking changes from this version onward are tracked here.
