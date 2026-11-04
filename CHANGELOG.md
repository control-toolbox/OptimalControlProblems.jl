# Changelog

All notable changes to OptimalControlProblems will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### 💥 Breaking Changes

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
sol = solve(ocp(pb); init=initial_guess(pb), grid_size=500)
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

See [BREAKING.md](BREAKING.md).

### ✨ New Features

- `Problem` type, read with `name`, `ocp`, `initial_guess` and `metadata`, and shown with
  its modeling, status and dimensions.
- `using OptimalControl, OptimalControlProblems` loads without name conflict.

### 🛠 Enhancements

- The initial guess of `rocket` is a function of time, independent of the grid.

### 📦 Dependencies

- OptimalControl 2.2, CTModels 0.19, CTBase 0.30.
- CTDirect, ADNLPModels, ExaModels and SolverCore are no longer dependencies.

### ✅ Compatibility

- **Breaking**: see the `💥 Breaking Changes` subsection above.

## [0.5.0] - 2026-03-10 — baseline

This is the reference version. No changelog was maintained before this point;
use `git log` for earlier history. Breaking changes from this version onward
are tracked in [BREAKING.md](BREAKING.md).
