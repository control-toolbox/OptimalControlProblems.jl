# Breaking Changes

This document describes breaking changes in OptimalControlProblems releases.

## [Unreleased]

#### `grid_size_data(problem)` replaced by `grid_size_data(problem, backend)`

- **The grid depends on the modeling**: `grid_size_data(problem, backend)` is the default
  grid of `problem` in the modeling of `backend` (#473); the one-argument form is removed.
  The field `:grid_size` of the metadata is still the grid of the problem, the default of the
  modelings but for ModelingToolkit (coarser, solved by shooting).
- **Why**: the modelings are independent (#435), and a grid without a modeling was the wrong
  one for ModelingToolkit and for any problem with `:grid_sizes`.

**Migration**:

```julia
# Before
N = grid_size_data(:beam)

# After
N = grid_size_data(:beam, OptimalControlBackend())
```

#### The references are by modeling

- **`reference_objective`, `solution_kind`, `references` and `best_reference` take the
  backend**: each modeling has its own reference objectives, computed from its own solves at
  its own default grid (`grid_size_data(problem, backend)`) and discretization (the
  trapezoidal rule, single shooting for ModelingToolkit). The files
  `ext/MetaData/references/<problem>.toml` have one table per modeling and scheme,
  `[JuMP.trapeze]`.
- **Why**: the modelings are independent (#435): different solvers, grids and
  discretizations; a reference shared by every modeling mixed them, and a modeling with its
  own grid (ModelingToolkit) had none.

**Migration**:

```julia
# Before
OptimalControlProblems.reference_objective(:beam)
OptimalControlProblems.solution_kind(:beam, objective; grid_size=500, scheme=:trapeze)

# After
OptimalControlProblems.reference_objective(:beam, OptimalControlBackend())
OptimalControlProblems.solution_kind(
    :beam, JuMPBackend(), objective; grid_size=500, scheme=:trapeze
)
```

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
