# [Guided tour](@id guided-tour)

What the collection holds, and what each problem comes with.

```@example tour
using OptimalControlProblems
```

## The problems

`problems()` lists the problems that are valid in at least one modeling. Given a backend, it
lists those of that modeling; given several, those valid in all of them, which are the ones to
use to compare modelings.

```@example tour
problems()
```

```@example tour
problems(JuMPBackend(), OptimalControlBackend())
```

A problem is `valid` in a modeling once it is checked there, `draft` before. Drafts are listed
with `status=:all`:

```@example tour
setdiff(problems(; status=:all), problems())
```

```@example tour
OptimalControlProblems.status(:bryson_denham)
```

The availability of a problem in a modeling is the presence of its file, so this list is never
written by hand. The [Status](@ref problems-status) page shows it for every problem.

## The metadata

Every problem has metadata, read without building it:

```@example tour
problem_metadata(:beam)[:grid_size]
```

```@example tour
parameters_data(:beam)
```

```@example tour
OptimalControlProblems.initial_guess_data(:beam)
```

```@example tour
OptimalControlProblems.reference_objective(:beam)
```

The reference objective depends on the scheme and the grid: it is the objective of the
discretized problem. When the continuous problem has a known solution, it is in the metadata
too:

```@example tour
OptimalControlProblems.reference_solution_data(:beam).objective
```

## The features

The features of a problem are computed from its OptimalControl model, which needs the
OptimalControl extension:

```@example tour
using OptimalControl
OptimalControlProblems.features(:beam)
```

The table of [all the problems](@ref problems-browser) filters them by these features.

## The modelings

A problem is written in one or more modelings, and each modeling returns the most abstract
object it can represent. OptimalControl returns the continuous problem: the discretization is
chosen when solving. JuMP returns a model already discretized: the scheme and the grid are
chosen when the problem is built.

```@example tour
using JuMP
pb = beam(OptimalControlBackend())
nlp = beam(JuMPBackend(); grid_size=100, scheme=:trapeze)
num_variables(nlp)
```

On each [problem page](@ref description-beam), a tab bar at the top switches between the
problem, its modelings and their comparison.
