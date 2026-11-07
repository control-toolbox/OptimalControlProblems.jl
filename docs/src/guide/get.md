# [Get a problem](@id get-problem)

```@example get
using OptimalControl, JuMP, OptimalControlProblems
```

## Choose the problems

```@example get
problems(OptimalControlBackend())
```

`problems(backend)` lists the problems valid in that modeling, and
`problems(backend1, backend2)` those valid in both. The keyword `status` (`:valid`, `:draft` or
`:all`) selects by status. Every list is computed from the files of the package, so it is
always up to date.

## Build a problem

A problem is a function of the modeling backend. Each modeling returns the most abstract object
it can represent, and takes at retrieval only what it needs to build it:

| Modeling | Returns | Keywords |
| :--- | :--- | :--- |
| OptimalControl, scalarized OptimalControl | a [`Problem`](@ref OptimalControlProblems.Problem): the continuous problem and its initial guess | `parameters`, `init` |
| JuMP | a `JuMP.Model`, already discretized | `grid_size`, `scheme`, `ordering`, `parameters`, `init`, and the arguments of `JuMP.Model` |

```@example get
pb = chain(OptimalControlBackend())
```

```@example get
nlp = chain(JuMPBackend(); grid_size=50)
num_variables(nlp)
```

## Change the parameters

The default parameters are in the metadata; `parameters` replaces some of them, the others keep
their default value.

```@example get
parameters_data(:beam)
```

```@example get
pb = beam(OptimalControlBackend(); parameters=(x₁_u=0.2,))
metadata(pb)[:parameters]
```

The [problem page](@ref description-beam) of each problem lists its parameters.

## Read the metadata

| Function | Returns |
| :--- | :--- |
| `problem_metadata(:p)` | every metadata field, without building the problem |
| `metadata(pb)` | the metadata of a built `Problem`, with the parameters it was built with |
| `grid_size_data(:p)` | the default number of steps |
| `parameters_data(:p)` | the default parameters |
| `OptimalControlProblems.initial_guess_data(:p)` | the shared initial guess (see [Initial guess](@ref initial-guess)) |
| `OptimalControlProblems.reference_objective(:p; grid_size, scheme)` | the reference objective of the discretized problem, or `nothing` |
| `OptimalControlProblems.reference_solution_data(:p)` | the analytic solution of the continuous problem, when known |
| `OptimalControlProblems.features(:p)` | dimensions, kind of cost, free times, constraints |
| `OptimalControlProblems.status(:p)` | the status in each modeling |

```@example get
OptimalControlProblems.features(:chain)
```
