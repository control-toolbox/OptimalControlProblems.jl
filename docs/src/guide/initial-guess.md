# [Initial guess](@id initial-guess)

Every problem has **one initial guess, shared by all its modelings**. It is written once, in
the metadata of the problem, as a function of the parameters: changing a parameter changes the
initial guess accordingly.

```@example init
using OptimalControl, JuMP, OptimalControlProblems
```

```@example init
OptimalControlProblems.initial_guess_data(:beam)
```

Each component is a constant, a vector, or a function of the time:

```@example init
OptimalControlProblems.initial_guess_data(:chain)
```

## How each modeling uses it

- The OptimalControl modelings return it with the problem, `initial_guess(pb)`, to be given as
  `init` to `solve`.
- JuMP evaluates it at the nodes of the scheme when the model is built: it is the start value
  of the variables.

The two give the same first iterate: this is checked, with the rest of the NLP, on every pull
request (see [Compare modelings](@ref compare-modelings)).

## Use another one

Every modeling takes another initial guess with the keyword `init`, with the same format:

```@example init
init = (state=[0.0, 0.0], control=0.0)
pb = beam(OptimalControlBackend(); init=init)
initial_guess(pb)
```

```@example init
nlp = beam(JuMPBackend(); grid_size=50, init=init)
start_value.(all_variables(nlp)[1:3])   # x₁, x₂ and u at the first node
```

## How robust it is

The initial guess of a problem must make every modeling and configuration converge, without
being the solution. The robustness crons solve each problem again under other options of the
solvers (barrier updates, initial barrier parameter): the results are on each problem page, and
the solves that reach another solution than the best known one are listed on the
[Status](@ref problems-status) page.
