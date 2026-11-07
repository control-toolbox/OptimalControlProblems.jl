# [Compare modelings](@id compare-modelings)

A problem written in two modelings is two writings of the same mathematical problem. The
collection checks that they are, and compares every solve to one reference objective.

## The same NLP

Discretized the same way, two modelings of a problem must give the **same nonlinear program**:
the same variables, constraints and objective, and the same derivatives, at the initial guess.
On every pull request, the tests build both NLPs and compare them:

```@eval
using Markdown
rows = String[]
for (spec, adapter) in Main.modelings()
    other = adapter.checked_against
    isnothing(other) && continue
    push!(rows, "| $(adapter.title) | $(Main.ADAPTERS[other].title) |")
end
Markdown.parse("""
| Modeling | compared to |
| :--- | :--- |
$(join(rows, "\n"))
""")
```

for every problem valid in both, with the trapezoidal rule, and with every scheme and both
orderings on a few problems chosen to cover every feature (kind of cost, free times, variable,
kind of constraint). See [JuMP schemes](@ref jump-schemes) for the NLP of each scheme.

The solutions are not compared: two solves of the same NLP by two solvers, or by the same
solver from two modelings, are two optimization paths, and may stop at two points within the
tolerance, or at two local solutions.

## One reference objective

Each problem has a reference objective per scheme, on its default grid: the best objective
among the solves of every modeling and configuration that converged, in the convergence and
robustness crons. Each solve is then shown with its **gap to the reference**, as it is: a gap
is reported, never tested.

On each problem page, the **Compare** tab, present when the problem has two modelings or more,
shows:

- which pairs of modelings are checked to give the same NLP;
- the results of each modeling side by side: number of solves, objectives, largest gap;
- which solution each modeling reached, when the problem has several local solutions.

For example, [Mountain car](@ref description-mountain_car) has two local solutions, and the
Compare tab says which configurations reach which.

## Compare yourself

To compare two modelings, build them the same way and compare the objectives to the reference:

```@example compare
using OptimalControl, JuMP, Ipopt, NLPModelsIpopt, OptimalControlProblems

problem = :beam
N = grid_size_data(problem)
reference = OptimalControlProblems.reference_objective(problem; grid_size=N, scheme=:trapeze)

pb = OptimalControlProblems.beam(OptimalControlBackend())
sol = solve(
    model(pb), :collocation, :adnlp, :ipopt;
    init=initial_guess(pb), grid_size=N, scheme=:trapeze, display=false,
)

nlp = OptimalControlProblems.beam(JuMPBackend(); grid_size=N, scheme=:trapeze)
set_optimizer(nlp, Ipopt.Optimizer)
set_silent(nlp)
optimize!(nlp)

gap(J) = abs(J - reference) / abs(reference)
(OptimalControl=gap(objective(sol)), JuMP=gap(objective_value(nlp)))
```
