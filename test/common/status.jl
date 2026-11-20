# Tests follow the statuses of the problems (#303, #304).
#
# - `:valid`: checked with `Test.@test`; a failure fails the run.
# - `:draft`: run in a `DraftTestSet`, which records the outcome and never reports to the
#   enclosing test set; a failure never fails the run.
#
# A test between several modelings is `:valid` only when the problem is `:valid` in all of
# them. Every outcome is collected and written as a report at the end of the run
# (`test/reports/<environment>.md`, git-ignored), so that the cron (#305) can tell which
# drafts are ready to become `:valid`.
#
# A solve to convergence is a result, not a test (#360): `result!` records it (solver status,
# objective, iterations, time, gap to the reference objective when there is one) and the
# report lists it; `:draft` means "not finished or not equivalent", not "does not converge".
# It also records how the solve was configured (modeler, solver and its options) and the
# constraint violation, and each run writes its environment, so that a reference solution says
# how it was obtained (#409). A result also carries the fingerprint of the problem and, from
# the convergence tests, the trajectory of the solution, published with the environment of the
# run for the documentation (#314). The files written at the end of a run are in
# `status_reports.jl`.
module TestStatus

using InteractiveUtils: InteractiveUtils
using Pkg: Pkg
using TOML: TOML
using Test: Test
using OptimalControlProblems: OptimalControlProblems

"""
Test set of a `:draft` problem: counts the results of its tests (nested test sets
included) and does not record them into a parent test set that is not a draft one.
"""
mutable struct DraftTestSet <: Test.AbstractTestSet
    description::String
    pass::Int
    fail::Int
    error::Int
    broken::Int
end
DraftTestSet(description::AbstractString; kwargs...) = DraftTestSet(description, 0, 0, 0, 0)

Test.record(ts::DraftTestSet, ::Test.Pass) = (ts.pass += 1; nothing)
Test.record(ts::DraftTestSet, ::Test.Fail) = (ts.fail += 1; nothing)
Test.record(ts::DraftTestSet, ::Test.Error) = (ts.error += 1; nothing)
Test.record(ts::DraftTestSet, ::Test.Broken) = (ts.broken += 1; nothing)
function Test.record(ts::DraftTestSet, child::DraftTestSet)
    ts.pass += child.pass
    ts.fail += child.fail
    ts.error += child.error
    ts.broken += child.broken
    return nothing
end

function Test.finish(ts::DraftTestSet)
    if Test.get_testset_depth() > 0 && Test.get_testset() isa DraftTestSet
        Test.record(Test.get_testset(), ts)
    end
    return ts
end

function outcome(ts::DraftTestSet)
    ts.error > 0 && return "error"
    ts.fail > 0 && return "fail"
    ts.pass > 0 && return "pass"
    return "no test"
end

function outcome(ts::Test.DefaultTestSet)
    counts = Test.get_test_counts(ts)
    # Julia 1.10 returns a tuple (direct counts then cumulative child counts); newer
    # versions return a `Test.TestCounts` with named fields. Nested test sets are part
    # of the problem outcome.
    passes =
        counts isa Tuple ? counts[1] + counts[5] : counts.passes + counts.cumulative_passes
    fails =
        counts isa Tuple ? counts[2] + counts[6] : counts.fails + counts.cumulative_fails
    errors =
        counts isa Tuple ? counts[3] + counts[7] : counts.errors + counts.cumulative_errors
    errors > 0 && return "error"
    fails > 0 && return "fail"
    return passes > 0 ? "pass" : "no test"
end

"""One line of the report."""
struct Row
    test::String
    problem::Symbol
    modelings::String
    status::Symbol
    outcome::String
end

const ROWS = Row[]

"""Record a line of the report (also used for checks that report instead of failing)."""
function report!(test, problem, modelings, status, outcome)
    push!(ROWS, Row(string(test), problem, string(modelings), status, string(outcome)))
    return nothing
end

"""
Result of one solve (#360): reported, never asserted. `converged` is the solver's flag;
`gap` the relative gap of the objective to the reference objective of the problem for the
grid and the scheme of the solve, only when the problem has one (`:reference_objective` in
its metadata, #375); `solution` which known solution a converged solve reached (`:best`,
`:local` or `:new`, #378), under the same condition. `modeler` (`nothing` for JuMP),
`solver`, `options` (every option of the solver, as an inline TOML table) and `violation`
(the largest violation of the constraints) say how the solve was configured and how feasible
it is (#409). `fingerprint` is the one of the problem when it was solved, and `trajectory`
the solution on its time grid, when the test gives it (#314, see `trajectory`).
"""
struct Result
    test::String
    problem::Symbol
    modelings::String
    configuration::String
    modeler::Union{Nothing,Symbol}
    solver::Symbol
    options::String
    grid_size::Int
    scheme::Symbol
    converged::Bool
    reason::String
    objective::Float64
    iterations::Int
    time::Float64
    violation::Float64
    gap::Union{Nothing,Float64}
    solution::Union{Nothing,Symbol}
    fingerprint::String
    trajectory::Union{Nothing,Dict{String,Any}}
end

const RESULTS = Result[]

"""Relative gap of `objective` to `reference` (absolute when the reference is zero)."""
function gap(objective, reference)
    return abs(objective - reference) / (iszero(reference) ? 1 : abs(reference))
end

"""
    result!(test, problem, backend, configuration; modeler, solver, options, grid_size,
            scheme, converged, reason, objective, iterations, violation, time,
            trajectory=nothing, reference=nothing)

Record the result of a solve of `problem` in the modeling of `backend`, discretized on
`grid_size` steps with `scheme`. `modeler`, `solver` and `options` are the `settings` of the
solve modules, the solver status the `outcome` of a solution, `trajectory` the
`trajectory` of the solution. The gap is to `reference`, by default the reference objective
of the problem in this modeling for this discretization (#435); a modeling may give another
one (ModelingToolkit: the objective of the analytic solution while it has none, #439).
"""
function result!(
    test,
    problem::Symbol,
    backend,
    configuration;
    modeler,
    solver,
    options,
    grid_size,
    scheme,
    converged,
    reason,
    objective,
    iterations,
    violation,
    time,
    trajectory=nothing,
    reference=nothing,
)
    if isnothing(reference)
        reference = OptimalControlProblems.reference_objective(
            problem, backend; grid_size, scheme
        )
    end
    push!(
        RESULTS,
        Result(
            string(test),
            problem,
            string(modeling(backend)),
            string(configuration),
            isnothing(modeler) ? nothing : Symbol(modeler),
            Symbol(solver),
            # written now: a solver's own types (`MadNLP.LogLevels`) may not print later, from
            # the report written after the tests (an older world age)
            inline_toml(NamedTuple(options)),
            Int(grid_size),
            Symbol(scheme),
            converged,
            string(reason),
            Float64(objective),
            Int(iterations),
            Float64(time),
            Float64(violation),
            isnothing(reference) ? nothing : gap(objective, reference),
            if converged
                OptimalControlProblems.solution_kind(
                    problem, backend, objective; grid_size, scheme
                )
            else
                nothing
            end,
            OptimalControlProblems.Catalogue._fingerprint(problem),
            trajectory,
        ),
    )
    return nothing
end

modeling(backend) = OptimalControlProblems.Modelings._modeling_spec(backend).name

# Points kept on the time grid of a trajectory: enough to draw it, small enough to publish.
const TRAJECTORY_POINTS = 501

"""
The trajectory of a solution `x` (an OptimalControl solution or a solved JuMP model, read
with the accessors they share), for the plots of the documentation (#314): the time grid,
at most `TRAJECTORY_POINTS` of its nodes, the state, the control and the costate at these
nodes by component, the variable, and the names of the components in the order of the model
(`names`: a dictionary loses it). The costate has the sign convention of `rules/signs.md`
(`-JuMP.dual` for JuMP, the same as OptimalControl's, #326). Values are rounded to 6
significant digits: enough to draw, and stable from one run to the next.
`nothing`, with a warning, when the solution cannot be read (a JuMP model without values):
the solve is still reported.
"""
function trajectory(x)
    try
        return _trajectory(x)
    catch err
        @warn "No trajectory for this solve" exception = err
        return nothing
    end
end

function _trajectory(x)
    O = OptimalControlProblems
    grid = collect(O.time_grid(x))
    keep = if length(grid) <= TRAJECTORY_POINTS
        eachindex(grid)
    else
        unique(round.(Int, range(1, length(grid); length=TRAJECTORY_POINTS)))
    end
    t = grid[keep]
    r(v) = round(Float64(v); sigdigits=6)
    function components(f, names)
        return Dict(name => [r(f(τ)[i]) for τ in t] for (i, name) in enumerate(names))
    end
    xs, us = O.state_components(x), O.control_components(x)
    scalar(f) = τ -> (y=f(τ); y isa Number ? [y] : y)
    return Dict{String,Any}(
        "time" => r.(t),
        "state" => components(scalar(O.state(x)), xs),
        "control" => components(scalar(O.control(x)), us),
        "costate" => components(scalar(O.costate(x)), xs),
        "variable" => r.(vcat(O.variable(x))),
        "names" =>
            Dict("state" => xs, "control" => us, "variable" => O.variable_components(x)),
    )
end

"""
`:valid` when `problem` is `:valid` in every modeling of `backends`, `:draft` otherwise.
"""
function status(problem::Symbol, backends)
    statuses = OptimalControlProblems.status(problem)
    return if all(b -> get(statuses, modeling(b), nothing) === :valid, backends)
        :valid
    else
        :draft
    end
end

"""
    select_problems!(names)

Restrict the tests to the problems `names` (`problem=a,b` in the test arguments); no names:
every problem. A name that is not in the catalogue is an error: a selection that matches
nothing would test nothing and pass.
"""
function select_problems!(names)
    if isempty(names)
        Main.TestConfigs.PROBLEMS[] = nothing
        return nothing
    end
    wanted = Symbol.(names)
    unknown = setdiff(wanted, OptimalControlProblems.problems(; status=:all))
    isempty(unknown) || throw(
        ArgumentError(
            "Unknown problems in `problem=`: $(join(unknown, ", ")); see problems(; status=:all)",
        ),
    )
    Main.TestConfigs.PROBLEMS[] = wanted
    return nothing
end

"""The problems of `problems` selected by `select_problems!`."""
function selected(problems)
    wanted = Main.TestConfigs.PROBLEMS[]
    return isnothing(wanted) ? problems : [p for p in problems if p in wanted]
end

"""
    for_each_problem(f, test, backends...; only=nothing)

Call `f(problem)` in a test set for every problem available in all the modelings of
`backends` (whatever its status): a plain test set for a `:valid` problem, a
`DraftTestSet` for a `:draft` one. The outcome of drafts is recorded for the report.
With `only`, a list of problems, the others are not visited: they have no line in the report
(a test that runs on a few representative problems, #441).
"""
function for_each_problem(f, test::AbstractString, backends...; only=nothing)
    modelings = join(map(modeling, backends), " vs ")
    problems = selected(OptimalControlProblems.problems(backends...; status=:all))
    isnothing(only) || (problems = [p for p in problems if p in only])
    for problem in problems
        if status(problem, backends) === :valid
            ts = Test.@testset "$problem" begin
                f(problem)
            end
            report!(test, problem, modelings, :valid, outcome(ts))
        else
            ts = Test.@testset DraftTestSet "$problem (draft)" begin
                f(problem)
            end
            report!(test, problem, modelings, :draft, outcome(ts))
            @info "Draft $problem ($modelings, $test): $(outcome(ts))" ts.pass ts.fail ts.error
        end
    end
    return nothing
end

# the files written at the end of a run
include(joinpath(@__DIR__, "status_reports.jl"))

end # module TestStatus
