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
module TestStatus

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
    passes = counts isa Tuple ? counts[1] + counts[5] : counts.passes + counts.cumulative_passes
    fails = counts isa Tuple ? counts[2] + counts[6] : counts.fails + counts.cumulative_fails
    errors = counts isa Tuple ? counts[3] + counts[7] : counts.errors + counts.cumulative_errors
    errors > 0 && return "error"
    fails > 0 && return "fail"
    passes > 0 ? "pass" : "no test"
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
its metadata, #375).
"""
struct Result
    test::String
    problem::Symbol
    modelings::String
    configuration::String
    grid_size::Int
    scheme::Symbol
    converged::Bool
    reason::String
    objective::Float64
    iterations::Int
    time::Float64
    gap::Union{Nothing,Float64}
end

const RESULTS = Result[]

"""Relative gap of `objective` to `reference` (absolute when the reference is zero)."""
gap(objective, reference) = abs(objective - reference) / (iszero(reference) ? 1 : abs(reference))

"""
    result!(test, problem, backend, configuration; grid_size, scheme, converged, reason,
            objective, iterations, time)

Record the result of a solve of `problem` in the modeling of `backend`, discretized on
`grid_size` steps with `scheme`.
"""
function result!(
    test, problem::Symbol, backend, configuration;
    grid_size, scheme, converged, reason, objective, iterations, time,
)
    reference = OptimalControlProblems.reference_objective(problem; grid_size, scheme)
    push!(
        RESULTS,
        Result(
            string(test),
            problem,
            string(modeling(backend)),
            string(configuration),
            Int(grid_size),
            Symbol(scheme),
            converged,
            string(reason),
            Float64(objective),
            Int(iterations),
            Float64(time),
            isnothing(reference) ? nothing : gap(objective, reference),
        ),
    )
    return nothing
end

modeling(backend) = OptimalControlProblems._modeling_spec(backend).name

"""
`:valid` when `problem` is `:valid` in every modeling of `backends`, `:draft` otherwise.
"""
function status(problem::Symbol, backends)
    statuses = OptimalControlProblems.status(problem)
    return all(b -> get(statuses, modeling(b), nothing) === :valid, backends) ? :valid : :draft
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
        ArgumentError("Unknown problems in `problem=`: $(join(unknown, ", ")); see problems(; status=:all)"),
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
    for_each_problem(f, test, backends...)

Call `f(problem)` in a test set for every problem available in all the modelings of
`backends` (whatever its status): a plain test set for a `:valid` problem, a
`DraftTestSet` for a `:draft` one. The outcome of drafts is recorded for the report.
"""
function for_each_problem(f, test::AbstractString, backends...)
    modelings = join(map(modeling, backends), " vs ")
    for problem in selected(OptimalControlProblems.problems(backends...; status=:all))
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

const REPORTS = joinpath(dirname(@__DIR__), "reports")

"""
Print the report and write it to `test/reports/<name>.md`, with the results of the solves
(#360) as a second table and as `test/reports/<name>-results.tsv`. When there is nothing to
report, nothing is written and the files of a previous run are removed.
"""
function write_report(name::AbstractString)
    file = joinpath(REPORTS, "$name.md")
    tsv = joinpath(REPORTS, "$name-results.tsv")
    rm(tsv; force=true)
    isempty(ROWS) && isempty(RESULTS) && (rm(file; force=true); return nothing)
    lines = [
        "# Test report: $name",
        "",
        "| test | problem | modelings | status | outcome |",
        "| --- | --- | --- | --- | --- |",
    ]
    for r in ROWS
        push!(lines, "| $(r.test) | $(r.problem) | $(r.modelings) | $(r.status) | $(r.outcome) |")
    end
    mkpath(REPORTS)
    isempty(RESULTS) || append!(lines, results_table(tsv))
    text = join(lines, "\n") * "\n"
    write(file, text)
    # lines worth reading: drafts, and checks that report instead of failing
    notable = filter(r -> r.status !== :valid || r.outcome != "see the test summary", ROWS)
    println("\nReport written to $file ($(length(ROWS)) lines, $(length(notable)) to read).")
    for r in notable
        println("  $(r.status): $(r.problem) [$(r.modelings)] $(r.test) → $(r.outcome)")
    end
    return file
end

# The results of the solves: a Markdown table for the report (the cron reads only the
# five-column table of the outcomes), and a tab-separated file for the tools, read by column
# name, with the objective at full precision (the reference objectives are computed from it).
function results_table(tsv::AbstractString)
    short(x) = isnothing(x) ? "" : string(round(x; sigdigits=6))
    full(x) = isnothing(x) ? "" : repr(x)
    header = [
        "test", "problem", "modeling", "configuration", "grid_size", "scheme", "result",
        "reason", "objective", "iterations", "time", "gap",
    ]
    cells(r, fmt) = [
        r.test, string(r.problem), r.modelings, r.configuration, string(r.grid_size),
        string(r.scheme), r.converged ? "converged" : "not_converged", r.reason,
        fmt(r.objective), string(r.iterations), short(r.time), short(r.gap),
    ]
    open(tsv, "w") do io
        println(io, join(header, "\t"))
        foreach(r -> println(io, join(cells(r, full), "\t")), RESULTS)
    end
    lines = [
        "",
        "## Results of the solves",
        "",
        "| " * join(header, " | ") * " |",
        "|" * repeat(" --- |", length(header)),
    ]
    append!(lines, ["| " * join(cells(r, short), " | ") * " |" for r in RESULTS])
    not_converged = count(r -> !r.converged, RESULTS)
    println("\nResults: $(length(RESULTS)) solves, $not_converged not converged (reported, not tested).")
    return lines
end

end # module TestStatus
