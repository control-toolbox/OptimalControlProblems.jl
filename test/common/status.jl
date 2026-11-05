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
    passes = counts isa Tuple ? counts[1] : counts.passes
    fails = counts isa Tuple ? counts[2] : counts.fails
    errors = counts isa Tuple ? counts[3] : counts.errors
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
Print the report and write it to `test/reports/<name>.md`. When there is nothing to
report, nothing is written and the report of a previous run is removed.
"""
function write_report(name::AbstractString)
    file = joinpath(REPORTS, "$name.md")
    isempty(ROWS) && (rm(file; force=true); return nothing)
    lines = [
        "# Test report: $name",
        "",
        "| test | problem | modelings | status | outcome |",
        "| --- | --- | --- | --- | --- |",
    ]
    for r in ROWS
        push!(lines, "| $(r.test) | $(r.problem) | $(r.modelings) | $(r.status) | $(r.outcome) |")
    end
    text = join(lines, "\n") * "\n"
    mkpath(REPORTS)
    write(file, text)
    # lines worth reading: drafts, and checks that report instead of failing
    notable = filter(r -> r.status !== :valid || r.outcome != "see the test summary", ROWS)
    println("\nReport written to $file ($(length(ROWS)) lines, $(length(notable)) to read).")
    for r in notable
        println("  $(r.status): $(r.problem) [$(r.modelings)] $(r.test) → $(r.outcome)")
    end
    return file
end

end # module TestStatus
