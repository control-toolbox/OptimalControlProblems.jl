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
# run for the documentation (#314).
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
            trajectory=nothing)

Record the result of a solve of `problem` in the modeling of `backend`, discretized on
`grid_size` steps with `scheme`. `modeler`, `solver` and `options` are the `settings` of the
solve modules, the solver status the `outcome` of a solution, `trajectory` the
`trajectory` of the solution.
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
)
    reference = OptimalControlProblems.reference_objective(problem; grid_size, scheme)
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
                OptimalControlProblems.solution_kind(problem, objective; grid_size, scheme)
            else
                nothing
            end,
            OptimalControlProblems._fingerprint(problem),
            trajectory,
        ),
    )
    return nothing
end

modeling(backend) = OptimalControlProblems._modeling_spec(backend).name

# Points kept on the time grid of a trajectory: enough to draw it, small enough to publish.
const TRAJECTORY_POINTS = 501

"""
The trajectory of a solution `x` (an OptimalControl solution or a solved JuMP model, read
with the accessors they share), for the plots of the documentation (#314): the time grid,
at most `TRAJECTORY_POINTS` of its nodes, the state, the control and the costate at these
nodes by component, the variable, and the names of the components in the order of the model
(`names`: a dictionary loses it). The costate is the one of the solver: for JuMP its sign
is the opposite of OptimalControl's (#326). Values are rounded to 6 significant digits:
enough to draw, and stable from one run to the next.
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
(#360) as a second table and as `test/reports/<name>-results.tsv`, the environment of the run
(`<name>-environment.toml`, `<name>-env/`) and the trajectories of the solutions
(`<name>-trajectories.toml`). When there is nothing to report, nothing is written and the
files of a previous run are removed.
"""
function write_report(name::AbstractString)
    file = joinpath(REPORTS, "$name.md")
    tsv = joinpath(REPORTS, "$name-results.tsv")
    environment = joinpath(REPORTS, "$name-environment.toml")
    env_dir = joinpath(REPORTS, "$name-env")
    trajectories = joinpath(REPORTS, "$name-trajectories.toml")
    rm(tsv; force=true)
    rm(environment; force=true)
    rm(env_dir; force=true, recursive=true)
    rm(trajectories; force=true)
    isempty(ROWS) && isempty(RESULTS) && (rm(file; force=true); return nothing)
    lines = [
        "# Test report: $name",
        "",
        "| test | problem | modelings | status | outcome |",
        "| --- | --- | --- | --- | --- |",
    ]
    for r in ROWS
        push!(
            lines,
            "| $(r.test) | $(r.problem) | $(r.modelings) | $(r.status) | $(r.outcome) |",
        )
    end
    mkpath(REPORTS)
    isempty(RESULTS) || append!(lines, results_table(tsv))
    isempty(RESULTS) || write_environment(environment)
    isempty(RESULTS) || write_environment_files(env_dir)
    write_trajectories(trajectories)
    text = join(lines, "\n") * "\n"
    write(file, text)
    # lines worth reading: drafts, and checks that report instead of failing
    notable = filter(r -> r.status !== :valid || r.outcome != "see the test summary", ROWS)
    println(
        "\nReport written to $file ($(length(ROWS)) lines, $(length(notable)) to read)."
    )
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
        "test",
        "problem",
        "modeling",
        "configuration",
        "grid_size",
        "scheme",
        "result",
        "reason",
        "objective",
        "iterations",
        "time",
        "gap",
        "solution",
    ]
    function cells(r, fmt)
        return [
            r.test,
            string(r.problem),
            r.modelings,
            r.configuration,
            string(r.grid_size),
            string(r.scheme),
            r.converged ? "converged" : "not_converged",
            r.reason,
            fmt(r.objective),
            string(r.iterations),
            short(r.time),
            short(r.gap),
            isnothing(r.solution) ? "" : string(r.solution),
        ]
    end
    # how each solve was configured (#409): in the file only, too long for the table
    extra = ["modeler", "solver", "options", "violation", "fingerprint"]
    function extra_cells(r)
        return [
            isnothing(r.modeler) ? "" : string(r.modeler),
            string(r.solver),
            r.options,
            full(r.violation),
            r.fingerprint,
        ]
    end
    open(tsv, "w") do io
        println(io, join([header; extra], "\t"))
        return foreach(
            r -> println(io, join([cells(r, full); extra_cells(r)], "\t")), RESULTS
        )
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
    println(
        "\nResults: $(length(RESULTS)) solves, $not_converged not converged (reported, not tested).",
    )
    return lines
end

"""
`options` as an inline TOML table, one cell of the results file read back with `TOML.parse`
(#409): numbers and booleans as they are, anything else (a symbol, an enumeration value) as
its string.
"""
function inline_toml(options::NamedTuple)
    value(x::Bool) = string(x)
    value(x::Integer) = string(x)
    function value(x::AbstractFloat)
        isnan(x) && return "nan"
        isinf(x) && return x > 0 ? "inf" : "-inf"
        return repr(Float64(x))
    end
    value(x) = repr(string(x))
    return "{" * join(("$k = $(value(v))" for (k, v) in pairs(options)), ", ") * "}"
end

# Packages whose versions say how a problem was discretized and solved, besides the direct
# dependencies of the test environment, which do not include them: the discretization, the
# modelers, the solvers and their binaries. Recorded when the environment has them.
const KEY_DEPENDENCIES = (
    "CTDirect",
    "CTModels",
    "CTSolvers",
    "ADNLPModels",
    "ExaModels",
    "MadNLP",
    "NLPModelsIpopt",
    "Ipopt_jll",
    "MUMPS_seq_jll",
)

"""
Write the environment of the run to `file` (#409): Julia, the machine, the commit and the
run when known, and the versions of the direct dependencies of the test environment and of
`KEY_DEPENDENCIES`. The full manifest of a cron run is kept with its results (#314).
"""
function write_environment(file::AbstractString)
    packages = Dict{String,String}()
    for (_, info) in Pkg.dependencies()
        (info.is_direct_dep || info.name in KEY_DEPENDENCIES) || continue
        isnothing(info.version) || (packages[info.name] = string(info.version))
    end
    run =
        if all(
            haskey.(Ref(ENV), ("GITHUB_SERVER_URL", "GITHUB_REPOSITORY", "GITHUB_RUN_ID"))
        )
            "$(ENV["GITHUB_SERVER_URL"])/$(ENV["GITHUB_REPOSITORY"])/actions/runs/$(ENV["GITHUB_RUN_ID"])"
        else
            "local"
        end
    data = Dict{String,Any}(
        "julia" => string(VERSION),
        "os" => string(Sys.KERNEL),
        "arch" => string(Sys.ARCH),
        "cpu" => String(strip(first(Sys.cpu_info()).model)),
        "threads" => Threads.nthreads(),
        "commit" => get(ENV, "GITHUB_SHA", commit()),
        "run" => run,
        "packages" => packages,
    )
    open(file, "w") do io
        println(
            io,
            "# Environment of the solves of this report (#409); generated by the tests.\n",
        )
        return TOML.print(io, data; sorted=true)
    end
    return file
end

"""
Write the files of the environment of the run to the directory `dir` (#314), for the
Reproducibility block of the documentation: `Project.toml` and `Manifest.toml` of the test
environment, `versioninfo.txt`, `status.txt` (`Pkg.status()`) and `status-manifest.txt`
(`Pkg.status(; mode=Pkg.PKGMODE_MANIFEST)`).
"""
function write_environment_files(dir::AbstractString)
    mkpath(dir)
    project = Base.active_project()
    manifest = joinpath(dirname(project), "Manifest.toml")
    cp(project, joinpath(dir, "Project.toml"); force=true)
    isfile(manifest) && cp(manifest, joinpath(dir, "Manifest.toml"); force=true)
    open(io -> InteractiveUtils.versioninfo(io), joinpath(dir, "versioninfo.txt"), "w")
    # the environment is a temporary copy: its path says nothing and would be published
    without_path(text) = replace(text, dirname(project) => "test environment")
    status(; kwargs...) = without_path(sprint(io -> Pkg.status(; io=io, kwargs...)))
    write(joinpath(dir, "status.txt"), status())
    write(joinpath(dir, "status-manifest.txt"), status(; mode=Pkg.PKGMODE_MANIFEST))
    return dir
end

"""
Write the trajectories of the results that have one to `file` (#314), one table per solve:
problem, modeling, configuration, fingerprint, and the `trajectory`. Nothing is written when
no result has a trajectory.
"""
function write_trajectories(file::AbstractString)
    entries = [
        Dict{String,Any}(
            "problem" => string(r.problem),
            "modeling" => r.modelings,
            "configuration" => r.configuration,
            "fingerprint" => r.fingerprint,
            "trajectory" => r.trajectory,
        ) for r in RESULTS if !isnothing(r.trajectory)
    ]
    isempty(entries) && return nothing
    open(file, "w") do io
        println(
            io,
            "# Trajectories of the solutions of this report (#314); generated by the tests.\n",
        )
        return TOML.print(io, Dict("solve" => entries); sorted=true)
    end
    return file
end

"""The commit of the package checkout, or `unknown` outside a git checkout."""
function commit()
    root = pkgdir(OptimalControlProblems)
    try
        return readchomp(`git -C $root rev-parse HEAD`)
    catch
        return "unknown"
    end
end

end # module TestStatus
