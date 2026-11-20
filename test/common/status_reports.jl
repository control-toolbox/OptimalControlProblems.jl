# The reports of `TestStatus` (#303, #360, #409, #314): the files written at the end of a
# run from what `status.jl` recorded: `test/reports/<report>.md` (statuses and results),
# `<report>-results.tsv`, `<report>-environment.toml` and the environments of the solves,
# `<report>-trajectories.toml`. Included in the module `TestStatus` by `status.jl`.

const REPORTS = joinpath(dirname(@__DIR__), "reports")

"""
    write_process_report(report, problem, environment, outcome; timeout)

The report of a problem whose process gave no report of its own (#435): stopped by a signal
(`:crashed`, e.g. out of memory) or after `timeout` seconds (`:timeout`). One row, an
`error` of the test `process`, so that the cron reports it for this problem only.
"""
function write_process_report(
    report::AbstractString,
    problem::AbstractString,
    environment::AbstractString,
    outcome::Symbol;
    timeout::Real,
)
    file = joinpath(REPORTS, "$report.md")
    isfile(file) && return file
    reason = outcome === :timeout ? "timeout after $(round(Int, timeout)) s" : "crashed"
    status = get(
        OptimalControlProblems.status(Symbol(problem)), Symbol(environment), :valid
    )
    mkpath(REPORTS)
    write(
        file,
        """
        # Test report: $report

        | test | problem | modelings | status | outcome |
        | --- | --- | --- | --- | --- |
        | process ($reason) | $problem | $environment | $status | error |
        """,
    )
    return file
end

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
