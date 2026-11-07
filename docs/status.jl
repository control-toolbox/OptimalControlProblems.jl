# Status page: the status of every problem in every modeling (from the metadata), and the
# latest convergence report of each cron (from the `cron/` directory of the `results` branch,
# #305). The reports are downloaded at build time; when they cannot be, the page says so and
# the build goes on.

using Downloads: Downloads
using JSON: JSON
using OptimalControlProblems: OptimalControlProblems

const STATUS_FILE = "problems-status.md"
const STATUS_PATH = joinpath(@__DIR__, "src", STATUS_FILE)
const RESULTS_API = "https://api.github.com/repos/control-toolbox/OptimalControlProblems.jl/contents/cron?ref=results"
const RESULTS_BLOB = "https://github.com/control-toolbox/OptimalControlProblems.jl/blob/results/cron"

function download_string(url::AbstractString)
    io = IOBuffer()
    Downloads.download(url, io; timeout=30)
    return String(take!(io))
end

"""The cron reports of the `results` branch, `name => text`, or `nothing` if unreachable."""
function cron_reports()
    try
        entries = JSON.parse(download_string(RESULTS_API))
        files = [e for e in entries if endswith(e["name"], ".md")]
        return [
            first(splitext(e["name"])) => download_string(e["download_url"]) for e in files
        ]
    catch e
        @warn "The cron reports could not be downloaded; the status page says so" exception =
            e
        return nothing
    end
end

# The rows of the table under `heading` in a report, as dictionaries keyed by column.
function report_table(text::AbstractString, heading::AbstractString)
    lines = split(text, '\n')
    start = findfirst(==(heading), lines)
    isnothing(start) && return Dict{String,String}[]
    header = findnext(l -> startswith(l, "|"), lines, start)
    isnothing(header) && return Dict{String,String}[]
    cells(l) = strip.(split(strip(l), '|')[2:(end - 1)])
    columns = cells(lines[header])
    rows = Dict{String,String}[]
    for l in lines[(header + 2):end]
        startswith(l, "|") || break
        push!(rows, Dict(zip(columns, cells(l))))
    end
    return rows
end

function report_field(text::AbstractString, field::AbstractString)
    m = match(Regex("^$(field): (.*)\$", "m"), text)
    return isnothing(m) ? "" : m.captures[1]
end

problem_link(p) = "[`$(p)`](@ref description-$(p))"

function status_table()
    mods = modelings()
    header = join((adapter.title for (_, adapter) in mods), " | ")
    out = """
    | Problem | $(header) | Reference objective | Distinct solutions |
    | :--- |$(repeat(" :--- |", length(mods))) ---: | ---: |
    """
    for problem in OptimalControlProblems.problems(; status=:all)
        statuses = OptimalControlProblems.status(problem)
        cells = map(mods) do (spec, _)
            haskey(statuses, spec.name) ? string(statuses[spec.name]) : "—"
        end
        refs = references(problem)
        table = get(refs, "trapeze", nothing)
        value = isnothing(table) ? "—" : objective_string(table["value"])
        nsolutions = isnothing(table) ? "—" : string(length(get(table, "solutions", [])))
        out *= "| $(problem_link(problem)) | $(join(cells, " | ")) | $(value) | $(nsolutions) |\n"
    end
    return out
end

function reports_section(reports)
    isnothing(reports) && return """
        The latest reports of the crons could not be read when this page was built. They are
        on the [`results` branch]($(RESULTS_BLOB)).
        """
    out = ""
    for (name, text) in sort(reports; by=first)
        rows = report_table(text, "## Convergence (reported, not tested)")
        isempty(rows) && continue
        results = Dict{String,Int}()
        for r in rows
            results[r["result"]] = get(results, r["result"], 0) + 1
        end
        counts = join(("$(n) $(k)" for (k, n) in sort!(collect(results); by=first)), ", ")
        out *= """
        ### $(name)

        Generated $(report_field(text, "Generated")), [run]($(report_field(text, "Run"))),
        [full report]($(RESULTS_BLOB)/$(name).md). $(length(rows)) solves: $(counts).

        """
        # what deserves a look: not converged, another solution than the best, or a change
        # (`new` only says that the configuration was not in the previous run)
        notable = [
            r for r in rows if r["result"] != "converged" ||
            get(r, "solution", "best") ∉ ("best", "") ||
            get(r, "change", "") ∉ ("", "new")
        ]
        if isempty(notable)
            out *= "Every solve converged to the best known solution.\n\n"
        else
            out *= """
            | Problem | Modeling | Configuration | Platform | Result | Reason | Gap | Solution | Change |
            | :--- | :--- | :--- | :--- | :--- | :--- | ---: | :--- | :--- |
            """
            for r in notable
                out *= "| $(problem_link(r["problem"])) | $(r["modeling"]) | `$(r["configuration"])` | $(platform_name(r["platform"])) | $(r["result"]) | $(r["reason"]) | $(r["gap"]) | $(r["solution"]) | $(r["change"]) |\n"
            end
            out *= "\n"
        end
    end
    return isempty(out) ? "No convergence report yet.\n" : out
end

function generate_status_page!(path::AbstractString)
    write(
        path,
        """
# [Status](@id problems-status)

The status of each problem in each modeling, and the latest convergence report of each
cron. A problem is `valid` in a modeling once it is checked there, `draft` before; a draft
is documented but left out of `problems(backend)`. Convergence is reported, never tested:
a solve that does not converge, or that reaches another solution than the best known one,
is listed below and does not fail anything.

## Problems and modelings

$(status_table())

## Latest convergence reports

Only the solves that deserve a look are listed: not converged, another solution than the
best known one, or a change since the previous run.

$(reports_section(cron_reports()))
""",
    )
    return path
end

"""
    with_status_page(f::Function)

Generates the status page for the duration of a documentation build.
"""
function with_status_page(f::Function)
    generate_status_page!(STATUS_PATH)
    try
        return f(STATUS_FILE)
    finally
        isfile(STATUS_PATH) && rm(STATUS_PATH)
    end
end
