# Results of the solves, for the problem pages (#314). Nothing is solved at build time: the
# results are those of the convergence crons, published on the `results` branch by
# `tools/cron/report.py` (`problems/`, `env/`, `manifest.json`; see
# `.github/workflows/README.md`). The build reads them, says on each page whether they are
# those of the current version of the problem (same fingerprint, `_fingerprint`), and copies
# what the pages draw to `docs/src/public/results/`, served with the site.
#
# `OCP_RESULTS_DIR=<dir>` reads a local results tree instead (one written by
# `tools/cron/report.py --results-dir <dir>` from local reports). Without results the build
# works: the pages say that the results are not computed yet.

using Tar: Tar

const RESULTS_DIR = joinpath(@__DIR__, "results")
const RESULTS_PUBLIC = joinpath(@__DIR__, "src", "public", "results")

"""The results read by the build: the manifest of the runs and the entries of each problem."""
struct SolveResults
    dir::Union{Nothing,String}
    manifest::Dict{String,Any}
    entries::Dict{Symbol,Vector{Dict{String,Any}}}
end

function SolveResults()
    return SolveResults(
        nothing, Dict{String,Any}(), Dict{Symbol,Vector{Dict{String,Any}}}()
    )
end

const RESULTS = Ref(SolveResults())

"""The directory of the results tree: `OCP_RESULTS_DIR`, or the `results` branch fetched into
`docs/results/`; `nothing`, with a warning, when there is none."""
function results_tree()
    local_dir = get(ENV, "OCP_RESULTS_DIR", "")
    if !isempty(local_dir)
        isdir(local_dir) && return abspath(local_dir)
        @warn "OCP_RESULTS_DIR is not a directory; the pages say the results are missing" local_dir
        return nothing
    end
    try
        rm(RESULTS_DIR; recursive=true, force=true)
        mkpath(RESULTS_DIR)
        run(`git -C $ROOT fetch --quiet --depth=1 origin results`)
        Tar.extract(
            IOBuffer(read(`git -C $ROOT archive --format=tar FETCH_HEAD`)), RESULTS_DIR
        )
        return RESULTS_DIR
    catch e
        @warn "The results branch could not be read; the pages say the results are missing" exception =
            e
        return nothing
    end
end

function load_results(dir::Union{Nothing,AbstractString})
    isnothing(dir) && return SolveResults()
    manifest_file = joinpath(dir, "manifest.json")
    manifest = isfile(manifest_file) ? JSON.parsefile(manifest_file) : Dict{String,Any}()
    entries = Dict{Symbol,Vector{Dict{String,Any}}}()
    problems_dir = joinpath(dir, "problems")
    if isdir(problems_dir)
        for problem in readdir(problems_dir)
            file = joinpath(problems_dir, problem, "result.json")
            isfile(file) && (entries[Symbol(problem)] = JSON.parsefile(file)["entries"])
        end
    end
    return SolveResults(dir, manifest, entries)
end

"""The entries of `problem`, all scopes, platforms and modelings."""
solve_entries(problem::Symbol) = get(RESULTS[].entries, problem, Dict{String,Any}[])

"""The entries of the convergence cron of `modeling` (its scope has the modeling's name)."""
function convergence_entries(problem::Symbol, modeling::Symbol)
    return [
        e for e in solve_entries(problem) if
        e["modeling"] == string(modeling) && e["scope"] == string(modeling)
    ]
end

# The platform whose trajectory a page draws, when a configuration ran on several: they agree
# to the solver tolerance, so one is enough, and the site stays small.
function platform_rank(p::AbstractString)
    return if occursin("-gpu-", p)
        3
    elseif occursin("windows", p)
        2
    elseif occursin("macos", p)
        1
    else
        0
    end
end

"""
The entries of `problem` as the page reads them: the trajectory kept for one platform per
scope, modeling and configuration (`platform_rank`), `nothing` for the others.
"""
function page_entries(problem::Symbol)
    entries = [copy(e) for e in solve_entries(problem)]
    best = Dict{Tuple{String,String,String},Dict{String,Any}}()
    for e in entries
        isnothing(e["trajectory"]) && continue
        key = (e["scope"], e["modeling"], e["configuration"])
        current = get(best, key, nothing)
        if isnothing(current) ||
            platform_rank(e["platform"]) < platform_rank(current["platform"])
            best[key] = e
        end
    end
    kept = Set(objectid(e) for e in values(best))
    for e in entries
        objectid(e) in kept || (e["trajectory"] = nothing)
    end
    return entries
end

"""
Copy what the pages draw to `docs/src/public/results/`: `<problem>.json` (the entries, with
the current fingerprint of the problem), the trajectories kept, and the environments the
entries refer to.
"""
function publish_results!(results::SolveResults)
    rm(RESULTS_PUBLIC; recursive=true, force=true)
    isnothing(results.dir) && return nothing
    mkpath(RESULTS_PUBLIC)
    environments = Set{String}()
    for problem in OCP.problems(; status=:all)
        haskey(results.entries, problem) || continue
        entries = page_entries(problem)
        for e in entries
            isnothing(e["environment"]) || push!(environments, e["environment"])
            isnothing(e["trajectory"]) && continue
            target = joinpath(RESULTS_PUBLIC, e["trajectory"])
            mkpath(dirname(target))
            cp(joinpath(results.dir, e["trajectory"]), target; force=true)
        end
        data = Dict(
            "problem" => string(problem),
            "fingerprint" => OCP._fingerprint(problem),
            "entries" => entries,
        )
        write(joinpath(RESULTS_PUBLIC, "$(problem).json"), JSON.json(data))
    end
    write(joinpath(RESULTS_PUBLIC, "thumbnails.json"), JSON.json(thumbnails(results)))
    for env in environments
        source = joinpath(results.dir, env)
        isdir(source) || continue
        target = joinpath(RESULTS_PUBLIC, env)
        mkpath(dirname(target))
        cp(source, target; force=true)
    end
    return nothing
end

# Points of a thumbnail, and its components: the shape of the solution, on a card.
const THUMBNAIL_POINTS = 80
const THUMBNAIL_COMPONENTS = 3

"""
The thumbnails of the gallery (`ProblemsGallery.vue`): for each problem with a trajectory, the
first state components of one solve (the first modeling, a converged solve, `platform_rank`),
on `THUMBNAIL_POINTS` nodes.
"""
function thumbnails(results::SolveResults)
    out = Dict{String,Any}()
    for problem in OCP.problems(; status=:all)
        order = [string(spec.name) for (spec, _) in modelings(problem)]
        candidates = [
            e for e in page_entries(problem) if !isnothing(e["trajectory"]) &&
            e["scope"] == e["modeling"] &&
            e["modeling"] in order
        ]
        isempty(candidates) && continue
        e = first(
            sort(
                candidates;
                by=e -> (
                    e["result"] != "converged",
                    findfirst(==(e["modeling"]), order),
                    platform_rank(e["platform"]),
                    e["configuration"],
                ),
            ),
        )
        trajectory = JSON.parsefile(joinpath(results.dir, e["trajectory"]))
        names = if haskey(trajectory, "names")
            trajectory["names"]["state"]
        else
            sort!(collect(keys(trajectory["state"])))
        end
        time = trajectory["time"]
        keep = unique(
            round.(Int, range(1, length(time); length=min(length(time), THUMBNAIL_POINTS)))
        )
        out[string(problem)] = Dict(
            "time" => time[keep],
            "series" => [
                Dict("name" => n, "values" => trajectory["state"][n][keep]) for
                n in first(names, THUMBNAIL_COMPONENTS)
            ],
        )
    end
    return out
end

"""
    load_results!()

Read the results of the crons and copy what the pages draw into the site (git-ignored, like
`docs/results/`), before the problem pages are written.
"""
function load_results!()
    seconds = @elapsed begin
        RESULTS[] = load_results(results_tree())
        publish_results!(RESULTS[])
    end
    n = length(RESULTS[].entries)
    @info "Results of the crons: $(n) problems, read in $(round(seconds; digits=1)) s"
    return nothing
end

# -----------------------------------
# Freshness and reproducibility, on the pages
# -----------------------------------
short_commit(sha::AbstractString) = first(sha, 7)
day(date::AbstractString) = first(date, 10)
function commit_link(sha)
    return """<a href="$(REPOSITORY)/commit/$(sha)"><code>$(short_commit(sha))</code></a>"""
end
run_link(url) = """<a href="$(url)">run $(last(split(url, '/')))</a>"""

"""
`:missing` (no result of the cron of `modeling`), `:current` (every result is for the current
version of the problem: same fingerprint) or `:stale`.
"""
function freshness(problem::Symbol, modeling::Symbol)
    entries = convergence_entries(problem, modeling)
    isempty(entries) && return :missing
    current = OCP._fingerprint(problem)
    return all(e -> get(e, "fingerprint", "") == current, entries) ? :current : :stale
end

function freshness_badge(problem::Symbol, modeling::Symbol, name::AbstractString="")
    state = freshness(problem, modeling)
    who = isempty(name) ? "" : "<strong>$(name)</strong>: "
    word(w) = isempty(name) ? uppercasefirst(w) : w
    state === :missing &&
        return """<p class="results-freshness results-missing">$(who)$(word("no")) results yet: the convergence cron of this modeling has not solved this problem.</p>"""
    entries = convergence_entries(problem, modeling)
    latest = argmax(e -> e["date"], entries)
    when = "Solved $(day(latest["date"])), $(run_link(latest["run"])), at commit $(commit_link(latest["commit"]))"
    if state === :current
        return """<p class="results-freshness results-current"><span class="results-dot" aria-hidden="true"></span>$(who)<strong>$(word("up to date"))</strong> · $(when).</p>"""
    end
    return """<p class="results-freshness results-stale"><span class="results-dot" aria-hidden="true"></span>$(who)<strong>$(word("stale"))</strong> · $(when): the problem changed since (its files, so its fingerprint); the next run of the cron updates the results.</p>"""
end

# `environment.toml` of an entry, or `nothing`.
function environment(entry)
    isnothing(get(entry, "environment", nothing)) && return nothing
    isnothing(RESULTS[].dir) && return nothing
    file = joinpath(RESULTS[].dir, entry["environment"], "environment.toml")
    return isfile(file) ? TOML.parsefile(file) : nothing
end

const ENVIRONMENT_FILES = (
    ("Project.toml", "Project.toml"),
    ("Manifest.toml", "Manifest.toml"),
    ("versioninfo.txt", "versioninfo"),
    ("status-manifest.txt", "Pkg.status"),
)

function reproducibility_block(problem::Symbol, modeling::Symbol)
    entries = convergence_entries(problem, modeling)
    isempty(entries) && return ""
    platforms = Dict{String,Dict{String,Any}}()
    for e in entries
        haskey(platforms, e["platform"]) || (platforms[e["platform"]] = e)
    end
    rows = String[]
    packages = Dict{String,Dict{String,String}}()
    order = sort!(collect(keys(platforms)); by=p -> (platform_rank(p), p))
    for p in order
        e = platforms[p]
        env = environment(e)
        files = if isnothing(e["environment"])
            "—"
        else
            join(
                (
                    """<ResultLink path="$(e["environment"])/$(file)">$(label)</ResultLink>"""
                    for (file, label) in ENVIRONMENT_FILES
                ),
                " · ",
            )
        end
        if isnothing(env)
            push!(
                rows,
                "<tr><td>$(platform_name(p))</td><td colspan=\"4\">—</td><td>$(files)</td></tr>",
            )
            continue
        end
        for (name, version) in get(env, "packages", Dict())
            get!(packages, name, Dict{String,String}())[platform_name(p)] = version
        end
        push!(
            rows,
            "<tr><td>$(platform_name(p))</td><td><code>$(env["julia"])</code></td><td>$(env["os"]) $(env["arch"])</td><td>$(get(env, "cpu", ""))</td><td>$(get(env, "threads", ""))</td><td>$(files)</td></tr>",
        )
    end
    names = [platform_name(p) for p in order]
    package_rows = join(
        (
            "<tr><td><code>$(name)</code></td>" *
            join(("<td>$(get(versions, n, "—"))</td>" for n in names), "") *
            "</tr>" for (name, versions) in sort!(collect(packages); by=first)
        ),
        "",
    )
    latest = argmax(e -> e["date"], entries)
    command = """Pkg.test(; test_args=["$(modeling)", "cron", "problem=$(problem)"])"""
    return """
    <details class="results-repro">
    <summary>Reproducibility</summary>
    <p>The results above come from $(run_link(latest["run"])) of the <code>$(modeling)</code> convergence cron, on $(day(latest["date"])), at commit $(commit_link(latest["commit"])). The environment of each platform, as the tests saw it:</p>
    <table><thead><tr><th>Platform</th><th>Julia</th><th>System</th><th>CPU</th><th>Threads</th><th>Files</th></tr></thead><tbody>$(join(rows, ""))</tbody></table>
    <table><thead><tr><th>Package</th>$(join(("<th>$(n)</th>" for n in names), ""))</tr></thead><tbody>$(package_rows)</tbody></table>
    <p>To solve again, check out the commit and run the cron tests of this problem: <code>$(command)</code>. The versions resolved then may differ from those of the run; the manifest lists those.</p>
    </details>
    """
end
