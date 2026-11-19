# Problem pages: one page per problem, generated from the files of the problem, never written
# by hand. Nothing is solved at build time: the results are those of the convergence and
# robustness crons, stored with the reference objectives (`ext/MetaData/references/`, #375),
# and the solutions those the convergence crons publish on the `results` branch, drawn in the
# browser (`ProblemPlot.vue`, read by `docs/results.jl`, #314).
#
# A page has one view per tab (`ProblemTabs.vue`): the problem (description, parameters,
# reference objective), one view per modeling where the problem exists (status, how to get
# and solve it, the model, its results), and the comparison of the modelings when there are
# two or more. The modelings come from the modeling table, through `docs/modelings.jl`.

using JSON: JSON
using Printf: @sprintf
using TOML: TOML
using OptimalControlProblems: OptimalControlProblems

const OCP = OptimalControlProblems
const REPOSITORY = "https://github.com/control-toolbox/OptimalControlProblems.jl"
const ROOT = abspath(joinpath(@__DIR__, ".."))
const PROBLEMS_DIR = joinpath(@__DIR__, "src", "problems")

# Two objectives closer than this, relative, are written once (a range otherwise).
const SAME_OBJECTIVE = 1e-9

# -----------------------------------
# Formatting
# -----------------------------------
title(problem::Symbol) = uppercasefirst(replace(string(problem), "_" => " "))

objective_string(x::Real) = @sprintf("%.10g", x)

function gap_string(g::Real)
    iszero(g) && return "0"
    return @sprintf("%.1e", g)
end

relative_gap(a, b) = OCP.References._relative_gap(a, b)

function objective_range(values)
    lo, hi = extrema(values)
    relative_gap(hi, lo) <= SAME_OBJECTIVE && return objective_string(lo)
    return objective_string(lo) * " – " * objective_string(hi)
end

status_badge(status) = """<span class="status status-$(status)">$(status)</span>"""

# `cron-JuMP-robustness-1.13-windows-latest` → `Windows`; the GPU runners carry `-gpu-`.
function platform_name(platform::AbstractString)
    occursin("-gpu-", platform) && return "GPU runner"
    occursin("windows", platform) && return "Windows"
    occursin("macos", platform) && return "macOS"
    occursin("ubuntu", platform) && return "Linux"
    return platform
end

# `modeler=adnlp solver=ipopt mu_init=0.001` → (`adnlp`, `ipopt`), (`mu_init=0.001`): the
# configuration of the convergence tests, and the options the robustness tests change.
const CONFIGURATION_KEYS = ("modeler", "solver")

function split_configuration(configuration::AbstractString)
    pairs = split(configuration)
    base = [split(p, '=')[2] for p in pairs if split(p, '=')[1] in CONFIGURATION_KEYS]
    options = [p for p in pairs if !(split(p, '=')[1] in CONFIGURATION_KEYS)]
    return join(("`$b`" for b in base), " + "),
    isempty(options) ? "default" : join(("`$o`" for o in options), " ")
end

# -----------------------------------
# Data of a problem
# -----------------------------------
"""The reference objectives of `problem`, one table per scheme, or an empty dictionary."""
function references(problem::Symbol)
    file = joinpath(ROOT, "ext", "MetaData", "references", "$(problem).toml")
    return isfile(file) ? TOML.parsefile(file) : Dict{String,Any}()
end

"""The features of `problem`, computed from its OptimalControl model, or `nothing`."""
function problem_features(problem::Symbol)
    problem in OCP.problems(OCP.OptimalControlBackend(); status=:all) || return nothing
    return OCP.features(problem)
end

function facts(problem::Symbol)
    f = problem_features(problem)
    isnothing(f) && return String[]
    items = [
        "<strong>$(f.state_dimension)</strong> state",
        "<strong>$(f.control_dimension)</strong> control",
    ]
    f.variable_dimension > 0 &&
        push!(items, "<strong>$(f.variable_dimension)</strong> variable")
    push!(items, "$(uppercasefirst(string(f.cost))) cost")
    f.initial_time === :free && push!(items, "free initial time")
    push!(items, f.final_time === :free ? "free final time" : "fixed final time")
    labels = (
        state_box="state box",
        control_box="control box",
        variable_box="variable box",
        path="path",
        boundary="boundary",
    )
    for (kind, n) in pairs(f.constraints)
        n > 0 && push!(items, "<strong>$(n)</strong> $(labels[kind])")
    end
    return items
end

# -----------------------------------
# Sections
# -----------------------------------
raw(html) = "```@raw html\n$(html)\n```\n"

anchor(problem, parts...) = join((string(problem), string.(parts)...), "-")

function parameters_section(problem::Symbol)
    params = OCP.parameters_data(problem)
    isnothing(params) &&
        return "### [Parameters](@id $(anchor(problem, :parameters)))\n\nThe problem has no parameters.\n"
    rows = join(("| `$(k)` | `$(v)` |" for (k, v) in pairs(params)), "\n")
    return """
    ### [Parameters](@id $(anchor(problem, :parameters)))

    | Parameter | Default value |
    | :--- | :--- |
    $(rows)

    Every modeling takes other values with the keyword `parameters`, merged with these:
    `$(problem)(backend; parameters=($(first(keys(params)))=…,))`.
    """
end

function reference_section(problem::Symbol, refs)
    out = "### [Reference objective](@id $(anchor(problem, :reference)))\n\n"
    if isempty(refs)
        out *= "No reference objective has been computed for this problem yet.\n"
    else
        out *= """
        The best objective among the solves that converged, for every modeling and
        configuration, on the default grid. See
        [how the references are computed]($(REPOSITORY)/blob/main/tools/references/README.md).

        | Scheme | Grid | Reference objective | Solves | Distinct solutions | Agreement |
        | :--- | ---: | ---: | ---: | ---: | :--- |
        """
        for (scheme, table) in sort!(collect(refs); by=first)
            solves = length(get(table, "results", []))
            nsolutions = length(get(table, "solutions", []))
            agreement =
                table["agreement"] ? "yes, within $(gap_string(table["spread"]))" : "no"
            out *= "| `:$(scheme)` | $(table["grid_size"]) | $(objective_string(table["value"])) | $(solves) | $(nsolutions) | $(agreement) |\n"
        end
        out *= "\n" * solutions_text(refs)
    end
    analytic = OCP.reference_solution_data(problem)
    if !isnothing(analytic)
        out *= """

        **Analytic solution.** The continuous problem has a known solution, of objective
        $(objective_string(analytic.objective)) ($(analytic.source)). The reference objective
        above is the one of the discretized problem: the gap between the two is the
        discretization error.
        """
    end
    return out
end

# Distinct solutions, said in words: the reader learns whether the problem has local solutions.
function solutions_text(refs)
    lines = String[]
    for (scheme, table) in sort!(collect(refs); by=first)
        solutions = get(table, "solutions", [])
        length(solutions) <= 1 && continue
        best = table["value"]
        others = [
            "$(objective_string(s["objective"])) (gap $(gap_string(relative_gap(s["objective"], best))), $(s["solves"]) solves)"
            for s in solutions[2:end]
        ]
        push!(
            lines,
            "With `:$(scheme)`, the solves reached $(length(solutions)) distinct solutions: " *
            "besides the best one, " *
            join(others, "; ") *
            ". " *
            "See the comparison of the modelings for which configurations reached which.",
        )
    end
    return isempty(lines) ? "" : join(lines, "\n\n") * "\n"
end

function results_rows(table, modeling::Symbol)
    results = [r for r in get(table, "results", []) if r["modeling"] == string(modeling)]
    grouped = Dict{String,Vector{Any}}()
    for r in results
        push!(get!(grouped, r["configuration"], Any[]), r)
    end
    return grouped
end

function results_section(problem::Symbol, spec, refs)
    out = "#### [Results](@id $(anchor(problem, spec.name, :results)))\n\n"
    tables = [
        (s, t) for
        (s, t) in sort!(collect(refs); by=first) if !isempty(results_rows(t, spec.name))
    ]
    if isempty(tables)
        return out *
               "No results yet for this modeling: they come from the convergence crons.\n"
    end
    for (scheme, table) in tables
        best = table["value"]
        out *= """
        Solved on the default grid ($(table["grid_size"]) steps) with the scheme `:$(scheme)`,
        by the convergence and robustness crons. The gap is relative to the reference objective,
        $(objective_string(best)); it is shown as it is, never tested.

        | Configuration | Options | Platforms | Objective | Gap to reference |
        | :--- | :--- | :--- | ---: | ---: |
        """
        rows = results_rows(table, spec.name)
        # the convergence configurations first, then the robustness ones
        order = sort!(collect(keys(rows)); by=c -> (length(split(c)), c))
        for configuration in order
            group = rows[configuration]
            base, options = split_configuration(configuration)
            platforms = join(
                sort!(unique(platform_name.(r["platform"] for r in group))), ", "
            )
            objectives = [r["objective"] for r in group]
            gap = maximum(relative_gap(o, best) for o in objectives)
            out *= "| $(base) | $(options) | $(platforms) | $(objective_range(objectives)) | $(gap_string(gap)) |\n"
        end
        out *= "\n"
    end
    return out
end

function modeling_panel(problem::Symbol, spec, adapter, refs)
    status = OCP.status(problem)[spec.name]
    file = problem_file(problem, spec)
    text = read(joinpath(ROOT, file), String)
    grid_size = OCP.grid_size_data(problem)
    notes = get(OCP.problem_metadata(problem), :notes, nothing)
    draft = if status === :draft
        """
        !!! warning "Draft in this modeling"
            The problem is not validated in this modeling yet$(isnothing(notes) ? "." : ": " * notes)
            Getting it warns, and it is left out of `problems($(nameof(spec.backend))())`.

        """
    else
        ""
    end
    return """
    ### [$(adapter.title)](@id $(anchor(problem, spec.name)))

    $(raw(status_badge(status)))
    $(adapter.summary) Load $(join(("`$p`" for p in adapter.packages), " and ")).

    $(draft)#### [Get and solve](@id $(anchor(problem, spec.name, :solve)))

    ```julia
    $(adapter.snippet(problem, grid_size))
    ```

    #### [The model](@id $(anchor(problem, spec.name, :model)))

    The model as written in [`$(file)`]($(REPOSITORY)/blob/main/$(file)), where the
    parameters are read from the metadata.

    ```julia
    $(model_excerpt(text, adapter.model_start))
    ```

    $(results_section(problem, spec, refs))

    $(solution_section(problem, [spec], [adapter]))
    """
end

# The solutions drawn in the browser, with whether they are those of the current version of
# the problem, and how they were obtained.
function solution_section(problem::Symbol, specs, adapters)
    names = [string(spec.name) for spec in specs]
    titles = Dict(
        string(spec.name) => adapter.title for (spec, adapter) in zip(specs, adapters)
    )
    id = if length(specs) == 1
        anchor(problem, only(specs).name, :solution)
    else
        anchor(problem, :compare, :solution)
    end
    heading = length(specs) == 1 ? "Solution" : "Solutions side by side"
    badges = if length(specs) == 1
        freshness_badge(problem, only(specs).name)
    else
        join(
            (freshness_badge(problem, s.name, a.title) for (s, a) in zip(specs, adapters)),
            "\n",
        )
    end
    plot = """<ClientOnly><ProblemPlot problem="$(problem)" :modelings='$(JSON.json(names))' :titles='$(JSON.json(titles))' /></ClientOnly>"""
    repro = length(specs) == 1 ? reproducibility_block(problem, only(specs).name) : ""
    return """
    #### [$(heading)](@id $(id))

    $(raw(badges * "\n" * plot * "\n" * repro))
    """
end

function compare_panel(problem::Symbol, mods, refs)
    out = "### [Comparison of the modelings](@id $(anchor(problem, :compare)))\n\n"
    names = [spec.name for (spec, _) in mods]

    # Same problem: the NLPs are compared on every pull request.
    checks = String[]
    for (spec, adapter) in mods
        other = adapter.checked_against
        (isnothing(other) || !(other in names)) && continue
        statuses = OCP.status(problem)
        drafts = [ADAPTERS[m].title for m in (spec.name, other) if statuses[m] === :draft]
        verdict = if isempty(drafts)
            "same NLP, checked on every pull request"
        else
            "not checked yet: the problem is a draft in $(join(drafts, " and "))"
        end
        push!(
            checks, "- **$(adapter.title)** and **$(ADAPTERS[other].title)**: $(verdict)."
        )
    end
    if !isempty(checks)
        out *= """
        #### [Same problem](@id $(anchor(problem, :compare, :nlp)))

        Two modelings of the same problem, discretized the same way, must give the same
        nonlinear program: same variables, constraints, objective and derivatives at the
        initial guess. This is what is compared, not the solutions: two solves are two
        optimization paths.

        $(join(checks, "\n"))

        """
    end

    for (scheme, table) in sort!(collect(refs); by=first)
        best = table["value"]
        out *= """
        #### [Results side by side, `:$(scheme)`](@id $(anchor(problem, :compare, scheme)))

        | Modeling | Status | Solves | Objective | Largest gap to reference |
        | :--- | :--- | ---: | ---: | ---: |
        """
        for (spec, adapter) in mods
            rows = results_rows(table, spec.name)
            status = string(OCP.status(problem)[spec.name])
            if isempty(rows)
                out *= "| $(adapter.title) | $(status) | 0 | — | — |\n"
                continue
            end
            objectives = [r["objective"] for group in values(rows) for r in group]
            gap = maximum(relative_gap(o, best) for o in objectives)
            out *= "| $(adapter.title) | $(status) | $(length(objectives)) | $(objective_range(objectives)) | $(gap_string(gap)) |\n"
        end
        solutions = get(table, "solutions", [])
        if !isempty(solutions)
            header = join((adapter.title for (_, adapter) in mods), " | ")
            out *= """

            Which solution each modeling reached: the number of configurations of each
            modeling whose solves reached it.

            | Solution | Objective | Gap to best | $(header) |
            | :--- | ---: | ---: |$(repeat(" ---: |", length(mods)))
            """
            for (i, s) in enumerate(solutions)
                counts = map(mods) do (spec, _)
                    n = count(
                        c -> first(split(c)) == string(spec.name), s["configurations"]
                    )
                    return n == 0 ? "—" : string(n)
                end
                label = i == 1 ? "best" : "local $(i - 1)"
                out *= "| $(label) | $(objective_string(s["objective"])) | $(gap_string(relative_gap(s["objective"], best))) | $(join(counts, " | ")) |\n"
            end
        end
        out *= "\n"
    end
    isempty(refs) && (out *= "No results yet: they come from the convergence crons.\n")
    out *= "\n" * solution_section(problem, first.(mods), last.(mods))
    return out
end

# -----------------------------------
# Page
# -----------------------------------
function problem_page(problem::Symbol)
    mods = modelings(problem)
    refs = references(problem)
    statuses = OCP.status(problem)
    description = read(joinpath(ROOT, "ext", "Descriptions", "$(problem).md"), String)

    views = Any[(id="problem", label="Problem", kind="problem")]
    for (spec, adapter) in mods
        push!(
            views,
            (
                id=string(spec.name),
                label=adapter.title,
                kind="modeling",
                status=string(statuses[spec.name]),
            ),
        )
    end
    length(mods) >= 2 && push!(views, (id="compare", label="Compare", kind="compare"))

    items = join(("<li>$(item)</li>" for item in facts(problem)), "")
    badges = join(
        (
            "<li>$(adapter.title) $(status_badge(statuses[spec.name]))</li>" for
            (spec, adapter) in mods
        ),
        "",
    )
    lists = if isempty(items)
        ""
    else
        """<ul class="problem-facts" aria-label="Features">$(items)</ul>\n"""
    end
    lists *= """<ul class="problem-facts" aria-label="Status per modeling">$(badges)</ul>"""

    page = raw("---\noutline: false\n---") * """

    # [$(title(problem))](@id description-$(problem))

    $(raw("""$(lists)
    <ProblemTabs :views='$(JSON.json(views))' />
    <ProblemPanel id="problem" default>"""))

    $(description)

    $(parameters_section(problem))

    $(reference_section(problem, refs))

    $(raw("</ProblemPanel>"))
    """
    for (spec, adapter) in mods
        page *= raw("""<ProblemPanel id="$(spec.name)">""") * "\n"
        page *= modeling_panel(problem, spec, adapter, refs) * "\n"
        page *= raw("</ProblemPanel>") * "\n"
    end
    if length(mods) >= 2
        page *= raw("""<ProblemPanel id="compare">""") * "\n"
        page *= compare_panel(problem, mods, refs) * "\n"
        page *= raw("</ProblemPanel>") * "\n"
    end
    return page
end

"""
Write the page of every problem, drafts included, under `docs/src/problems/`, and return their
paths relative to `docs/src/`.
"""
function generate_problem_pages()
    rm(PROBLEMS_DIR; recursive=true, force=true)
    mkpath(PROBLEMS_DIR)
    names = OCP.problems(; status=:all)
    for problem in names
        write(joinpath(PROBLEMS_DIR, "$(problem).md"), problem_page(problem))
    end
    return [joinpath("problems", "$(problem).md") for problem in names]
end
