# The linter of the problems (#449, T3): what a problem must have, checked the same way for
# every problem, so that an agent or a person adding one knows what is missing. It says what
# it checks (✅/❌), what other tools check (☑️), and what is left to check by hand (ℹ️): no
# failure here does not mean that a problem is right.
#
# Used by `tools/lint/problems.jl` (the command line) and by `test/suite/core/test_lint.jl`
# (no failure allowed). The checks read the files of the problem as text and its metadata
# through the package; they are pure functions of a `ProblemSource`, which the tests build by
# hand.
module ProblemLint

using OptimalControlProblems: OptimalControlProblems

const OCP = OptimalControlProblems

"""One check: `:pass` or `:fail`, the area it belongs to, and what it says."""
struct Check
    problem::Symbol
    status::Symbol
    area::String
    message::String
end

"""The model of a problem in one modeling: its name, backend, and file text (`nothing` when
the problem has no file in this modeling)."""
struct Model
    modeling::Symbol
    backend::String
    text::Union{Nothing,String}
end

"""Everything the checks read about a problem."""
struct ProblemSource
    name::Symbol
    metadata::AbstractDict
    metadata_text::String
    description::Union{Nothing,String}
    models::Vector{Model}
end

"""The `ProblemSource` of `problem`, read from the files of the package under `root`."""
function source(problem::Symbol; root::AbstractString=pkgdir(OCP))
    ext = joinpath(root, "ext")
    read_or_nothing(path) = isfile(path) ? read(path, String) : nothing
    models = [
        Model(
            spec.name,
            string(nameof(spec.backend)),
            read_or_nothing(joinpath(ext, spec.directory, "$(problem).jl")),
        ) for spec in OCP.Modelings.MODELING_SPECS
    ]
    return ProblemSource(
        problem,
        OCP.problem_metadata(problem),
        read(joinpath(ext, "MetaData", "$(problem).jl"), String),
        read_or_nothing(joinpath(ext, "MetaData", "$(problem).md")),
        models,
    )
end

# ------------------------------------------------------------------------------------------
# Descriptions
# ------------------------------------------------------------------------------------------

"""
The outline of a description: its `###` headings, in this order. The qualitative behaviour
is optional: a page whose description has none gets a placeholder (`with_behaviour` in
`docs/problems.jl`). There is no parameters section: the page generates it from the metadata
(`:parameters`, `:parameter_info`).
"""
const OUTLINE = (
    "Mathematical formulation", "Qualitative behaviour", "Characteristics", "References"
)
const REQUIRED_HEADINGS = ("Mathematical formulation", "Characteristics", "References")

"""Whether the `###` headings of `text` follow `OUTLINE`, with every required heading."""
function follows_outline(text::AbstractString)
    headings = [m[1] for m in eachmatch(r"^### (.+?)\s*$"m, text)]
    all(in(OUTLINE), headings) || return false
    issorted(headings; by=h -> findfirst(==(h), OUTLINE)) || return false
    allunique(headings) || return false
    return all(in(headings), REQUIRED_HEADINGS)
end

"""
Whether the description `text` cites its sources: a heading `References` followed by at
least one list entry (#379).
"""
function cites_sources(text::AbstractString)
    m = match(r"^#+ References[^\n]*\n(.*?)(?=^#|\z)"ms, text)
    return !isnothing(m) && occursin(r"^\s*[-*] \S"m, m[1])
end

# ------------------------------------------------------------------------------------------
# Models
# ------------------------------------------------------------------------------------------

"""
The template of a model file, by modeling (`ext/README.md`, "Add a problem"): what every
file of the modeling contains, as `(what, regex)` pairs; `P` stands for the problem, `M` for
the modeling, `B` for its backend type.
"""
function template(modeling::Symbol, problem::Symbol, backend::AbstractString)
    p = string(problem)
    m = string(modeling)
    items = [
        ("a docstring first", r"\A\"\"\""),
        (
            "the method `OptimalControlProblems.$p(::$backend; …)`",
            Regex("function OptimalControlProblems\\.$p\\(\\s*::$backend\\b"),
        ),
        (
            "the keyword `parameters::Union{Nothing,NamedTuple}=nothing`",
            r"parameters::Union\{Nothing,NamedTuple\}=nothing",
        ),
        ("the keyword `init=nothing`", r"\binit=nothing"),
        (
            "the draft warning `_warn_if_draft(:$p, :$m)`",
            Regex("_warn_if_draft\\(\\s*:$p,\\s*:$m\\s*\\)"),
        ),
        (
            "the parameters `parameters_data(:$p, parameters)`",
            Regex("parameters_data\\(\\s*:$p,\\s*parameters\\s*\\)"),
        ),
        (
            "the initial guess of the metadata `_initial_guess(:$p, …)` (#377)",
            Regex("_initial_guess\\(\\s*:$p\\s*,"),
        ),
    ]
    if modeling in (:OptimalControl, :OptimalControlScalarized)
        push!(
            items,
            (
                "the returned `Problems._problem(:$p, :$m, …)`",
                Regex("_problem\\(\\s*:$p,\\s*:$m\\s*,"),
            ),
        )
    elseif modeling === :JuMP
        push!(
            items,
            ("the keyword `grid_size`", Regex("grid_size::Int=grid_size_data\\(:$p\\)")),
        )
        push!(items, ("the keyword `scheme`", r"scheme::Symbol=:trapeze"))
    end
    return items
end

"""The names a model file reads from the parameters: `params[:x]` and `(; x, y) = params`."""
function parameters_read(text::AbstractString)
    names = [m[1] for m in eachmatch(r"params\[:([^\]\s]+)\]", text)]
    for m in eachmatch(r"\(;\s*([^)]*?)\)\s*=\s*params\b", text)
        append!(names, strip.(split(m[1], ',')))
    end
    return unique!(filter!(!isempty, String.(names)))
end

"""How many times `name` appears in `text` as a whole name, `p.name` included (the initial
guess of the metadata reads `p.name`), not as `:name`."""
function occurrences(name::AbstractString, text::AbstractString)
    return count(
        _ -> true, eachmatch(Regex("(?<![\\w:])" * escape(name) * "(?![\\w])"), text)
    )
end

escape(name) = replace(name, r"([\\^$.|?*+()\[\]{}])" => s"\\\1")

"""The code of a line, without its comment."""
code(line) = first(split(line, '#'; limit=2))

"""
The lines of `text` that bound something with a number written in the code: a constraint
(`≤`, `≥`, `<=`, `>=`) or a `lower=`, `upper=`, `bounds=` with a numeric literal. A bound is
a parameter of the problem, so that a user can change it and the page shows it.
"""
function literal_bounds(text::AbstractString)
    found = String[]
    for line in split(text, '\n')
        c = code(line)
        occursin(r"≤|≥|<=|>=|\blower\s*=|\bupper\s*=|\bbounds\s*=", c) || continue
        occursin(r"(?<![\w.₀-₉^\[])-?\d+(\.\d+)?(?![\w.\]])", c) && push!(found, strip(c))
    end
    return found
end

"""The text of the metadata file outside its `:parameters` and `:parameter_info` blocks:
where the initial guess and the analytic solution read the parameters."""
function metadata_code(text::AbstractString)
    for key in (":parameter_info", ":parameters")
        i = findfirst(key * " =>", text)
        isnothing(i) && continue
        k = findnext('(', text, last(i))
        depth = 0
        while k <= lastindex(text)
            text[k] == '(' && (depth += 1)
            text[k] == ')' && (depth -= 1)
            depth == 0 && break
            k = nextind(text, k)
        end
        text = text[1:prevind(text, first(i))] * text[nextind(text, k):end]
    end
    return text
end

# ------------------------------------------------------------------------------------------
# Checks
# ------------------------------------------------------------------------------------------

"""Every check of the problem `src`, in order: files, description, metadata, models."""
function checks(src::ProblemSource)
    results = Check[]
    p = src.name
    add(ok, area, message) = push!(results, Check(p, ok ? :pass : :fail, area, message))

    # files: the description, and a status for each modeling with a file, only for those
    add(!isnothing(src.description), "files", "a description `ext/MetaData/$p.md`")
    statuses = src.metadata[:status]
    for model in src.models
        has_file = !isnothing(model.text)
        declared = haskey(statuses, model.modeling)
        add(
            has_file == declared,
            "files",
            if has_file
                "a status for $(model.modeling), which has a model file"
            else
                "no status for $(model.modeling), which has no model file"
            end,
        )
    end
    add(
        any(m -> !isnothing(m.text), src.models),
        "files",
        "a model in one modeling at least",
    )

    # the description: one outline, sources
    if !isnothing(src.description)
        add(
            follows_outline(src.description),
            "description",
            "the outline: $(join(OUTLINE, ", ")) (qualitative behaviour optional)",
        )
        add(
            cites_sources(src.description), "description", "a `### References` with entries"
        )
    end

    # the metadata: the unit and meaning of every parameter
    parameters = src.metadata[:parameters]
    names = isnothing(parameters) ? String[] : String.(collect(keys(parameters)))
    info = get(src.metadata, :parameter_info, nothing)
    if isnothing(info)
        add(false, "parameters", "a `:parameter_info`")
    else
        add(
            String.(collect(keys(info))) == names,
            "parameters",
            "`:parameter_info` describes exactly the parameters, in their order",
        )
        bad = [
            String(k) for (k, e) in pairs(info) if !(
                e isa NamedTuple &&
                keys(e) == (:unit, :meaning) &&
                e.unit isa Union{Nothing,String} &&
                e.meaning isa String &&
                !isempty(e.meaning)
            )
        ]
        add(
            isempty(bad),
            "parameters",
            "every entry of `:parameter_info` is `(unit, meaning)`" *
            (isempty(bad) ? "" : ": not $(join(bad, ", "))"),
        )
    end

    # the models: the template, the parameters they read, no bound written in the code
    read_somewhere = Set{String}()
    for model in src.models
        isnothing(model.text) && continue
        text = model.text
        where = "model $(model.modeling)"
        for (what, regex) in template(model.modeling, p, model.backend)
            add(occursin(regex, text), where, what)
        end
        read = parameters_read(text)
        union!(read_somewhere, read)
        unknown = setdiff(read, names)
        add(
            isempty(unknown),
            where,
            "reads parameters of the problem only" *
            (isempty(unknown) ? "" : ": not $(join(unknown, ", "))"),
        )
        unused = [n for n in read if occurrences(n, text) <= 1]
        add(
            isempty(unused),
            where,
            "uses every parameter it reads" *
            (isempty(unused) ? "" : ": not $(join(unused, ", "))"),
        )
        bounds = literal_bounds(text)
        add(
            isempty(bounds),
            where,
            "no bound written in the code" *
            (isempty(bounds) ? "" : ": $(join(("`$b`" for b in bounds), ", "))"),
        )
    end
    init_code = metadata_code(src.metadata_text)
    unread = [n for n in names if !(n in read_somewhere) && occurrences(n, init_code) == 0]
    add(
        isempty(unread),
        "parameters",
        "every parameter is read by a model or by the metadata (initial guess, solution)" *
        (isempty(unread) ? "" : ": not $(join(unread, ", "))"),
    )
    return results
end

"""The checks of `problems` (every problem of the collection by default)."""
function lint(problems=OCP.problems(; status=:all); root::AbstractString=pkgdir(OCP))
    return reduce(vcat, (checks(source(p; root)) for p in problems); init=Check[])
end

# ------------------------------------------------------------------------------------------
# Report
# ------------------------------------------------------------------------------------------

"""What other tools check (☑️), with the tool: the linter does not repeat it."""
const ELSEWHERE = (
    "the types of the metadata fields" => "the package, when it loads (`src/Metadata/loading.jl`), and `test_schema`",
    "the files of `ext/`: one metadata file and one description per problem, nothing else" => "`test_files` (`test/suite/core/test_consistency.jl`)",
    "a `:valid` problem has a reference (another `:valid` modeling or a reference objective)" => "`test_comparison_rule`",
    "the names of the components, the dynamics, the cost and the constraints agree between the modelings of a problem, where it is `:valid` in two" => "the tests of each modeling (`test_features`, the continuous comparison, `test/common/continuous.jl`)",
    "the NLP of JuMP and that of CTDirect agree" => "the comparison cron (#441)",
    "the problem is solved, and to the reference objective" => "the convergence crons (#360), reported, not tested",
)

"""What no tool checks (ℹ️): left to the person who writes or reviews the problem."""
const MANUAL = (
    "the mathematical formulation of the description is the one of the models",
    "the meaning and the unit of each parameter (`:parameter_info`) are right",
    "the sources of the description are those of the problem and its values",
    "the qualitative behaviour and the characteristics of the description are true",
    "the signs of the costates and of the duals (`rules/signs.md`, #326, #430)",
)

"""
The report of `results` in Markdown: a line per problem, the failures, what other tools
check and what is left to check by hand. With `verbose`, every check.
"""
function report(results::Vector{Check}; verbose::Bool=false)
    io = IOBuffer()
    problems = unique(r.problem for r in results)
    failed = filter(r -> r.status === :fail, results)
    println(
        io,
        "## Problem lint — $(length(problems)) problem(s), $(length(results)) checks: ",
        isempty(failed) ? "✅ no failure" : "❌ $(length(failed)) failure(s)",
        "\n",
    )
    println(io, "| Problem | ✅ | ❌ |\n| --- | --- | --- |")
    for p in problems
        mine = filter(r -> r.problem === p, results)
        n = count(r -> r.status === :fail, mine)
        println(io, "| `$p` | $(length(mine) - n) | $(n == 0 ? "" : n) |")
    end
    shown = verbose ? results : failed
    if !isempty(shown)
        println(io, "\n### ", verbose ? "Checks" : "Failures", "\n")
        println(io, "| | Problem | Area | Check |\n| --- | --- | --- | --- |")
        for r in shown
            mark = r.status === :pass ? "✅" : "❌"
            println(io, "| $mark | `$(r.problem)` | $(r.area) | $(r.message) |")
        end
    end
    println(io, "\n### ☑️ Checked by other tools, not by this linter\n")
    for (what, tool) in ELSEWHERE
        println(io, "- ", what, ": ", tool, ".")
    end
    println(io, "\n### ℹ️ To check by hand, for each problem\n")
    for what in MANUAL
        println(io, "- ", what, ".")
    end
    println(
        io,
        "\nNo failure means that the automatic checks above pass, not that a problem is ",
        "right: the points checked by other tools and by hand are not covered here.",
    )
    return String(take!(io))
end

end # module ProblemLint
