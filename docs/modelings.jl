# Documentation adapters: what the documentation needs to know about each modeling. One entry
# per modeling of `OptimalControlProblems.Modelings.MODELING_SPECS`, the single place where a modeling is
# named; the page generators loop over the modelings and never name one.
#
# - `title`: display name;
# - `summary`: one sentence, shown on the Installation page and on the problem pages;
# - `packages`: what to load besides OptimalControlProblems to get and solve a problem;
# - `snippet(problem, grid_size)`: the short code to get and solve `problem`;
# - `model_start`: the first line of the model in a problem file of this modeling, whose block
#   is shown on the problem pages (see `model_excerpt`);
# - `checked_against`: the modeling whose NLP this one is checked equal to on every pull request
#   (`test/suite/comparison/test_equivalence.jl`), or `nothing`.

using OptimalControlProblems: OptimalControlProblems

const ADAPTERS = Dict{Symbol,NamedTuple}(
    :OptimalControl => (
        title="OptimalControl",
        summary="The continuous problem, written with the `@def` syntax of OptimalControl.jl, \
            discretized and solved by `OptimalControl.solve`.",
        packages=["OptimalControl", "NLPModelsIpopt"],
        snippet=(problem, grid_size) -> """
            using OptimalControl, OptimalControlProblems
            using NLPModelsIpopt

            pb = $(problem)(OptimalControlBackend())
            sol = solve(
                model(pb), :collocation, :adnlp, :ipopt;
                init=initial_guess(pb),
                grid_size=$(grid_size),
                scheme=:trapeze,
            )
            objective(sol)""",
        model_start=r"^\s*ocp = @def begin",
        checked_against=nothing,
    ),
    :OptimalControlScalarized => (
        title="OptimalControl, scalarized",
        summary="The same continuous problem, written with scalar components only, as needed \
            by the ExaModels modeler on CPU and GPU.",
        packages=["OptimalControl", "MadNLP"],
        snippet=(problem, grid_size) -> """
            using OptimalControl, OptimalControlProblems
            using MadNLP

            pb = $(problem)(OptimalControlScalarizedBackend())
            sol = solve(
                model(pb), :collocation, :exa, :madnlp;
                init=initial_guess(pb),
                grid_size=$(grid_size),
                scheme=:trapeze,
            )
            objective(sol)""",
        model_start=r"^\s*ocp = @def begin",
        checked_against=:OptimalControl,
    ),
    :JuMP => (
        title="JuMP",
        summary="A JuMP model, already discretized: the scheme and the grid are chosen when \
            the problem is built.",
        packages=["JuMP", "Ipopt"],
        snippet=(problem, grid_size) -> """
            using JuMP, Ipopt, OptimalControlProblems

            nlp = $(problem)(
                JuMPBackend();
                grid_size=$(grid_size),
                scheme=:trapeze,
            )
            set_optimizer(nlp, Ipopt.Optimizer)
            optimize!(nlp)
            objective_value(nlp)""",
        model_start=r"^\s*data = ProblemData\(",
        checked_against=:OptimalControl,
    ),
    :ModelingToolkit => (
        title="ModelingToolkit",
        summary="The continuous problem, written with ModelingToolkit.jl, solved with direct \
            shooting via Corleone.jl.",
        packages=["ModelingToolkit", "Corleone", "Ipopt"],
        snippet=(problem, grid_size) -> """
            using ModelingToolkit, OptimalControlProblems

            pb = $(problem)(ModelingToolkitBackend())
            sys = model(pb)""",
        model_start=r"^\s*ModelingToolkit\.@named sys = ModelingToolkit\.System\(",
        checked_against=:OptimalControl,
    ),
)

# A modeling without an adapter would be missing from every page: fail the build instead.
for spec in OptimalControlProblems.Modelings.MODELING_SPECS
    haskey(ADAPTERS, spec.name) ||
        error("No documentation adapter for the modeling $(spec.name) in docs/modelings.jl")
end

"""The modelings, in the order of the modeling table, as `(spec, adapter)` pairs."""
modelings() =
    [(spec, ADAPTERS[spec.name]) for spec in OptimalControlProblems.Modelings.MODELING_SPECS]

"""The modelings in which `problem` exists, whatever its status there."""
function modelings(problem::Symbol)
    return [
        (spec, adapter) for (spec, adapter) in modelings() if
        problem in OptimalControlProblems.problems(spec.backend(); status=:all)
    ]
end

"""Path of the file of `problem` in the modeling `spec`, relative to the repository root."""
problem_file(problem::Symbol, spec) = joinpath("ext", spec.directory, "$(problem).jl")

"""
The block of the model in the file `text`: from the line matching `start` to the first next
line at the same indentation that closes it (`end` or `)`), dedented. The whole file when the
start line is not found.
"""
function model_excerpt(text::AbstractString, start::Regex)
    lines = split(text, '\n')
    first = findfirst(l -> occursin(start, l), lines)
    isnothing(first) && return strip(text)
    indent = length(lines[first]) - length(lstrip(lines[first]))
    last = findnext(eachindex(lines), first + 1) do i
        l = lines[i]
        isempty(strip(l)) && return false
        length(l) - length(lstrip(l)) == indent && startswith(lstrip(l), r"end|\)")
    end
    block = lines[first:something(last, length(lines))]
    pad = " "^indent
    return join((startswith(l, pad) ? l[(indent + 1):end] : lstrip(l) for l in block), '\n')
end
