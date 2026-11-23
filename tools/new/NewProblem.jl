# The skeleton of a new problem (#449, T3): its metadata, its description and a model per
# modeling asked for, from the templates of `templates/`, the same for every problem. The
# skeleton passes the linter (`tools/lint/ProblemLint.jl`) but for its `TODO`s, which mark
# what is left to write: fill them, lint again, until no failure.
#
# Used by `tools/new/problem.jl` (the command line) and by `test/suite/core/test_new.jl`.
# A modeling is one of `MODELING_SPECS`; each has a template `templates/<modeling>.jl`, a
# test checks it, so adding a modeling to the package means adding its template.
module NewProblem

using OptimalControlProblems: OptimalControlProblems

const OCP = OptimalControlProblems
const TEMPLATES = joinpath(@__DIR__, "templates")

"""The modelings of the package, in the order of `MODELING_SPECS`."""
modelings() = [spec.name for spec in OCP.Modelings.MODELING_SPECS]

modeling_spec(m::Symbol) = only(s for s in OCP.Modelings.MODELING_SPECS if s.name === m)

function template(name::AbstractString, problem::Symbol)
    text = read(joinpath(TEMPLATES, name), String)
    return replace(text, "{{problem}}" => string(problem))
end

metadata_path(root, problem) = joinpath(root, "ext", "MetaData", "$(problem).jl")
description_path(root, problem) = joinpath(root, "ext", "MetaData", "$(problem).md")
function model_path(root, problem, m)
    return joinpath(root, "ext", modeling_spec(m).directory, "$(problem).jl")
end

"""The `:status` entry of a metadata file: one `:draft` (or given) status per modeling."""
function status_entry(statuses)
    lines = ["        $(m)=:$(statuses[m])," for m in modelings() if haskey(statuses, m)]
    return join(["    :status => (", lines..., "    ),"], "\n")
end

function check_modelings(ms)
    isempty(ms) && throw(ArgumentError("give at least one modeling, among $(modelings())"))
    unknown = setdiff(ms, modelings())
    isempty(unknown) || throw(
        ArgumentError("unknown modelings $(unknown), expected some of $(modelings())")
    )
    allunique(ms) || throw(ArgumentError("a modeling is given twice: $(ms)"))
    return nothing
end

function write_new(path, text)
    isfile(path) && throw(ArgumentError("$(path) exists already"))
    mkpath(dirname(path))
    write(path, text)
    return path
end

"""
    create(problem, modelings; root) -> paths

The skeleton of a new `problem` in `modelings`, each `:draft`: the metadata, the description
and one model file per modeling, under `root`. Refuses a problem that exists.
"""
function create(
    problem::Symbol, ms::AbstractVector{Symbol}; root::AbstractString=pkgdir(OCP)
)
    Base.isidentifier(problem) ||
        throw(ArgumentError("$(problem) is not a valid name of a function"))
    check_modelings(ms)
    isfile(metadata_path(root, problem)) &&
        throw(ArgumentError("$(problem) exists already: add a modeling with `add`"))
    status = status_entry(Dict(m => :draft for m in ms))
    paths = [
        write_new(
            metadata_path(root, problem),
            replace(template("metadata.jl", problem), "{{status}}" => status),
        ),
        write_new(description_path(root, problem), template("description.md", problem)),
    ]
    for m in ms
        push!(paths, write_new(model_path(root, problem, m), template("$(m).jl", problem)))
    end
    return paths
end

"""
    add(problem, modelings; root) -> paths

A model file per modeling for an existing `problem`, from the template of the modeling, and
a `:draft` status for each in its metadata. The other fields of the metadata do not depend on
the modeling. Refuses a modeling the problem has already.
"""
function add(problem::Symbol, ms::AbstractVector{Symbol}; root::AbstractString=pkgdir(OCP))
    check_modelings(ms)
    path = metadata_path(root, problem)
    isfile(path) || throw(ArgumentError("$(problem) does not exist: create it with `new`"))
    text = read(path, String)
    found = match(r"^[ \t]*:status => \(([^()]*)\),[ \t]*$"m, text)
    isnothing(found) && throw(ArgumentError("no `:status => (…),` entry found in $(path)"))
    statuses = Dict(
        Symbol(e[1]) => Symbol(e[2]) for e in eachmatch(r"(\w+)\s*=\s*:(\w+)", found[1])
    )
    present = filter(m -> haskey(statuses, m) || isfile(model_path(root, problem, m)), ms)
    isempty(present) || throw(ArgumentError("$(problem) has already $(present)"))
    paths = [
        write_new(model_path(root, problem, m), template("$(m).jl", problem)) for m in ms
    ]
    merge!(statuses, Dict(m => :draft for m in ms))
    write(path, replace(text, found.match => status_entry(statuses)))
    return [path; paths]
end

end # module NewProblem
