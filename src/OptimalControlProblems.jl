module OptimalControlProblems

using CTBase
using CTDirect
import CTModels:
    CTModels,
    time_grid,
    state,
    control,
    costate,
    iterations,
    control_components,
    control_dimension,
    state_components,
    state_dimension,
    variable_components,
    variable_dimension
import ExaModels: ExaModels, ExaModel, variable, objective
using DocStringExtensions
using OrderedCollections: OrderedDict
using SolverCore
import ADNLPModels: ADNLPModels, ADNLPModel

"""
    $(TYPEDEF)

    Abstract tag selecting an optimal control problem modeling.
"""
abstract type AbstractModelBackend end

"""Select the JuMP modeling."""
struct JuMPBackend <: AbstractModelBackend end

"""Select the OptimalControl modeling."""
struct OptimalControlBackend <: AbstractModelBackend end

"""Select the scalarized OptimalControl modeling."""
struct OptimalControlScalarizedBackend <: AbstractModelBackend end

# Keep the modeling table as the single place to add a backend.
const MODELING_SPECS = (
    (name=:OptimalControl, backend=OptimalControlBackend, directory="OptimalControlModels", weakdep=:OptimalControl),
    (name=:OptimalControlScalarized, backend=OptimalControlScalarizedBackend, directory="OptimalControlScalarizedModels", weakdep=:OptimalControl),
    (name=:JuMP, backend=JuMPBackend, directory="JuMPModels", weakdep=:JuMP),
)

const MODELING_BY_TYPE = Dict(spec.backend => spec for spec in MODELING_SPECS)
const WEAKDEPS = Dict(spec.backend => spec.weakdep for spec in MODELING_SPECS)

function _modeling_spec(::T) where {T<:AbstractModelBackend}
    return get(MODELING_BY_TYPE, T) do
        throw(CTBase.IncorrectArgument("Unknown modeling backend $(T)"))
    end
end

function _status_key(::T) where {T<:AbstractModelBackend}
    return _modeling_spec(T).name
end

function _problem_file(problem::Symbol, spec)
    return joinpath(dirname(@__DIR__), "ext", spec.directory, "$(problem).jl")
end

function _problem_exists(problem::Symbol, spec)
    return isfile(_problem_file(problem, spec))
end

function _status_value(problem::Symbol, spec)
    status = METADATA_STORAGE[problem][:status]
    return get(status, spec.name, nothing)
end

function _warn_if_draft(problem::Symbol, modeling::Symbol)
    data = METADATA_STORAGE[problem]
    statuses = data[:status]
    if get(statuses, modeling, nothing) === :draft
        notes = get(data, :notes, nothing)
        suffix = isnothing(notes) ? "" : ": $(notes)"
        @warn "$(problem) is a draft for $(modeling)$(suffix)"
    end
end

function _validate_status(status::Symbol)
    status in (:valid, :draft, :all) || throw(
        CTBase.IncorrectArgument("status must be :valid, :draft, or :all; got $status"),
    )
    return status
end

function _select_problems(modelings, status::Symbol; require_all::Bool=true)
    _validate_status(status)
    selected = Symbol[]
    for problem in PROBLEM_NAMES
        statuses = METADATA_STORAGE[problem][:status]
        predicate = spec -> begin
            _problem_exists(problem, spec) &&
            (status === :all || get(statuses, spec.name, nothing) === status)
        end
        matches = (require_all ? all : any)(predicate, modelings)
        if matches
            push!(selected, problem)
        end
    end
    return sort!(selected)
end

function make_list_of_problems()
    path = joinpath(dirname(@__FILE__), "..", "ext", "MetaData")
    Base.include_dependency(path)
    files = filter(endswith(".jl"), readdir(path))
    return tuple(sort!(Symbol.(first.(splitext.(files))))...), path
end

const PROBLEM_NAMES, METADATA_PATH = make_list_of_problems()

# Metadata files and modeling directories are precompilation dependencies: adding a
# problem file must invalidate the package cache.
for spec in MODELING_SPECS
    Base.include_dependency(joinpath(dirname(@__FILE__), "..", "ext", spec.directory))
end

# ------- Problem Metadata -------
for problem in PROBLEM_NAMES
    include(joinpath(METADATA_PATH, "$problem.jl"))
end

const METADATA_INFOS = (:grid_size, :parameters)
const METADATA_TYPES = (Int, Union{Nothing,NamedTuple})
const METADATA_STORAGE = OrderedDict{Symbol,OrderedDict}()

for problem in PROBLEM_NAMES
    raw = eval(Symbol(problem, :_meta))
    statuses = get(raw, :status, NamedTuple())
    METADATA_STORAGE[problem] = OrderedDict{Symbol,Any}(
        :grid_size => raw[:grid_size],
        :parameters => raw[:parameters],
        :status => statuses,
    )
    if haskey(raw, :notes)
        METADATA_STORAGE[problem][:notes] = raw[:notes]
    end
    for (data, T) in zip(METADATA_INFOS, METADATA_TYPES)
        value = METADATA_STORAGE[problem][data]
        value isa T || error("Type mismatch: Expected $(T) for $(data), but got $(typeof(value))")
    end
end

# Generic methods are installed before weak-dependency extensions load.
for problem in PROBLEM_NAMES
    doc = """Return the `$problem` problem for a selected modeling.

Available modelings and statuses are listed by `status(:$problem)`."""
    code = quote
        @doc $doc function $(problem)(
            model_backend::T, args...; kwargs...
        ) where {T<:AbstractModelBackend}
            spec = _modeling_spec(model_backend)
            if !_problem_exists($(QuoteNode(problem)), spec)
                throw(CTBase.IncorrectArgument(
                    string($(QuoteNode(problem)), " is not available for ", spec.name,
                        "Backend; see problems(", spec.name, "Backend())"),
                ))
            end
            _warn_if_draft($(QuoteNode(problem)), spec.name)
            throw(CTBase.ExtensionError(WEAKDEPS[T]))
        end
    end
    eval(code)
    @eval export $problem
end

"""
$(TYPEDSIGNATURES)

Return the dictionary containing the metadata of all available optimal control problems.

# Example

```julia-repl
julia> metadata()
```
"""
metadata() = METADATA_STORAGE

"""
$(TYPEDSIGNATURES)

Return a dictionary containing the metadata of `problem`. 

To get specific data, the following keys are valid:

- `grid_size::Int`: the default number of steps. For example:
```julia
:grid_size => 500,
```
- `parameters::Union{Nothing,NamedTuple}`: the list of parameters. For example:
```julia
:parameters => (
    t0 = 0,
    tf = 1,
    x₁_l = 0,
    x₁_u = 0.1,
    x₁_t0 = 0,
    x₂_t0 = 1,
    x₁_tf = 0,
    x₂_tf = -1,
),
```

# Example

```julia-repl
julia> data = metadata(:beam)
julia> data[:grid_size]
500
```
"""
function metadata(problem::Symbol)
    !(problem ∈ keys(METADATA_STORAGE)) && throw(
        CTBase.IncorrectArgument(
            "There is no problem named $problem in metadata. To get the list of available problems, make julia> metadata()",
        ),
    )
    return METADATA_STORAGE[problem]
end

# ------- Available Problems Function -------
"""
$(TYPEDSIGNATURES)

Returns the list of available optimal control problems.

# Returns

- `Vector{Symbol}`: A vector of problem names as symbols.

# Example

```julia-repl
julia> OptimalControlProblems.problems()
[:problem1, :problem2, :problem3]
```
"""
function problems(; status::Symbol=:valid)::Vector{Symbol}
    return _select_problems(MODELING_SPECS, status; require_all=false)
end

function problems(backend::AbstractModelBackend; status::Symbol=:valid)::Vector{Symbol}
    return _select_problems((_modeling_spec(backend),), status)
end

function problems(
    first::AbstractModelBackend,
    rest::AbstractModelBackend...;
    status::Symbol=:valid,
)::Vector{Symbol}
    modelings = (_modeling_spec(first), (_modeling_spec(backend) for backend in rest)...)
    return _select_problems(modelings, status)
end

"""
$(TYPEDSIGNATURES)

Return the status of `problem` in each modeling where it exists.
"""
function status(problem::Symbol)
    haskey(METADATA_STORAGE, problem) || throw(
        CTBase.IncorrectArgument("There is no problem named $problem in metadata"),
    )
    return METADATA_STORAGE[problem][:status]
end

"""
$(TYPEDSIGNATURES)

Return the number of discretisation steps, from the metadata, for a given optimal control problem.

# Arguments

- `problem::Symbol`: The name of the problem.

# Returns

- `Int`: The number of discretisation steps (`N`) of the specified problem.

# Example

```julia-repl
julia> grid_size_data(:beam)
500
```
"""
function grid_size_data(problem::Symbol)
    return metadata(problem)[:grid_size]
end

"""
$(TYPEDSIGNATURES)

Merge two `Nothing` values.

# Arguments

- `::Nothing`: first argument.
- `::Nothing`: second argument.

# Returns

- `nothing::Nothing`: always returns `nothing`.

# Example

```julia-repl
julia> merge(nothing, nothing)
nothing
```
"""
merge(::Nothing, ::Nothing) = nothing

"""
$(TYPEDSIGNATURES)

Merge a `NamedTuple` with `nothing`.  

# Arguments

- `A::NamedTuple`: a named tuple to keep.
- `::Nothing`: placeholder, ignored.

# Returns

- `::NamedTuple`: returns `A` unchanged.

# Example

```julia-repl
julia> merge((a=1,), nothing)
(a = 1,)
```
"""
merge(A::NamedTuple, ::Nothing) = A

"""
$(TYPEDSIGNATURES)

Throw an error when attempting to merge `nothing` with a `NamedTuple`.  

# Arguments

- `::Nothing`: indicates there is no data to merge.
- `::NamedTuple`: the data that cannot be merged.

# Returns

- This function always throws `CTBase.UnauthorizedCall`.

# Example

```julia-repl
julia> merge(nothing, (a=1,))
ERROR: CTBase.UnauthorizedCall("There is nothing to merge.")
```
"""
function merge(::Nothing, ::NamedTuple)
    return throw(CTBase.UnauthorizedCall("There is nothing to merge."))
end

"""
$(TYPEDSIGNATURES)

Merge two `NamedTuple`s, with the second one overriding keys from the first when duplicated.

# Arguments

- `A::NamedTuple`: first set of key–value pairs.
- `B::NamedTuple`: second set of key–value pairs, takes precedence if keys overlap.

# Returns

- `::NamedTuple`: merged named tuple containing keys from both `A` and `B`.

# Example

```julia-repl
julia> merge((a=1, b=2), (b=3, c=4))
(a = 1, b = 3, c = 4)
```
"""
function merge(A::NamedTuple, B::NamedTuple)
    f(; kwargs...) = kwargs
    return NamedTuple(f(; A..., B...))
end

"""
$(TYPEDSIGNATURES)

Return the parameter set associated with a given problem.

# Arguments

- `problem::Symbol`: the name of the problem whose parameters are requested.

# Returns

- `::Union{Nothing,NamedTuple}`: the parameters of the problem, or `nothing` if none exist.

# Example

```julia-repl
julia> parameters_data(:beam)
(t0 = 0, tf = 1, ...)
```
"""
function parameters_data(problem::Symbol)
    return metadata(problem)[:parameters]
end

"""
$(TYPEDSIGNATURES)

Return the parameter set associated with a given problem, optionally merged with user-supplied parameters.

# Arguments

- `problem::Symbol`: the name of the problem.
- `parameters::Union{Nothing,NamedTuple}`: user-supplied parameters to override or extend the defaults.  
  If `nothing`, returns the default parameters unchanged.

# Returns

- `::Union{Nothing,NamedTuple}`: the merged parameters.  
  Throws `CTBase.UnauthorizedCall` if attempting to merge with a problem that has no parameters.

# Example

```julia-repl
julia> parameters_data(:beam, (tf = 2,))
(t0 = 0, tf = 2, ...)
```
"""
function parameters_data(problem::Symbol, parameters::Union{Nothing,NamedTuple})
    try
        return merge(parameters_data(problem), parameters)
    catch e
        if e isa CTBase.UnauthorizedCall
            throw(
                CTBase.UnauthorizedCall(
                    "There is no parameters to merge in problem: $problem."
                ),
            )
        else
            rethrow(e)
        end
    end
end

export JuMPBackend, OptimalControlBackend, OptimalControlScalarizedBackend, problems, status
export time_grid, state, costate, control, variable, iterations, objective
export control_components,
    control_dimension,
    state_components,
    state_dimension,
    variable_components,
    variable_dimension
export metadata, grid_size_data, parameters_data

end
