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

Abstract type for all optimal control problem model backends.
"""
abstract type AbstractModelBackend end

"""
$(TYPEDEF)

Backend for modelling optimal control problems using the JuMP optimisation framework.
"""
struct JuMPBackend <: AbstractModelBackend end

"""
$(TYPEDEF)

Backend for modelling optimal control problems using the OptimalControl.jl package.
"""
struct OptimalControlBackend <: AbstractModelBackend end

# weak dependencies
weakdeps = Dict(OptimalControlBackend => :OptimalControl, JuMPBackend => :JuMP)

# Create the list of problems
function make_list_of_problems()

    # path to problems
    path = joinpath(dirname(@__FILE__), "..", "ext", "MetaData")

    # ------- Problem Definitions -------
    files = filter(x -> x[(end - 2):end] == ".jl", readdir(path))

    # collect all the problems
    list_of_problems = Symbol[]
    for file in files
        problem = Symbol(file[1:(end - 3)])
        push!(list_of_problems, problem)
    end

    # exclude the following problems
    problems_to_exclude = [
        :bioreactor,
        :cart_pendulum,
        :dielectrophoretic_particle,
        :moonlander,
        :ducted_fan,
        #:robot,
        #:space_shuttle,
    ]
    list_of_problems = setdiff(list_of_problems, problems_to_exclude)

    return tuple(list_of_problems...), path
end

const LIST_OF_PROBLEMS, METADATA_PATH = make_list_of_problems()

for problem in LIST_OF_PROBLEMS
    problem_s = Symbol(problem, :_s)

    # Build the docstring string explicitly here
    doc = """
    $(TYPEDSIGNATURES)

    This method throws an `ExtensionError` and is called if the required backend is not available.
    ```
    """

    code = quote
        @doc $doc function $problem(
            model_backend::T, args...; kwargs...
        ) where {T<:AbstractModelBackend}
            throw(CTBase.ExtensionError(weakdeps[T]))
        end
        export $problem
    end

    eval(code)

    doc_s = """
    $(TYPEDSIGNATURES)

    This method throws an `ExtensionError` and is called if the required backend is not available.
    ```
    """

    code_s = quote
        @doc $doc_s function $problem_s(
            model_backend::T, args...; kwargs...
        ) where {T<:AbstractModelBackend}
            throw(CTBase.ExtensionError(weakdeps[T]))
        end
        export $problem_s
    end

    eval(code_s)
end

# ------- Problem Metadata -------
for problem in LIST_OF_PROBLEMS
    include(joinpath(METADATA_PATH, "$problem.jl"))
end

const METADATA_INFOS = [
    :grid_size
    :parameters
]

const METADATA_TYPES = [Int, Union{Nothing,NamedTuple}]

const METADATA_STORAGE = OrderedDict()

for problem in LIST_OF_PROBLEMS
    METADATA_STORAGE[problem] = OrderedDict()
    for (data, T) in zip(METADATA_INFOS, METADATA_TYPES)
        value = eval(Meta.parse("$(problem)_meta"))[data]
        if !(value isa T)
            error("Type mismatch: Expected $(T) for $(data), but got $(typeof(value))")
        end
        METADATA_STORAGE[problem][data] = value
    end
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
function problems()::Vector{Symbol}
    return Symbol[LIST_OF_PROBLEMS...]
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
    throw(CTBase.UnauthorizedCall("There is nothing to merge."))
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

export JuMPBackend, OptimalControlBackend, problems
export time_grid, state, costate, control, variable, iterations, objective
export control_components,
    control_dimension,
    state_components,
    state_dimension,
    variable_components,
    variable_dimension
export metadata, grid_size_data, parameters_data

end
