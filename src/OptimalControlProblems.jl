module OptimalControlProblems

using CTBase
using CTDirect
import CTModels: CTModels, time_grid, state, control, costate, iterations
import ExaModels: ExaModels, ExaModel, variable, objective
using DocStringExtensions
using OrderedCollections: OrderedDict
using SolverCore
import ADNLPModels: ADNLPModels, ADNLPModel

# -----------------
# SHOULD NO BE HERE ("triiiiiiiiiiiiiiit !!!" - bruit de sifflet)
nlp_model(docp::CTDirect.DOCP) = docp.nlp
ocp_model(docp::CTDirect.DOCP) = docp.ocp
function build_ocp_solution(
    docp::CTDirect.DOCP, nlp_solution::SolverCore.AbstractExecutionStats
)
    nlp_model_backend = if nlp_model(docp) isa ADNLPModel
        CTDirect.ADNLPBackend()
    elseif nlp_model(docp) isa ExaModel
        CTDirect.ExaBackend()
    else
        throw(CTBase.IncorrectArgument("The NLP model is of unknown type."))
    end
    return CTDirect.build_OCP_solution(docp, nlp_solution; nlp_model=nlp_model_backend)
end

export nlp_model, ocp_model, build_ocp_solution
#

"""
$(TYPEDEF)

Abstract type for all optimal control problem model back-ends.
"""
abstract type AbstractModelBackend end

"""
$(TYPEDEF)

Back-end for modelling optimal control problems using the JuMP optimisation framework.
"""
struct JuMPBackend <: AbstractModelBackend end

"""
$(TYPEDEF)

Back-end for modelling optimal control problems using the OptimalControl.jl package.
"""
struct OptimalControlBackend <: AbstractModelBackend end

# weak dependencies
weakdeps = Dict(OptimalControlBackend => :OptimalControl, JuMPBackend => :JuMP)

# path to problems
path = joinpath(dirname(@__FILE__), "..", "ext", "MetaData")

# ------- Problem Definitions -------
files = filter(x -> x[(end - 2):end] == ".jl", readdir(path))
for file in files
    problem = Symbol(file[1:(end - 3)])
    problem_s = Symbol(problem, :_s)

    # Build the docstring string explicitly here
    doc = """
    $(TYPEDSIGNATURES)

    Defines the optimal control problem `$(string(problem))` for a given back-end.

    # Arguments

    - `model_backend::T`: The modelling back-end, subtype of `AbstractModelBackend`.
    - `N::Int=0`: Number of discretisation steps (optional).

    # Returns

    - Throws an `ExtensionError` if the required back-end is not available.

    # Example

    ```julia-repl
    julia> $(string(problem))(JuMPBackend(); N=20)
    ERROR: ExtensionError(:JuMP)
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

    Defines the optimal control problem `$(string(problem_s))` for a given back-end.

    # Arguments

    - `model_backend::T`: The modelling back-end, subtype of `AbstractModelBackend`.
    - `N::Int=0`: Number of discretisation steps (optional).

    # Returns

    - Throws an `ExtensionError` if the required back-end is not available.

    # Example

    ```julia-repl
    julia> $(string(problem_s))(JuMPBackend(); N=20)
    ERROR: ExtensionError(:JuMP)
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
for file in files
    include(joinpath(path, file))
end
number_of_problems = length(files)

const infos = [
    :N
    :state_name
    :costate_name
    :control_name
    :variable_name
    :time_grid_name
    :parameters
]

const types = [
    Int,
    Vector{String},
    Vector{String},
    Vector{String},
    Union{Vector{String},Nothing},
    Dict,
    Union{Nothing,NamedTuple},
]

"""
metadata::Dict()

Dictionary containing metadata for all available optimal control problems.

The following keys are valid:

- `name::String`: the problem name.
- `N::Int`: the default number of steps.
- `minimise::Bool`: indicates whether the objective function is minimised (`true`) or maximised (`false`).
- `state_name::Vector{String}`: names of the state components.
- `costate_name::Vector{String}`: names of the differential constraints to obtain the costate (dual variables associated with the differential constraints).
- `control_name::Vector{String}`: names of the control components.
- `variable_name::Union{Vector{String},Nothing}`: names of the optimisation variables, or `nothing` if no such variable exists.
- `final_time::Tuple{Symbol, Union{Float64, Int}}`: of the form `(type, value_or_index)`, where:
    - `type` is either `:fixed` or `:free`.
    - `value_or_index` is the index in `variable` if the final time is free, or its value if it is fixed.

# Example

```julia-repl
julia> metadata[:my_problem][:name]
"My Problem"
```
"""
const metadata = Dict()

for i in 1:number_of_problems
    file_key = Symbol(split(files[i], ".")[1])
    metadata[file_key] = OrderedDict()
    for (data, T) in zip(infos, types)
        value = eval(Meta.parse("$(file_key)_meta"))[data]
        if !(value isa T)
            error("Type mismatch: Expected $(T) for $(data), but got $(typeof(value))")
        end
        metadata[file_key][data] = value
    end
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

    #
    list_of_problems = Symbol[]

    # collect all the problems
    files = filter(x -> x[(end - 2):end] == ".jl", readdir(path))
    for file in files
        problem = Symbol(file[1:(end - 3)])
        push!(list_of_problems, problem)
    end

    # # exclude the following problems
    # problems_to_exclude = [

    # ]
    # list_of_problems = setdiff(list_of_problems, problems_to_exclude)

    return list_of_problems
end

"""
$(TYPEDSIGNATURES)

Retrieve the discretised time grid from a JuMP model.

# Arguments

- `::Symbol`: Problem name.
- `model`: JuMP model object.

# Returns

- Throws `ExtensionError(:JuMP)` since JuMP support must be extended.

# Example

```julia-repl
julia> time_grid(:problem1, model)
ERROR: ExtensionError(:JuMP)
```
"""
function time_grid(::Symbol, model)
    throw(CTBase.ExtensionError(:JuMP))
end

"""
$(TYPEDSIGNATURES)

Retrieve the state trajectory from a JuMP model.

# Arguments

- `::Symbol`: Problem name.
- `model`: JuMP model object.

# Returns

- Throws `ExtensionError(:JuMP)` since JuMP support must be extended.

# Example

```julia-repl
julia> state(:problem1, model)
ERROR: ExtensionError(:JuMP)
```
"""
function state(::Symbol, model)
    throw(CTBase.ExtensionError(:JuMP))
end

"""
$(TYPEDSIGNATURES)

Retrieve the costate (adjoint variables) from a JuMP model.

# Arguments

- `::Symbol`: Problem name.
- `model`: JuMP model object.

# Returns

- Throws `ExtensionError(:JuMP)` since JuMP support must be extended.

# Example

```julia-repl
julia> costate(:problem1, model)
ERROR: ExtensionError(:JuMP)
```
"""
function costate(::Symbol, model)
    throw(CTBase.ExtensionError(:JuMP))
end

"""
$(TYPEDSIGNATURES)

Retrieve the control trajectory from a JuMP model.

# Arguments

- `::Symbol`: Problem name.
- `model`: JuMP model object.

# Returns

- Throws `ExtensionError(:JuMP)` since JuMP support must be extended.

# Example

```julia-repl
julia> control(:problem1, model)
ERROR: ExtensionError(:JuMP)
```
"""
function control(::Symbol, model)
    throw(CTBase.ExtensionError(:JuMP))
end

"""
$(TYPEDSIGNATURES)

Retrieve optimisation variables from a JuMP model.

# Arguments

- `::Symbol`: Problem name.
- `model`: JuMP model object.

# Returns

- Throws `ExtensionError(:JuMP)` since JuMP support must be extended.

# Example

```julia-repl
julia> variable(:problem1, model)
ERROR: ExtensionError(:JuMP)
```
"""
function variable(::Symbol, model)
    throw(CTBase.ExtensionError(:JuMP))
end

"""
$(TYPEDSIGNATURES)

Retrieve objective value from a JuMP model.

# Arguments

- `::Symbol`: Problem name.
- `model`: JuMP model object.

# Returns

- Throws `ExtensionError(:JuMP)` since JuMP support must be extended.

# Example

```julia-repl
julia> objective(:problem1, model)
ERROR: ExtensionError(:JuMP)
```
"""
function objective(::Symbol, model)
    throw(CTBase.ExtensionError(:JuMP))
end

"""
$(TYPEDSIGNATURES)

Retrieve the number of iterations from a JuMP model.

# Arguments

- `::Symbol`: Problem name.
- `model`: JuMP model object.

# Returns

- Throws `ExtensionError(:JuMP)` since JuMP support must be extended.

# Example

```julia-repl
julia> iterations(:problem1, model)
ERROR: ExtensionError(:JuMP)
```
"""
function iterations(::Symbol, model)
    throw(CTBase.ExtensionError(:JuMP))
end

"""
$(TYPEDSIGNATURES)

Return the number of discretisation steps, from the metadata, for a given optimal control problem.

# Arguments

- `problem::Symbol`: The name of the problem, used as a key in the global `metadata` dictionary.

# Returns

- `Int`: The number of discretisation steps (`N`) of the specified problem.

# Example

```julia-repl
julia> steps_number_data(:beam)
500
```
"""
function steps_number_data(problem::Symbol)
    return metadata[problem][:N]
end

#
merge(::Nothing, ::Nothing) = nothing
merge(A::NamedTuple, ::Nothing) = A
merge(::Nothing, ::NamedTuple) = throw(CTBase.UnauthorizedCall("There is nothing to merge."))
function merge(A::NamedTuple, B::NamedTuple)
    f(;kwargs...) = kwargs
    return NamedTuple(f(; A..., B...))
end
function parameters_data(problem::Symbol)
    return metadata[problem][:parameters]
end
function parameters_data(problem::Symbol, parameters::Union{Nothing, NamedTuple})
    try
        return merge(parameters_data(problem), parameters)
    catch e
        if e isa CTBase.UnauthorizedCall
            throw(CTBase.UnauthorizedCall("There is no parameters to merge in problem: $problem."))
        else
            rethrow(e)
        end
    end
end

export JuMPBackend, OptimalControlBackend, problems
export time_grid, state, costate, control, variable, iterations, objective
export metadata, steps_number_data, parameters_data

end
