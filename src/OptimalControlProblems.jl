module OptimalControlProblems

using CTBase
import CTModels: CTModels, time_grid, state, control, costate
import ExaModels: ExaModels, variable
using DocStringExtensions
using OrderedCollections: OrderedDict

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
        @doc $doc function $problem(model_backend::T; N::Int=0) where {T<:AbstractModelBackend}
            throw(CTBase.ExtensionError(weakdeps[T]))
        end
        export $problem
    end

    eval(code)
end

# ------- Problem Metadata -------
for file in files
    include(joinpath(path, file))
end
number_of_problems = length(files)

const infos = [
    :name
    :N
    :minimise
    :state_name
    :costate_name
    :control_name
    :variable_name
    :final_time
]

const types = [
    String,
    Int,
    Bool,
    Vector{String},
    Vector{String},
    Vector{String},
    Union{Vector{String},Nothing},
    Tuple{Symbol,Union{Float64,Int}},
]

"""
OptimalControlProblems.metadata::Dict()

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
julia> OptimalControlProblems.metadata[:my_problem][:name]
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

Return the fixed final time, from the metadata, associated with a given optimal control problem.

# Arguments

- `problem::Symbol`: The name of the problem, used as a key in the global `metadata` dictionary.

# Returns

- `Float64`: The fixed final time of the specified problem.

# Example

```julia-repl
julia> final_time_data(:beam)
10.0
```
"""
function final_time_data(problem::Symbol)
    @assert metadata[problem][:final_time][1] == :fixed
    return metadata[problem][:final_time][2]
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

export JuMPBackend, OptimalControlBackend, problems
export time_grid, state, costate, control, variable
export metadata, final_time_data, steps_number_data

end
