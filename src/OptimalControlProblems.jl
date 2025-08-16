module OptimalControlProblems

using CTBase
import CTModels: CTModels, time_grid, state, control, costate
import ExaModels: ExaModels, variable
using DocStringExtensions

abstract type AbstractModelBackend end
struct JuMPBackend <: AbstractModelBackend end
struct OptimalControlBackend <: AbstractModelBackend end
using OrderedCollections: OrderedDict

# weak dependencies
weakdeps = Dict(OptimalControlBackend => :OptimalControl, JuMPBackend => :JuMP)

# path to problems
path = joinpath(dirname(@__FILE__), "..", "ext", "MetaData")

# ------- Problem Definitions -------
files = filter(x -> x[(end - 2):end] == ".jl", readdir(path))
for file in files
    problem = Symbol(file[1:(end - 3)])
    code = quote
        function $problem(model_backend::T; N::Int=0) where {T<:AbstractModelBackend}
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
    :minimize
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
    Tuple{Symbol,Union{Float64, Int}},
]

"""
OptimalControlProblems.metadata::Dict()

The following keys are valid:

- `name::String`: the problem name.
- `N::Int`: the default number of steps.
- `minimize::Bool`: indicates whether the objective function is minimized (`true`) or maximized (`false`).
- `state_name::Vector{String}`: names of the state components.
- `costate_name::Vector{String}`: names of the differential constraints to get the costate (dual variables associated with the differential constraints).
- `control_name::Vector{String}`: names of the control components.
- `variable_name::Union{Vector{String},Nothing}`: names of the optimization variables, or `nothing` if no such variable exists. Here, "variable" refers to the optimization variable of the optimal control problem.
- `final_time::Tuple{Symbol, Union{Float64, Int}}`: of the form `(type, value_or_index)`, where:
    - `type` is either `:fixed` or `:free`.
    - `value_or_index` is the index in `variable` if the final time is free, or its value if it is fixed.
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
"""
function available_problems()

    #
    list_of_problems = []

    # collect all the problems
    files = filter(x -> x[(end - 2):end] == ".jl", readdir(path))
    for file in files
        problem = Symbol(file[1:(end - 3)])
        push!(list_of_problems, problem)
    end

    # all_names = names(OptimalControlProblems; all=true)
    # list_of_problems = filter(
    #     x ->
    #         isdefined(OptimalControlProblems, x) &&
    #         isa(getfield(OptimalControlProblems, x), Function) &&
    #         !startswith(string(x), "#") &&
    #         !(x in [:eval, :include, :available_problems]),
    #     all_names,
    # )

    # # exclude the following problems
    # problems_to_exclude = [

    # ]
    # list_of_problems = setdiff(list_of_problems, problems_to_exclude)

    return list_of_problems
end

# JuMP: getters for the time grid, state, control and costate
function time_grid(::Symbol, model)
    throw(CTBase.ExtensionError(:JuMP))
end
function state(::Symbol, model)
    throw(CTBase.ExtensionError(:JuMP))
end
function costate(::Symbol, model)
    throw(CTBase.ExtensionError(:JuMP))
end
function control(::Symbol, model)
    throw(CTBase.ExtensionError(:JuMP))
end
function variable(::Symbol, model)
    throw(CTBase.ExtensionError(:JuMP))
end

export JuMPBackend, OptimalControlBackend, available_problems
export time_grid, state, costate, control, variable

end
