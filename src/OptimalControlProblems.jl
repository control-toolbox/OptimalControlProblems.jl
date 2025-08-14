module OptimalControlProblems

using CTBase
import CTModels: CTModels, time_grid, state, control, costate

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
    :time
]

const types = [
    Union{String},
    Union{Int},
    Union{Bool},
    Union{Vector{String},String},
    Union{Vector{String},String},
    Union{Vector{String},String},
    Union{Tuple{String,String,Union{Real,Nothing}}},
]

"""
OptimalControlProblems.metadata

The following keys are valid:

    - `name::String`: problem name.
    - `N::Int`: default number of discretization points.
    - `minimize::Bool`: true or false depending on whether we minimise or maximise the objective function.
    - `state_name::Vector{String}`: the names of the components of the state.
    - `costate_name::Vector{String}`: the names of the differential constraints associated to each component of the costate.
    - `control_name::Vector{String}`: the names of the components of the control.
    - `time::Tuple{String, String, Union{Int, Nothing}}`: `time` is of the form `(type, name, value)` where:
        - `type` is either `final_time` or `step` depending on how the problem is modelled. Either the final time or the time step is a decision variable. If the final time is fixed, then `type="final_time"`.
        - `name` is the name of the final time variable.
        - `value` is either the value of the final time or `nothing` if it is free. If the final time is fixed, then it is simply a parameter while if it is free, then it is one of the decision variable. If `type="step"`, then `value` is the value of the time step (assuming the grid is uniform).
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
    available_problems()

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
function CTModels.time_grid(::Symbol, model)
    throw(CTBase.ExtensionError(:JuMP))
end
function CTModels.state(::Symbol, model)
    throw(CTBase.ExtensionError(:JuMP))
end
function CTModels.costate(::Symbol, model)
    throw(CTBase.ExtensionError(:JuMP))
end
function CTModels.control(::Symbol, model)
    throw(CTBase.ExtensionError(:JuMP))
end

export JuMPBackend, OptimalControlBackend, available_problems
export time_grid, state, costate, control

end
