module OptimalControlProblems

using CTBase

abstract type AbstractModelBackend end
struct JuMPBackend <: AbstractModelBackend end
struct OptimalControlBackend <: AbstractModelBackend end

# weak dependencies
weakdeps = Dict(OptimalControlBackend => :OptimalControl, JuMPBackend => :JuMP)

# path to problems
path = joinpath(dirname(@__FILE__), "..", "ext", "MetaData")

# ------- Problem Definitions -------
files = filter(x -> x[(end - 2):end] == ".jl", readdir(path))
for file in files
    problem = Symbol(file[1:(end - 3)])
    code = quote
        function $problem(model_backend::T, args...; kwargs...) where {T<:AbstractModelBackend}
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
    :nh
    :nvar
    :ncon
    :minimize
    :state_name
    :costate_name
    :control_name
    :time
]

const types = [
    Union{String,Nothing},
    Union{Int,Nothing},
    Union{Int,Nothing},
    Union{Int,Nothing},
    Union{Bool,Nothing},
    Union{Vector{String}, String, Nothing}, 
    Union{Vector{String}, String, Nothing}, 
    Union{Vector{String}, String, Nothing},  
    Union{Tuple{String, String, Union{Real,Nothing}}, Nothing},
]

"""
OptimalControlProblems.metadata
---
The following keys are valid:
    - `name::String`: problem name
    - `nh::Int`: default number of discretization points
    - `nvar::Int`: number of variables
    - `ncon::Int`: number of general constraints
    - `minimize::Bool`: true if optimize == minimize
"""
const metadata = Dict()

for i in 1:number_of_problems
    file_key = Symbol(split(files[i], ".")[1])
    metadata[file_key] = Dict()
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

    # collect all the problems
    all_names = names(OptimalControlProblems; all=true)
    list_of_problems = filter(
        x ->
            isdefined(OptimalControlProblems, x) &&
                isa(getfield(OptimalControlProblems, x), Function) &&
                !startswith(string(x), "#") &&
                !(x in [:eval, :include, :available_problems]),
        all_names,
    )

    # # exclude the following problems
    # problems_to_exclude = [

    # ]
    # list_of_problems = setdiff(list_of_problems, problems_to_exclude)

    return list_of_problems
end

export JuMPBackend, OptimalControlBackend, available_problems

end
