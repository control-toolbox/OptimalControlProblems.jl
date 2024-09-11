module OptimalControlProblems

using CTBase

abstract type AbstractModelBackend end
struct JuMPBackend <: AbstractModelBackend end
struct OptimalControlBackend <: AbstractModelBackend end

# weak dependencies
weakdeps = Dict(OptimalControlBackend => :CTDirect, JuMPBackend => :JuMP)

# path to problems
path = joinpath(dirname(@__FILE__), "..", "ext", "MetaData")

# ------- Problem Definitions -------
files = filter(x -> x[(end - 2):end] == ".jl", readdir(path))
for file in files
    problem = Symbol(file[1:(end - 3)])
    code = quote
        function $problem(model_backend::T, args...; kwargs...) where {T<:AbstractModelBackend}
            throw(ExtensionError(weakdeps[T]))
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
]

const types = [
    Union{String,Nothing},
    Union{Int,Nothing},
    Union{Int,Nothing},
    Union{Int,Nothing},
    Union{Bool,Nothing},
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

export JuMPBackend, OptimalControlBackend

end
