module OptimalControlProblems

using CTBase
using DataFrames

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
const metadata = DataFrame(infos .=> [Array{T}(undef, number_of_problems) for T in types])

for data in infos, i in 1:number_of_problems
    metadata[!, data][i] = eval(Meta.parse("$(split(files[i], ".")[1])_meta"))[data]
end

export JuMPBackend, OptimalControlBackend

end
