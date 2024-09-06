module OptimalControlProblems

using CTBase

# include(joinpath("./OptimalControlModels", "OptimalControlModels.jl"))
# include(joinpath("./JuMPModels", "JuMPModels.jl"))

# export JuMPModels, OptimalControlModels

# path = dirname(@__FILE__)
# files = filter(x -> x[(end - 2):end] == ".jl", readdir(path * "/MetaData"))
# for file in files
#     include(joinpath("./MetaData", file))
# end
# nb_problems = length(files)

# """
# The following keys are valid:
#     - `name::String`: problem name
#     - `nvar::Int`: number of variables
#     - `ncon::Int`: number of general constraints
#     - `minimize::Bool`: true if optimize == minimize
# """
# const infos = [
#     :name
#     :nvar
#     :ncon
#     :minimize
# ]

# const types = [
#     String
#     Int
#     Int
#     Bool
# ]

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
    problem = Symbol(file[1:end-3])
    code = quote
        function $problem(model_backend::T, args...; kwargs...) where {T <: AbstractModelBackend}
            throw(ExtensionError(weakdeps[T]))
        end
        export $problem
    end
    eval(code)
end

export JuMPBackend, OptimalControlBackend

end
