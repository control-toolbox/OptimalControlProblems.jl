module OptimalControlProblems

using DataFrames

include("./OptimalControlModels/OptimalControlModels.jl")
include("./JuMPModels/JuMPModels.jl")

export JuMPModels, OptimalControlModels

greet() = print("Hello World!")

path = dirname(@__FILE__)
files = filter(x -> x[(end - 2):end] == ".jl", readdir(path * "/MetaData"))
for file in files
  include("MetaData/" * file)
end
nb_problems = length(files)

const infos = [
    :name
    :nvar
    :ncon
    :minimize
    :objective_value
]

const types = [
    String
    Int
    Int
    Bool
    Real
]

"""
The following keys are valid:
    - `name::String`: problem name
    - `nvar::Int`: number of variables
    - `ncon::Int`: number of general constraints
    - `minimize::Bool`: true if optimize == minimize
    - `objective_value::Real`: objective value
"""
const meta = DataFrame(infos .=> [Array{T}(undef, nb_problems) for T in types])

for info in infos, i = 1:nb_problems
  meta[!, info][i] = eval(Meta.parse("$(split(files[i], ".")[1])_meta"))[info]
end


end
