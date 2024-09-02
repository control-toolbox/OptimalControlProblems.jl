module OptimalControlProblems

include(joinpath("./OptimalControlModels", "OptimalControlModels.jl"))
include(joinpath("./JuMPModels", "JuMPModels.jl"))

export JuMPModels, OptimalControlModels

greet() = print("Hello World!")

path = dirname(@__FILE__)
files = filter(x -> x[(end - 2):end] == ".jl", readdir(path * "/MetaData"))
for file in files
    include(joinpath("./MetaData", file))
end
nb_problems = length(files)

const infos = [
    :name
    :nvar
    :ncon
    :minimize
]

const types = [
    String
    Int
    Int
    Bool
]

"""
The following keys are valid:
    - `name::String`: problem name
    - `nvar::Int`: number of variables
    - `ncon::Int`: number of general constraints
    - `minimize::Bool`: true if optimize == minimize
"""

end
