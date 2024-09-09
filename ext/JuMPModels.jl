module JuMPModels

using OptimalControlProblems
using JuMP

rel_path_problems = "JuMPModels"
path = joinpath(dirname(@__FILE__), rel_path_problems)

files = filter(x -> x[(end - 2):end] == ".jl", readdir(path))
for file in files
    if file ≠ "JuMPModels.jl"
        include(joinpath(rel_path_problems, file))
    end
end

end
