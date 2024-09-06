module OptimalControlModels

using OptimalControlProblems
using CTBase
using CTDirect

rel_path_problems = "OptimalControlModels"
path = joinpath(dirname(@__FILE__), rel_path_problems)

files = filter(x -> x[(end - 2):end] == ".jl", readdir(path))
for file in files
    if file ≠ "OptimalControlModels.jl"
        include(joinpath(rel_path_problems, file))
    end
end

end