module OptimalControlModels

using OptimalControlProblems
using OptimalControl
using DocStringExtensions
using OrderedCollections: OrderedDict

# include problems files
rel_path_problems = "OptimalControlModels"
path = joinpath(dirname(@__FILE__), rel_path_problems)
files = filter(x -> x[(end - 2):end] == ".jl", readdir(path))
for file in files
    include(joinpath(rel_path_problems, file))
end

# include problems files (_s versions)
rel_path_problems = "OptimalControlModels_s"
path = joinpath(dirname(@__FILE__), rel_path_problems)
files = filter(x -> x[(end - 2):end] == ".jl", readdir(path))
for file in files
    include(joinpath(rel_path_problems, file))
end

end
